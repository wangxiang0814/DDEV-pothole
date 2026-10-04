"""Deterministic sensor trials and independent plant-truth acceptance logs."""
from .config import ObservationNoiseConfig

from collections import deque

import numpy as np

from .closed_loop import WheelLoadFilter
from .support_geometry import assess_support, _lambda




class ObservationNoise:
    def __init__(self, config=ObservationNoiseConfig()):
        self.config = config
        self.rng = np.random.default_rng(config.seed)
        self.last_tick_s = None
        self.offsets = {}
        self.packets = deque()
        self.measurement_time_s = None
        self.measurement_age_s = 0.
        self.scales = {**{f'Fz_{w}': config.load_std_n for w in ('L1', 'R1', 'L2', 'R2')},
                       'Yo': config.lateral_std_m, 'Yaw': config.attitude_std_deg,
                       'Roll_E': config.attitude_std_deg, 'Pitch': config.attitude_std_deg,
                       'Vx': config.speed_std_kph}

    def observe(self, now_s, truth):
        new_tick = (self.last_tick_s is None or
                    now_s - self.last_tick_s >= self.config.period_s - 1e-8)
        if new_tick:
            self.last_tick_s = now_s
            self.offsets = {key: float(np.clip(self.rng.normal(),
                            -self.config.clip_sigma, self.config.clip_sigma)) * scale
                            for key, scale in self.scales.items()}
        observed = dict(truth)
        for key, offset in self.offsets.items():
            if key in observed:
                observed[key] += offset
                # Existing load estimators require nonnegative wheel loads.
                if key.startswith('Fz_'):
                    observed[key] = max(0., observed[key])
        if self.config.feedback_delay_s > 0.:
            if new_tick:
                self.packets.append((now_s, observed))
            cutoff = now_s - self.config.feedback_delay_s
            # Keep the latest packet at or before cutoff and all newer packets.
            # Before the history fills, the first available sample is held.
            while len(self.packets) > 1 and self.packets[1][0] <= cutoff + 1e-10:
                self.packets.popleft()
            self.measurement_time_s, observed = self.packets[0]
        else:
            self.measurement_time_s = now_s
        self.measurement_age_s = now_s - self.measurement_time_s
        return dict(observed)


class TruthAudit:
    def __init__(self, *, tyre_radius_m, lambda_safe, cutoff_hz=5.):
        self.radius = tyre_radius_m
        self.lambda_safe = lambda_safe
        self.filter = WheelLoadFilter(cutoff_hz=cutoff_hz, wheel_count=4)
        self.rows = []

    def capture(self, observed_row, truth, *, lifted_corner):
        row = dict(observed_row)
        for key in ('rr_ready', 'rr_safe'):
            if key in row:
                row[f'observed_{key}'] = row.pop(key)
        wheels = ('FL', 'FR', 'RL', 'RR')
        truck = ('L1', 'R1', 'L2', 'R2')
        raw = np.array([truth[f'Fz_{w}'] for w in truck])
        loads = self.filter.update(row['time_s'], raw)
        contacts = {c: (truth[f'Xctc_{w}i'], truth[f'Yctc_{w}i'])
                    for c, w in zip(wheels, truck)}
        com = (truth['XCG_TM'], truth['YCG_TM'])
        assessment = assess_support(contacts, dict(zip(wheels, loads)), com,
                                    self.lambda_safe, lifted_corner=lifted_corner)
        triangle = tuple(contacts[c] for c in assessment.support_corners)
        row.update(vx_kph=truth['Vx'], yo_m=truth['Yo'], yaw_deg=truth['Yaw'],
                   roll_deg=truth['Roll_E'], pitch_deg=truth['Pitch'],
                   x_fr_m=truth['X_R1'], x_rr_m=truth['X_R2'],
                   zmp_lambda_min=assessment.lambda_min,
                   com_lambda_min=min(_lambda(triangle, com)),
                   zmp_x_m=assessment.zmp_xy[0], zmp_y_m=assessment.zmp_xy[1],
                   zmp_edge_distance_m=assessment.edge_distance_m,
                   com_x_m=com[0], com_y_m=com[1],
                   min_support_n=min(loads[wheels.index(c)] for c in assessment.support_corners),
                   fr_top_clearance_m=truth['Z_R1'] - self.radius,
                   rr_clearance_m=truth['Z_R2'] - self.radius)
        for i, (corner, wheel) in enumerate(zip(wheels, truck)):
            row[f'fz_{corner.lower()}_n'] = float(raw[i])
            row[f'fz_{corner.lower()}_filtered_n'] = float(loads[i])
            row[f'travel_{corner.lower()}_mm'] = truth[f'Jnc_{wheel}']
        self.rows.append(row)
        return row


class NoisyFeedbackTrial:
    """Wrap an unchanged control loop; audit only when it actually logs a tick."""
    def __init__(self, controller, export_names, front_config, config=ObservationNoiseConfig()):
        self.controller = controller
        self.export_names = export_names
        self.noise = ObservationNoise(config)
        self.path_reference_yo_m = None
        self.path_reference_yaw_deg = None
        self.front_truth = TruthAudit(tyre_radius_m=front_config.tyre_radius_m,
                                     lambda_safe=front_config.lambda_target,
                                     cutoff_hz=config.truth_load_cutoff_hz)
        self.rear_truth = TruthAudit(tyre_radius_m=controller.rear.config.tyre_radius_m,
                                    lambda_safe=controller.rear.config.lambda_safe,
                                    cutoff_hz=config.truth_load_cutoff_hz)

    def __call__(self, now_s, exports):
        truth = dict(zip(self.export_names, exports))
        observed = self.noise.observe(now_s, truth)
        if hasattr(self.controller, 'set_feedback_timestamp'):
            self.controller.set_feedback_timestamp(now_s, self.noise.measurement_time_s)
        front_count = len(self.controller.front.rows)
        rear_count = len(self.controller.rear.rows)
        command = self.controller(now_s, [observed[key] for key in self.export_names])
        if (self.path_reference_yo_m is None and
                self.controller.front.initial_yo_m is not None):
            self.path_reference_yo_m = truth['Yo']
            self.path_reference_yaw_deg = truth['Yaw']
        timing = dict(measurement_time_s=self.noise.measurement_time_s,
                      measurement_age_s=self.noise.measurement_age_s)
        if len(self.controller.front.rows) > front_count:
            self.controller.front.rows[-1].update(timing)
            self.front_truth.capture(self.controller.front.rows[-1], truth, lifted_corner='FR')
        if len(self.controller.rear.rows) > rear_count:
            self.controller.rear.rows[-1].update(timing)
            self.rear_truth.capture(self.controller.rear.rows[-1], truth, lifted_corner='RR')
        return command
