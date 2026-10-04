"""Online local three-support allocation around the existing expert feedforward."""
from __future__ import annotations

from dataclasses import replace
import math
import time

import numpy as np

from .config import SUPPORT_QP
from .load_allocator import allocate_loads
from .quintic_trajectory import quintic_step
from .support_geometry import assess_support, _lambda
from .suspension_allocator import allocate_support_increment

ORDER = ('FL', 'FR', 'RL', 'RR')
WHEELS = ('L1', 'R1', 'L2', 'R2')


class SupportAllocationFeedback:
    """50 Hz local allocation, with a continuous handoff to the existing recovery."""
    def __init__(self, *, mode_models, active, period_s, front_limits,
                 rear_limits, config=SUPPORT_QP, transitions=False):
        self.models = mode_models
        self.active = active
        self.period_s = period_s
        self.front_limits = front_limits
        self.rear_limits = rear_limits
        self.config = config
        self.transitions = transitions
        self.phase_counts = {}
        self.stage = None
        self.phase = None
        self.correction = np.zeros(3)
        self.last_update_s = None
        self.last_applied = np.zeros(4)
        self.last_application_s = None
        self.abort_max_output_slew_n_s = 0.
        self.abort_active = False
        self.filtered = None
        self.target_att = None
        self.release_start_s = None
        self.release_correction = None
        self.failure_start_s = None
        self.longest_failure_s = 0.
        self.solve_times_ms = []
        self.status_counts = {}
        self.telemetry = self._blank()

    @staticmethod
    def _blank():
        return {'load_qp_status': 'DISABLED', 'support_qp_status': 'DISABLED',
                'support_qp_cost': 0., 'support_qp_predicted_margin': 0.,
                'support_qp_solve_ms': 0., 'support_qp_failure_s': 0.,
                'support_qp_clearance_floor_m': 0.,
                'support_qp_roll_target_deg': 0., 'support_qp_pitch_target_deg': 0.,
                'support_qp_swing_gain_weight': 0.,
                **{f'allocation_corr_{c.lower()}_n': 0. for c in ORDER},
                **{f'allocation_ref_{c.lower()}_n': 0. for c in ORDER}}

    def apply(self, now_s, x, command, *, stage, phase):
        base = np.asarray(command, dtype=float).copy()
        if base.shape != (9,) or stage not in ('FR', 'RR'):
            raise ValueError('full-cycle command/stage required')
        support = np.array([i for i, c in enumerate(ORDER) if c != stage])
        if self.stage != stage:
            self.stage = stage
            self.phase = None
            self.correction[:] = 0.
            self.filtered = None
            self.target_att = None
            self.last_update_s = None
            self.release_start_s = None
            self.release_correction = None
            self.failure_start_s = None
            self.abort_active = False
        active_phases = ('THREE_WHEEL_HOLD', 'CRAWL', 'STOP') if stage == 'FR' else ('RR_HOLD', 'RR_CRAWL', 'RR_STOP', 'RR_ABORT_STOP', 'RR_ABORT_EXIT')
        transition_phases = ('PRELOAD_SHIFT', 'LIFTING') if stage == 'FR' else ('RR_PRELOAD', 'RR_LIFTING', 'RR_POSTURE')
        transition = self.transitions and phase in transition_phases
        preload = phase in ('PRELOAD_SHIFT', 'RR_PRELOAD')
        lowering = phase in ('LOWERING', 'RETURN_TO_FOUR_WHEEL', 'RR_LOWERING', 'RR_RETURN')
        if stage == 'RR' and phase == 'RR_ABORT_STOP' and self.phase != 'RR_ABORT_STOP':
            self.abort_active = True
            # RR's recovery snapshot already includes our last applied inputs.
            self.correction[:] = 0.
            self.release_start_s = None
            self.release_correction = None
            self.failure_start_s = None
            if self.target_att is not None:
                self.target_att[0] = np.deg2rad(self.rear_limits.posture_target_roll_deg)
        elif lowering and self.release_start_s is None:
            self.release_start_s = now_s
            self.release_correction = (self.last_applied[support] - base[4:8][support]
                if self.phase == 'RR_ABORT_STOP' else self.correction.copy())
        if lowering:
            self.correction = self.release_correction * (1. - quintic_step(
                now_s, self.release_start_s, self.config.release_s)[0])
        eligible = phase in active_phases or transition
        waiting_status = 'WAITING_CONTACT'
        if transition and preload:
            # The established preload loop first creates the three-contact
            # region. This local model must not be applied to fully loaded lift wheels.
            limit = self.front_limits.fr_ready_load_n if stage == 'FR' else self.rear_limits.wheel_unloaded_n
            eligible = x[f'Fz_{WHEELS[ORDER.index(stage)]}'] <= limit
            if eligible:
                contacts = {c: (x[f'Xctc_{w}i'], x[f'Yctc_{w}i']) for c, w in zip(ORDER, WHEELS)}
                loads = {c: max(0., x[f'Fz_{w}']) for c, w in zip(ORDER, WHEELS)}
                geometry = assess_support(contacts, loads, (x['XCG_TM'], x['YCG_TM']),
                                          self.config.lambda_floor, lifted_corner=stage)
                com_margin = min(_lambda([contacts[c] for c in geometry.support_corners], geometry.com_xy))
                eligible = (geometry.safe_inside and com_margin >= self.config.lambda_floor and
                            min(loads[c] for c in geometry.support_corners) >= self.config.support_floor_n)
                if not eligible:
                    waiting_status = 'WAITING_TRIANGLE'
        due = self.last_update_s is None or now_s - self.last_update_s >= self.period_s - 1e-8
        if eligible and due:
            dt = self.period_s if self.last_update_s is None else now_s - self.last_update_s
            self.last_update_s = now_s
            observed = np.r_[[max(0., x[f'Fz_{w}']) for w in WHEELS],
                              np.deg2rad([x['Roll_E'], x['Pitch']])]
            alpha = dt / (dt + 1. / (2 * math.pi * self.config.filter_cutoff_hz))
            self.filtered = observed if self.filtered is None else self.filtered + alpha * (observed - self.filtered)
            if self.target_att is None and not transition:
                self.target_att = self.filtered[4:].copy()
                if stage == 'RR' and (self.transitions or phase == 'RR_ABORT_STOP'):
                    # Do not freeze a transient posture overshoot as the hold goal.
                    self.target_att[0] = np.deg2rad(self.rear_limits.posture_target_roll_deg)
            contacts = {c: (x[f'Xctc_{w}i'], x[f'Yctc_{w}i']) for c, w in zip(ORDER, WHEELS)}
            com = (x['XCG_TM'], x['YCG_TM'])
            loads = self.filtered[:4]
            assessment = assess_support(contacts, dict(zip(ORDER, loads)), com,
                                         self.config.lambda_target, lifted_corner=stage)
            low = np.full(4, self.config.support_floor_n)
            low[ORDER.index(stage)] = 0.
            high = np.full(4, self.config.max_load_n)
            high[ORDER.index(stage)] = 0.
            begin = time.perf_counter()
            allocation = allocate_loads(list(contacts.values()), float(sum(loads)),
                                        assessment.target_zmp_xy, loads,
                                        lifted_corner=stage, lift_target_n=0.,
                                        lower_n=low, upper_n=high)
            current_limits = self.front_limits if stage == 'FR' else self.rear_limits
            att_bound = current_limits.attitude_limit_deg
            rebound = (current_limits.moving_rebound_abort_mm if stage == 'FR'
                       else current_limits.travel_rebound_abort_mm)
            jounce = (current_limits.jounce_abort_mm if stage == 'FR'
                      else current_limits.travel_jounce_abort_mm)
            cfg = replace(self.config, attitude_limit_deg=att_bound,
                           travel_lower_m=rebound / 1000., travel_upper_m=jounce / 1000.)
            if phase in ('RR_ABORT_STOP', 'RR_ABORT_EXIT'):
                # Hard braking needs faster damping than steady crawl. Keep
                # the simulated actuator force, travel and contact bounds.
                cfg = replace(cfg, force_slew_n_s=cfg.abort_force_slew_n_s,
                              correction_limit_n=(cfg.abort_correction_limit_n
                                if self.rear_limits.allow_abort_exit else cfg.correction_limit_n))
            height = x[f'Z_{WHEELS[ORDER.index(stage)]}'] - cfg.tyre_radius_m
            if transition:
                cfg = replace(cfg, clearance_floor_m=min(cfg.clearance_floor_m,
                    height - cfg.transition_clearance_slack_m))
            att_target = self.filtered[4:].copy() if transition else self.target_att
            if transition and stage == 'RR' and phase in ('RR_LIFTING', 'RR_POSTURE'):
                # RR intentionally rolls to create clearance. Neutral damping
                # alone would oppose the established posture maneuver.
                att_target[0] = np.deg2rad(self.rear_limits.posture_target_roll_deg)
            gains = self.models[stage]['gains']
            swing_weight = 1.
            if transition and loads[ORDER.index(stage)] > cfg.transition_contact_blend_low_n:
                weight = float(np.clip((cfg.transition_contact_blend_high_n - loads[ORDER.index(stage)]) /
                    (cfg.transition_contact_blend_high_n - cfg.transition_contact_blend_low_n), 0., 1.))
                gains = {k: weight * np.asarray(v) + (1. - weight) * np.asarray(self.models['FOUR_CONTACT']['gains'][k])
                         for k, v in gains.items()}
                swing_weight = weight
            att_goal = att_target - cfg.attitude_damping_s * np.deg2rad([x['AVx'], x['AVy']])
            result = None
            if allocation.status == 'OPTIMAL':
                result = allocate_support_increment(
                    lifted_corner=stage, contacts_xy=list(contacts.values()), fz_n=loads,
                    fz_target_n=allocation.fz_ref_n, com_xy=com, zmp_xy=assessment.zmp_xy,
                    attitude_rad=np.deg2rad([x['Roll_E'], x['Pitch']]),
                    attitude_target_rad=att_goal,
                    travel_m=[x[f'Jnc_{w}'] / 1000. for w in WHEELS],
                    wheel_height_m=[x[f'Z_{w}'] - self.config.tyre_radius_m for w in WHEELS],
                    gains=gains, base_force_n=base[4:8],
                    previous_applied_n=self.last_applied, correction_n=self.correction,
                    dt_s=min(dt, self.period_s), config=cfg)
            elapsed_ms = (time.perf_counter() - begin) * 1000.
            status = result.status if result is not None else 'LOAD_INFEASIBLE'
            self.status_counts[status] = self.status_counts.get(status, 0) + 1
            self.phase_counts[phase] = self.phase_counts.get(phase, 0) + 1
            self.solve_times_ms.append(elapsed_ms)
            if status == 'OPTIMAL':
                # Monitor mode solves from the real base without accumulating unapplied forces.
                if self.active:
                    self.correction = result.force_n[support] - base[4:8][support]
                self.failure_start_s = None
            elif self.failure_start_s is None:
                self.failure_start_s = now_s
            failure_s = 0. if self.failure_start_s is None else now_s - self.failure_start_s
            self.longest_failure_s = max(self.longest_failure_s, failure_s)
            self.telemetry = {**self._blank(), 'load_qp_status': allocation.status,
                              'support_qp_status': status,
                              'support_qp_cost': result.cost if result and result.cost is not None else 0.,
                              'support_qp_predicted_margin': result.predicted_margin if result and result.predicted_margin is not None else 0.,
                              'support_qp_solve_ms': elapsed_ms, 'support_qp_failure_s': failure_s}
            self.telemetry.update(support_qp_clearance_floor_m=cfg.clearance_floor_m,
                                  support_qp_roll_target_deg=float(np.rad2deg(att_goal[0])),
                                  support_qp_pitch_target_deg=float(np.rad2deg(att_goal[1])),
                                  support_qp_swing_gain_weight=swing_weight)
            if allocation.fz_ref_n is not None:
                for i, c in enumerate(ORDER):
                    self.telemetry[f'allocation_ref_{c.lower()}_n'] = float(allocation.fz_ref_n[i])
        elif not eligible:
            self.telemetry = self._blank()
            if transition and preload:
                self.telemetry['support_qp_status'] = waiting_status
            self.failure_start_s = None
        if self.active:
            base[4 + support] += self.correction
            if (self.abort_active or phase in ('RR_ABORT_STOP','RR_ABORT_EXIT')) and self.last_application_s is not None:
                application_dt = max(0., now_s - self.last_application_s)
                maximum_step = self.rear_limits.sim_force_slew_n_s * application_dt
                step = np.clip(base[4:8] - self.last_applied, -maximum_step, maximum_step)
                base[4:8] = self.last_applied + step
                if application_dt > 0.:
                    self.abort_max_output_slew_n_s = max(self.abort_max_output_slew_n_s,
                                                        float(np.max(np.abs(step))) / application_dt)
        self.last_applied = base[4:8].copy()
        self.last_application_s = now_s
        for i, c in enumerate(ORDER):
            self.telemetry[f'allocation_corr_{c.lower()}_n'] = float(
                base[4 + i] - command[4 + i])
        self.phase = phase
        return tuple(base)

    def summary(self):
        samples = np.asarray(self.solve_times_ms)
        return {'active': self.active, 'status_counts': self.status_counts,
                'transitions': self.transitions, 'phase_counts': self.phase_counts,
                'longest_failure_s': self.longest_failure_s,
                'solve_mean_ms': float(samples.mean()) if samples.size else None,
                'solve_p95_ms': float(np.percentile(samples, 95)) if samples.size else None,
                'solve_max_ms': float(samples.max()) if samples.size else None,
                'period_ms': self.period_s * 1000.,
                'abort_max_native_output_slew_n_s': self.abort_max_output_slew_n_s,
                'scope': 'local quasi-static support QP; existing feedforward retained'}
