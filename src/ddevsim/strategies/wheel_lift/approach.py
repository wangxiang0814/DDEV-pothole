"""Four-contact position and velocity feedback before wheel lift (native units)."""
from dataclasses import dataclass
import math

import numpy as np


WHEELS = ('L1', 'R1', 'L2', 'R2')


@dataclass(frozen=True)
class ApproachConfig:
    period_s: float = .02
    settle_s: float = .8
    speed_mps: float = 4. / 3.6
    acceleration_mps2: float = .6
    deceleration_mps2: float = .5
    position_gain_s: float = 1.4
    speed_kp_nm_per_mps: float = 165.
    speed_ki_nm_per_m: float = 65.
    rolling_torque_nm: float = 28.
    integral_limit_nm: float = 125.
    torque_limit_nm: float = 500.
    torque_slew_nm_s: float = 1000.
    steer_lateral_deg_per_m: float = 350.
    steer_yaw_deg_per_deg: float = 12.
    steer_limit_deg: float = 80.
    steer_slew_deg_s: float = 400.
    capture_m: float = .06
    capture_speed_mps: float = .16
    stop_tolerance_m: float = .15
    min_pit_gap_m: float = .5
    stop_speed_kph: float = .1
    stop_wheel_rpm: float = 2.
    stop_dwell_s: float = .6
    min_load_n: float = 850.
    max_lateral_error_m: float = .05
    max_yaw_error_deg: float = 2.
    timeout_s: float = 25.
    abort_coast_s: float = 2.


class PitApproach:
    def __init__(self, edge_m, target_m, config=ApproachConfig()):
        if not all(map(math.isfinite, (edge_m, target_m))) or (
                edge_m - target_m < config.min_pit_gap_m + config.stop_tolerance_m):
            raise ValueError('wheel lift stop target must be safely before the pit')
        self.edge_m, self.target_m, self.config = edge_m, target_m, config
        self.phase = 'APPROACH_DRIVE'
        self.start_s = self.last_tick_s = self.stable_since_s = self.abort_s = None
        self.reference = None
        self.abort_reason = None
        self.ready = self.failed = self.terminal_brake = False
        self.integral_nm = self.speed_reference_mps = 0.
        self.command = (0.,) * 9
        self.rows = []

    def abort(self, now_s, reason):
        if self.abort_reason is None:
            self.abort_reason, self.abort_s = reason, now_s
            self.phase = 'APPROACH_ABORT'
            self.stable_since_s = None

    def _measurement(self, x):
        return dict(x_fr_m=x['X_R1'], vx_kph=x['Vx'],
                    wheel_rpm_max=max(abs(x[f'AVy_{w}']) for w in WHEELS),
                    min_load_n=min(x[f'Fz_{w}'] for w in WHEELS),
                    yo_m=x['Yo'], yaw_deg=x['Yaw'])

    def record_truth(self, now_s, x):
        if self.rows and self.rows[-1]['time_s'] == now_s:
            self.rows[-1].update(self._measurement(x))

    def _stationary(self, sample):
        c = self.config
        return (abs(sample['x_fr_m'] - self.target_m) <= c.stop_tolerance_m
                and abs(sample['vx_kph']) <= c.stop_speed_kph
                and sample['wheel_rpm_max'] <= c.stop_wheel_rpm
                and sample['min_load_n'] >= c.min_load_n)

    def step(self, now_s, x=None):
        c = self.config
        if self.last_tick_s is not None and now_s - self.last_tick_s < c.period_s - 1e-8:
            return self.command
        dt = c.period_s if self.last_tick_s is None else now_s - self.last_tick_s
        self.last_tick_s = now_s
        torque_target = steering_target = 0.
        sample = self._measurement(x) if x is not None else None
        if self.abort_reason is None:
            if sample is None or not all(map(math.isfinite, sample.values())):
                self.abort(now_s, 'invalid approach observation')
            else:
                if self.start_s is None:
                    self.start_s = now_s
                    self.reference = sample['yo_m'], sample['yaw_deg']
                elapsed = now_s - self.start_s
                distance = self.target_m - sample['x_fr_m']
                speed = sample['vx_kph'] / 3.6
                lateral = sample['yo_m'] - self.reference[0]
                yaw = sample['yaw_deg'] - self.reference[1]
                if distance < -c.stop_tolerance_m or self.edge_m - sample['x_fr_m'] < c.min_pit_gap_m:
                    self.abort(now_s, 'approach overshot the stop zone')
                elif abs(lateral) > c.max_lateral_error_m or abs(yaw) > c.max_yaw_error_deg:
                    self.abort(now_s, 'approach left the straight path')
                elif elapsed > c.timeout_s:
                    self.abort(now_s, 'approach stop deadline exceeded')
                elif elapsed >= c.settle_s:
                    if distance <= c.speed_mps ** 2 / (2. * c.deceleration_mps2):
                        self.phase = 'APPROACH_BRAKE'
                    if distance <= 0. or (distance <= c.capture_m and abs(speed) <= c.capture_speed_mps):
                        self.terminal_brake = True
                    if self.terminal_brake:
                        self.speed_reference_mps = 0.
                        if speed > c.stop_speed_kph / 3.6:
                            torque_target = -c.speed_kp_nm_per_mps * speed
                    else:
                        remaining = max(0., distance)
                        self.speed_reference_mps = min(c.speed_mps, c.acceleration_mps2 * (elapsed - c.settle_s),
                                                       math.sqrt(2. * c.deceleration_mps2 * remaining),
                                                       c.position_gain_s * remaining)
                        error = self.speed_reference_mps - speed
                        if self.phase == 'APPROACH_BRAKE':
                            self.integral_nm = 0.
                        else:
                            self.integral_nm = float(np.clip(self.integral_nm + c.speed_ki_nm_per_m * error * dt,
                                                             -c.integral_limit_nm, c.integral_limit_nm))
                        torque_target = c.rolling_torque_nm + c.speed_kp_nm_per_mps * error + self.integral_nm
                        steering_target = -c.steer_lateral_deg_per_m * lateral - c.steer_yaw_deg_per_deg * yaw
        torque_target = float(np.clip(torque_target, -c.torque_limit_nm, c.torque_limit_nm))
        steering_target = float(np.clip(steering_target, -c.steer_limit_deg, c.steer_limit_deg))
        torque = self.command[0] + float(np.clip(torque_target - self.command[0],
                                                -c.torque_slew_nm_s * dt, c.torque_slew_nm_s * dt))
        steering = self.command[8] + float(np.clip(steering_target - self.command[8],
                                                  -c.steer_slew_deg_s * dt, c.steer_slew_deg_s * dt))
        self.command = (torque,) * 4 + (0.,) * 4 + (steering,)
        if self.abort_reason is not None:
            self.failed = now_s - self.abort_s >= c.abort_coast_s and torque == 0.
        elif self.terminal_brake and torque == 0. and steering == 0. and self._stationary(sample):
            if self.stable_since_s is None:
                self.stable_since_s = now_s
            self.ready = now_s - self.stable_since_s >= c.stop_dwell_s - 1e-8
        else:
            self.stable_since_s = None
        self.rows.append(dict(time_s=now_s, mode=self.phase, **(sample or {}),
                              speed_ref_kph=3.6 * self.speed_reference_mps,
                              torque_each_nm=torque, steer_sw_deg=steering,
                              stop_ready=self.ready, abort_reason=self.abort_reason))
        return self.command

    def summary(self):
        c = self.config
        samples = [row for row in self.rows if 'x_fr_m' in row]
        last = samples[-1] if samples else None
        dwell = [r for r in samples if self.stable_since_s is not None and r['time_s'] >= self.stable_since_s]
        straight = bool(samples) and all(abs(r['yo_m'] - samples[0]['yo_m']) <= c.max_lateral_error_m and
                                         abs(r['yaw_deg'] - samples[0]['yaw_deg']) <= c.max_yaw_error_deg for r in samples)
        criteria = dict(approach_straight=straight,
                        approach_remained_before_pit=bool(samples) and all(
                            self.edge_m - r['x_fr_m'] >= c.min_pit_gap_m for r in samples),
                        approach_stopped_before_preload=self.ready and bool(dwell) and all(
                            self._stationary(r) for r in dwell))
        return dict(criteria=criteria, abort_reason=self.abort_reason, target_station_m=self.target_m,
                    stop_station_m=last['x_fr_m'] if self.ready and last else None,
                    stop_speed_kph=last['vx_kph'] if self.ready and last else None,
                    stop_gap_m=self.edge_m - last['x_fr_m'] if self.ready and last else None,
                    initial_gap_m=self.edge_m - samples[0]['x_fr_m'] if samples else None,
                    handoff_time_s=last['time_s'] if self.ready and last else None,
                    max_speed_kph=max((abs(r['vx_kph']) for r in samples), default=None),
                    max_lateral_error_m=max((abs(r['yo_m'] - samples[0]['yo_m']) for r in samples), default=None),
                    max_yaw_error_deg=max((abs(r['yaw_deg'] - samples[0]['yaw_deg']) for r in samples), default=None))
