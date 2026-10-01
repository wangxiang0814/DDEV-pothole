"""Measured support-load correction and three-wheel crawl torque feedback.

The controllers operate on simulation timestamps.  They never advance plant time
or infer a contact from suspension travel alone.  Array order is FL/RL/RR.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def blend_contact_gain(static_gain, swing_gain, fr_load_n: float, *,
                       swing_n: float, stance_n: float) -> np.ndarray:
    """Interpolate measured local gains across a lightly loaded FR contact."""
    static = np.asarray(static_gain, dtype=float)
    swing = np.asarray(swing_gain, dtype=float)
    if (static.shape != (3, 3) or swing.shape != (3, 3) or
            not np.isfinite(static).all() or not np.isfinite(swing).all() or
            not all(math.isfinite(x) for x in (fr_load_n, swing_n, stance_n)) or
            not 0. <= swing_n < stance_n or fr_load_n < 0.):
        raise ValueError("invalid contact gain schedule")
    swing_weight = float(np.clip((stance_n - fr_load_n) /
                                 (stance_n - swing_n), 0., 1.))
    return (1. - swing_weight) * static + swing_weight * swing


class PreloadProgress:
    """Advance a validated force trace only as quickly as loads can follow."""

    def __init__(self, *, start_s: float, end_s: float, max_rate: float,
                 slow_error_n: float, pause_error_n: float):
        if (not 0. <= start_s < end_s or max_rate < 1. or
                not 0. <= slow_error_n < pause_error_n):
            raise ValueError("invalid preload progress limits")
        self.start_s = start_s
        self.end_s = end_s
        self.max_rate = max_rate
        self.slow_error_n = slow_error_n
        self.pause_error_n = pause_error_n
        self.progress_s = 0.
        self.last_time_s: float | None = None
        self.rate = 1.

    def update(self, now_s: float, *, load_error_n: float) -> float:
        if not math.isfinite(now_s) or not math.isfinite(load_error_n) or load_error_n < 0.:
            raise ValueError("invalid preload observation")
        if self.last_time_s is None:
            self.progress_s = min(now_s, self.start_s)
            self.last_time_s = now_s
            return self.progress_s
        if now_s <= self.last_time_s:
            raise ValueError("preload timestamps must increase")
        before_start = max(0., min(now_s, self.start_s) -
                           min(self.last_time_s, self.start_s))
        active_dt = max(0., now_s - max(self.last_time_s, self.start_s))
        self.rate = (0. if load_error_n >= self.pause_error_n else
                     1. if load_error_n >= self.slow_error_n else self.max_rate)
        self.progress_s = min(self.end_s, self.progress_s + before_start +
                              self.rate * active_dt)
        self.last_time_s = now_s
        return self.progress_s

    def align_reference(self, now_s: float, reference_s: float) -> None:
        """Start the active trace after actual settle, skipping its idle prefix."""
        if (not math.isfinite(now_s) or not math.isfinite(reference_s) or
                self.last_time_s is None or now_s < self.last_time_s or
                not self.progress_s <= reference_s <= self.end_s):
            raise ValueError("invalid preload reference alignment")
        self.progress_s = reference_s
        self.start_s = now_s
        self.last_time_s = now_s


class WheelLoadFilter:
    """First-order filter for FSM contact gates; raw loads remain available."""

    def __init__(self, *, cutoff_hz: float, wheel_count: int):
        if cutoff_hz <= 0. or wheel_count <= 0:
            raise ValueError("invalid wheel-load filter settings")
        self.cutoff_hz = cutoff_hz
        self.wheel_count = wheel_count
        self.value: np.ndarray | None = None
        self.last_time_s: float | None = None

    def update(self, now_s: float, raw_n) -> np.ndarray:
        raw = np.asarray(raw_n, dtype=float)
        if (raw.shape != (self.wheel_count,) or not np.isfinite(raw).all()
                or np.any(raw < 0.) or not math.isfinite(now_s)):
            raise ValueError("invalid wheel-load observation")
        if self.last_time_s is None:
            self.value = raw.copy()
        else:
            dt = now_s - self.last_time_s
            if dt <= 0.:
                raise ValueError("wheel-load timestamps must increase")
            tau = 1. / (2. * math.pi * self.cutoff_hz)
            self.value += dt / (tau + dt) * (raw - self.value)
        self.last_time_s = now_s
        return self.value.copy()


class ScalarLoadFeedback:
    """Rate-limited FR command correction using a measured local gain."""

    def __init__(self, *, gain_load_per_force: float,
                 control_period_s: float,
                 correction_limit_n: float, slew_n_s: float,
                 tracking_gain: float, deadband_n: float):
        if (not all(math.isfinite(x) for x in (
                gain_load_per_force, control_period_s, correction_limit_n, slew_n_s,
                tracking_gain, deadband_n)) or
                abs(gain_load_per_force) < 1e-6 or
                min(control_period_s, correction_limit_n,
                    slew_n_s, tracking_gain) <= 0. or
                deadband_n < 0.):
            raise ValueError("invalid scalar load-feedback settings")
        self.gain = gain_load_per_force
        self.period_s = control_period_s
        self.limit_n = correction_limit_n
        self.slew_n_s = slew_n_s
        self.tracking_gain = tracking_gain
        self.deadband_n = deadband_n
        self.correction_n = 0.
        self.last_time_s: float | None = None

    def update(self, now_s: float, *, measured_n: float,
               reference_n: float) -> float:
        if not all(math.isfinite(v) for v in (now_s, measured_n, reference_n)):
            raise ValueError("invalid scalar load observation")
        if self.last_time_s is not None and now_s <= self.last_time_s:
            raise ValueError("scalar load timestamps must increase")
        dt = self.period_s if self.last_time_s is None else now_s - self.last_time_s
        self.last_time_s = now_s
        error = reference_n - measured_n
        error = math.copysign(max(abs(error) - self.deadband_n, 0.), error)
        requested = self.tracking_gain * error / self.gain
        self.correction_n = float(np.clip(
            self.correction_n + np.clip(requested, -self.slew_n_s * dt,
                                        self.slew_n_s * dt),
            -self.limit_n, self.limit_n))
        return self.correction_n


@dataclass(frozen=True)
class SupportFeedbackConfig:
    filter_cutoff_hz: float = 5.0
    correction_gain: float = 0.1
    ridge_gain: float = 0.05
    force_slew_n_s: float = 400.0
    load_deadband_n: float = 150.0
    # The identified swing gain shows FL force is the useful RL load actuator.
    load_weights: tuple[float, float, float] = (0.25, 1.0, 0.25)


class SupportForceFeedback:
    """Bounded integral correction around a validated suspension feedforward."""

    def __init__(self, gain_fz_per_force, *, control_period_s: float,
                 correction_limit_n: float, rebound_guard_mm: float,
                 config: SupportFeedbackConfig = SupportFeedbackConfig()):
        gain = np.asarray(gain_fz_per_force, dtype=float)
        if gain.shape != (3, 3) or not np.isfinite(gain).all():
            raise ValueError("three-support identified gain required")
        if (control_period_s <= 0 or correction_limit_n <= 0 or
                not np.isfinite(rebound_guard_mm)):
            raise ValueError("invalid support feedback limits")
        self.gain = gain
        self.period_s = control_period_s
        self.limit_n = correction_limit_n
        self.rebound_guard_mm = rebound_guard_mm
        self.config = config
        self.filtered: np.ndarray | None = None
        self.correction = np.zeros(3)
        self.last_update_s: float | None = None

    def update(self, now_s: float, measured_n, reference_n, *, travel_mm) -> np.ndarray:
        measured = np.asarray(measured_n, dtype=float)
        reference = np.asarray(reference_n, dtype=float)
        travel = np.asarray(travel_mm, dtype=float)
        if any(v.shape != (3,) or not np.isfinite(v).all()
               for v in (measured, reference, travel)) or not math.isfinite(now_s):
            raise ValueError("invalid support feedback observation")
        if self.last_update_s is not None:
            if now_s <= self.last_update_s:
                raise ValueError("simulation timestamps must increase")
            if now_s - self.last_update_s < self.period_s - 1e-8:
                return self.correction.copy()
        dt = self.period_s if self.last_update_s is None else now_s - self.last_update_s
        self.last_update_s = now_s
        tau = 1.0 / (2.0 * math.pi * self.config.filter_cutoff_hz)
        alpha = dt / (tau + dt)
        self.filtered = (measured.copy() if self.filtered is None else
                         self.filtered + alpha * (measured - self.filtered))
        weights = np.diag(self.config.load_weights)
        matrix = np.vstack((weights @ self.gain,
                            self.config.ridge_gain * np.eye(3)))
        error = reference - self.filtered
        error = np.sign(error) * np.maximum(np.abs(error) -
                                             self.config.load_deadband_n, 0.)
        rhs = np.r_[weights @ error, np.zeros(3)]
        desired_step = self.config.correction_gain * np.linalg.lstsq(
            matrix, rhs, rcond=1e-5)[0]
        slew = self.config.force_slew_n_s * dt
        step = np.clip(desired_step, -slew, slew)
        self.correction = np.clip(self.correction + step,
                                  -self.limit_n, self.limit_n)
        # Positive FL correction uses up rebound reserve in this measured plant.
        if travel[0] <= self.rebound_guard_mm:
            self.correction[0] = min(0., self.correction[0])
        return self.correction.copy()


@dataclass(frozen=True)
class CrawlFeedbackConfig:
    speed_kp_n_per_m_s: float = 6000.0
    speed_ki_n_per_m: float = 400.0
    rolling_force_n: float = 100.0
    yaw_kp_nm_per_deg: float = 80.0
    yaw_kd_nm_per_deg_s: float = 25.0
    lateral_kp_n_per_m: float = 800.0
    rl_force_fraction: float = 0.08
    max_torque_nm: float = 500.0
    max_torque_slew_nm_s: float = 1000.0
    integral_limit_n: float = 400.0
    traction_utilization: float = 0.8


def front_clearance_guard_m(x_fr_m: float, far_edge_m: float,
                            crossing_clearance_m: float, pit_guard_m: float,
                            post_pit_guard_m: float) -> float:
    """Keep the full lip guard over the pit, then require positive clearance."""
    if (crossing_clearance_m < 0. or post_pit_guard_m <= 0. or
            pit_guard_m < post_pit_guard_m):
        raise ValueError("invalid phase-specific clearance guards")
    if x_fr_m < far_edge_m + crossing_clearance_m:
        return pit_guard_m
    return post_pit_guard_m


class CrawlTorqueFeedback:
    """Speed PI and yaw/lateral PD with friction-limited three-wheel torque."""

    def __init__(self, *, control_period_s: float, tyre_radius_m: float,
                 half_track_m: float, friction: float,
                 config: CrawlFeedbackConfig = CrawlFeedbackConfig()):
        if min(control_period_s, tyre_radius_m, half_track_m, friction) <= 0:
            raise ValueError("invalid crawl geometry or friction")
        self.period_s = control_period_s
        self.radius_m = tyre_radius_m
        self.half_track_m = half_track_m
        self.friction = friction
        self.config = config
        self.integral_n = 0.
        self.torque_nm = np.zeros(3)
        self.last_update_s: float | None = None

    def update(self, now_s: float, *, vx_kph: float, yaw_deg: float,
               yaw_rate_deg_s: float, lateral_m: float, loads_n,
               target_kph: float) -> np.ndarray:
        loads = np.asarray(loads_n, dtype=float)
        values = (now_s, vx_kph, yaw_deg, yaw_rate_deg_s, lateral_m, target_kph)
        if (loads.shape != (3,) or not np.isfinite(loads).all() or
                np.any(loads < 0.) or not all(math.isfinite(v) for v in values)):
            raise ValueError("invalid crawl observation")
        if self.last_update_s is not None:
            if now_s <= self.last_update_s:
                raise ValueError("simulation timestamps must increase")
            if now_s - self.last_update_s < self.period_s - 1e-8:
                return self.torque_nm.copy()
        dt = self.period_s if self.last_update_s is None else now_s - self.last_update_s
        self.last_update_s = now_s
        speed_error = (target_kph - vx_kph) / 3.6
        candidate_integral = float(np.clip(
            self.integral_n + self.config.speed_ki_n_per_m * speed_error * dt,
            -self.config.integral_limit_n, self.config.integral_limit_n))
        yaw_moment_nm = (-self.config.yaw_kp_nm_per_deg * yaw_deg -
                         self.config.yaw_kd_nm_per_deg_s * yaw_rate_deg_s -
                         self.config.lateral_kp_n_per_m * lateral_m)
        def requested_torque(integral_n: float) -> np.ndarray:
            forward_n = (self.config.speed_kp_n_per_m_s * speed_error +
                         integral_n +
                         (self.config.rolling_force_n if target_kph > 0 else 0.))
            rl_n = self.config.rl_force_fraction * forward_n
            remainder = forward_n - rl_n
            difference = yaw_moment_nm / self.half_track_m + rl_n
            forces = np.array([(remainder - difference) / 2., rl_n,
                               (remainder + difference) / 2.])
            return forces * self.radius_m

        cap = np.minimum(self.config.traction_utilization * self.friction *
                         loads * self.radius_m,
                         self.config.max_torque_nm)
        requested = requested_torque(candidate_integral)
        if np.any(np.abs(requested) > cap + 1e-8) and speed_error != 0.:
            requested = requested_torque(self.integral_n)
        else:
            self.integral_n = candidate_integral
        desired = np.clip(requested, -cap, cap)
        slew = self.config.max_torque_slew_nm_s * dt
        self.torque_nm += np.clip(desired - self.torque_nm, -slew, slew)
        return self.torque_nm.copy()
