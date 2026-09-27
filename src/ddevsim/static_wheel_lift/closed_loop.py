"""Measured support-load correction and three-wheel crawl torque feedback.

The controllers operate on simulation timestamps.  They never advance plant time
or infer a contact from suspension travel alone.  Array order is FL/RL/RR.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


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
        self.integral_n = float(np.clip(
            self.integral_n + self.config.speed_ki_n_per_m * speed_error * dt,
            -self.config.integral_limit_n, self.config.integral_limit_n))
        forward_n = (self.config.speed_kp_n_per_m_s * speed_error +
                     self.integral_n +
                     (self.config.rolling_force_n if target_kph > 0 else 0.))
        yaw_moment_nm = (-self.config.yaw_kp_nm_per_deg * yaw_deg -
                         self.config.yaw_kd_nm_per_deg_s * yaw_rate_deg_s -
                         self.config.lateral_kp_n_per_m * lateral_m)
        rl_n = self.config.rl_force_fraction * forward_n
        remainder = forward_n - rl_n
        difference = yaw_moment_nm / self.half_track_m + rl_n
        forces = np.array([(remainder - difference) / 2., rl_n,
                           (remainder + difference) / 2.])
        cap = np.minimum(self.friction * loads * self.radius_m,
                         self.config.max_torque_nm)
        desired = np.clip(forces * self.radius_m, -cap, cap)
        slew = self.config.max_torque_slew_nm_s * dt
        self.torque_nm += np.clip(desired - self.torque_nm, -slew, slew)
        return self.torque_nm.copy()
