"""Measured-dwell supervisor for one static FR unloading increment."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .quintic_trajectory import quintic_step


@dataclass(frozen=True)
class UnloadSample:
    time_s: float
    fz_filtered_n: tuple[float, float, float, float]
    lambda_min: float
    travel_mm: tuple[float, float, float, float]
    roll_deg: float
    pitch_deg: float
    roll_rate_deg_s: float
    pitch_rate_deg_s: float
    vx_kph: float
    wheel_speed_rpm: tuple[float, float, float, float]


@dataclass(frozen=True)
class UnloadOutput:
    status: str
    force_n: tuple[float, float, float, float]
    reason: str | None


class WheelLoadLowPass:
    """Causal first-order load filter driven by simulation timestamps."""

    def __init__(self, *, cutoff_hz: float):
        if not math.isfinite(cutoff_hz) or cutoff_hz <= 0:
            raise ValueError("positive finite cutoff required")
        self.cutoff_hz = cutoff_hz
        self.last_time_s: float | None = None
        self.value: tuple[float, float, float, float] | None = None

    def update(self, time_s: float, raw_n) -> tuple[float, float, float, float]:
        raw = tuple(float(v) for v in raw_n)
        if (len(raw) != 4 or not math.isfinite(time_s) or
                not all(math.isfinite(v) for v in raw) or
                self.last_time_s is not None and time_s <= self.last_time_s):
            raise ValueError("invalid load sample or timestamp")
        if self.value is None:
            self.value = raw
        else:
            alpha = 1. - math.exp(-2. * math.pi * self.cutoff_hz *
                                  (time_s - self.last_time_s))
            self.value = tuple(prev + alpha * (new - prev)
                               for prev, new in zip(self.value, raw))
        self.last_time_s = time_s
        return self.value


class IncrementalUnloadController:
    """Run one trusted force increment; request fresh identification afterward."""

    def __init__(self, *, target_force_n, baseline_fr_n, fr_drop_target_n,
                 earliest_start_s, settle_s, ramp_s, hold_s, recovery_s,
                 lambda_safe, support_floor_n, travel_lower_mm,
                 travel_upper_mm, max_attitude_deg, max_rate_deg_s,
                 max_vx_kph, max_wheel_rpm, fr_tolerance_n):
        target = tuple(float(v) for v in target_force_n)
        if len(target) != 4 or not all(math.isfinite(v) for v in target):
            raise ValueError("four finite suspension forces required")
        bounds = (baseline_fr_n, fr_drop_target_n, earliest_start_s,
                  settle_s, ramp_s, hold_s, recovery_s, lambda_safe,
                  support_floor_n, travel_lower_mm, travel_upper_mm,
                  max_attitude_deg, max_rate_deg_s, max_vx_kph,
                  max_wheel_rpm, fr_tolerance_n)
        if (not all(math.isfinite(v) for v in bounds) or
                min(settle_s, ramp_s, hold_s, recovery_s, fr_drop_target_n) <= 0 or
                not 0 <= lambda_safe < 1 / 3 or baseline_fr_n <= 0 or
                support_floor_n < 0 or travel_lower_mm >= travel_upper_mm or
                min(max_attitude_deg, max_rate_deg_s, max_vx_kph,
                    max_wheel_rpm) <= 0 or fr_tolerance_n < 0):
            raise ValueError("invalid staged unloading limits")
        self.target = target
        self.baseline_fr_n = baseline_fr_n
        self.fr_drop_target_n = fr_drop_target_n
        self.earliest_start_s = earliest_start_s
        self.settle_s = settle_s
        self.ramp_s = ramp_s
        self.hold_s = hold_s
        self.recovery_s = recovery_s
        self.lambda_safe = lambda_safe
        self.support_floor_n = support_floor_n
        self.travel_lower_mm = travel_lower_mm
        self.travel_upper_mm = travel_upper_mm
        self.max_attitude_deg = max_attitude_deg
        self.max_rate_deg_s = max_rate_deg_s
        self.max_vx_kph = max_vx_kph
        self.max_wheel_rpm = max_wheel_rpm
        self.fr_tolerance_n = fr_tolerance_n
        self.status = "WAIT_STABLE"
        self.good_since_s: float | None = None
        self.ramp_started_s: float | None = None
        self.hold_started_s: float | None = None
        self.abort_started_s: float | None = None
        self.abort_anchor = (0., 0., 0., 0.)
        self.last_force = (0., 0., 0., 0.)
        self.last_time_s: float | None = None
        self.reason: str | None = None

    def _safe(self, s: UnloadSample) -> bool:
        numbers = (*s.fz_filtered_n, s.lambda_min, *s.travel_mm,
                   s.roll_deg, s.pitch_deg, s.roll_rate_deg_s,
                   s.pitch_rate_deg_s, s.vx_kph, *s.wheel_speed_rpm)
        if (len(s.fz_filtered_n) != 4 or len(s.travel_mm) != 4 or
                len(s.wheel_speed_rpm) != 4 or
                not all(math.isfinite(float(v)) for v in numbers)):
            return False
        return (s.lambda_min >= self.lambda_safe and
                s.fz_filtered_n[1] >= 0 and
                min(s.fz_filtered_n[i] for i in (0, 2, 3)) >= self.support_floor_n and
                min(s.travel_mm) >= self.travel_lower_mm and
                max(s.travel_mm) <= self.travel_upper_mm and
                max(abs(s.roll_deg), abs(s.pitch_deg)) <= self.max_attitude_deg and
                max(abs(s.roll_rate_deg_s),
                    abs(s.pitch_rate_deg_s)) <= self.max_rate_deg_s and
                abs(s.vx_kph) <= self.max_vx_kph and
                max(abs(v) for v in s.wheel_speed_rpm) <= self.max_wheel_rpm)

    def _output(self, force) -> UnloadOutput:
        self.last_force = tuple(float(v) for v in force)
        return UnloadOutput(self.status, self.last_force, self.reason)

    def update(self, s: UnloadSample) -> UnloadOutput:
        t = float(s.time_s)
        if not math.isfinite(t) or self.last_time_s is not None and t <= self.last_time_s:
            raise ValueError("simulation time must increase")
        self.last_time_s = t
        safe = self._safe(s)
        if self.status == "WAIT_STABLE":
            if t < self.earliest_start_s or not safe:
                self.good_since_s = None
                return self._output((0.,) * 4)
            if self.good_since_s is None:
                self.good_since_s = t
            if t - self.good_since_s >= self.settle_s:
                self.status = "RAMPING"
                self.ramp_started_s = t
            return self._output((0.,) * 4)
        if self.status == "ABORTED":
            return self._output((0.,) * 4)
        if self.status != "ABORT_RECOVERY" and not safe:
            self.status = "ABORT_RECOVERY"
            self.reason = "safety_limit"
            self.abort_started_s = t
            self.abort_anchor = self.last_force
        if self.status == "ABORT_RECOVERY":
            fraction = quintic_step(t, self.abort_started_s, self.recovery_s)[0]
            if fraction >= 1.:
                self.status = "ABORTED"
            return self._output(tuple(v * (1. - fraction) for v in self.abort_anchor))
        if self.status == "RAMPING":
            fraction = quintic_step(t, self.ramp_started_s, self.ramp_s)[0]
            if fraction >= 1.:
                self.status = "HOLDING"
                self.hold_started_s = t
            return self._output(tuple(v * fraction for v in self.target))
        if self.status == "HOLDING":
            if t - self.hold_started_s >= self.hold_s:
                achieved = (s.fz_filtered_n[1] <= self.baseline_fr_n -
                            self.fr_drop_target_n + self.fr_tolerance_n)
                self.status = "READY_REIDENTIFY" if achieved else "STEP_UNMET"
                if not achieved:
                    self.reason = "fr_drop_unmet"
            return self._output(self.target)
        return self._output(self.target)
