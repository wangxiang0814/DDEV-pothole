"""Sensor-gated FR lift state machine, independent of the TruckSim execution loop."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class LiftState(str, Enum):
    INIT_SETTLE = "INIT_SETTLE"
    PRELOAD_SHIFT = "PRELOAD_SHIFT"
    LIFT_READY_HOLD = "LIFT_READY_HOLD"
    LIFTING = "LIFTING"
    THREE_WHEEL_HOLD = "THREE_WHEEL_HOLD"
    LOWERING = "LOWERING"
    RETURN_TO_FOUR_WHEEL = "RETURN_TO_FOUR_WHEEL"
    ABORT_RECOVERY = "ABORT_RECOVERY"
    COMPLETE = "COMPLETE"


@dataclass(frozen=True)
class LiftSignals:
    stationary: bool
    four_wheel_loaded: bool
    feasible: bool
    safe_triangle: bool
    low_fr_load: bool
    support_loaded: bool
    attitude_safe: bool
    actuator_safe: bool
    lift_height_reached: bool
    lift_clearance_confirmed: bool
    four_wheel_recovered: bool
    lowering_complete: bool = False


class LiftFSM:
    def __init__(self, *, settle_s: float, ready_s: float, hold_s: float):
        if settle_s <= 0 or ready_s <= 0 or hold_s < 5.0:
            raise ValueError("invalid dwell or hold duration")
        self.settle_s = settle_s
        self.ready_s = ready_s
        self.hold_s = hold_s
        self.state = LiftState.INIT_SETTLE
        self.entered_s: float | None = None
        self.good_since_s: float | None = None
        self.last_time_s: float | None = None
        self.recovery_required = False

    def _transition(self, state: LiftState, now_s: float):
        self.state = state
        self.entered_s = now_s
        self.good_since_s = None

    def _dwelled(self, condition: bool, now_s: float, required_s: float) -> bool:
        if not condition:
            self.good_since_s = None
            return False
        if self.good_since_s is None:
            self.good_since_s = now_s
        return now_s - self.good_since_s >= required_s

    def update(self, now_s: float, s: LiftSignals) -> LiftState:
        if self.last_time_s is not None and now_s <= self.last_time_s:
            raise ValueError("simulation timestamps must increase")
        self.last_time_s = now_s
        if self.entered_s is None:
            self.entered_s = now_s
        if self.state in (LiftState.COMPLETE, LiftState.ABORT_RECOVERY):
            if (self.state == LiftState.ABORT_RECOVERY and s.four_wheel_recovered
                    and s.four_wheel_loaded and s.stationary and s.support_loaded
                    and s.attitude_safe and s.actuator_safe):
                self._transition(LiftState.COMPLETE, now_s)
            return self.state
        stable = s.stationary and s.support_loaded and s.attitude_safe and s.actuator_safe
        if self.state != LiftState.INIT_SETTLE and (not stable or not s.feasible):
            self.recovery_required = True
            self._transition(LiftState.ABORT_RECOVERY, now_s)
        elif self.state == LiftState.INIT_SETTLE:
            if self._dwelled(stable and s.four_wheel_loaded and s.feasible,
                             now_s, self.settle_s):
                self._transition(LiftState.PRELOAD_SHIFT, now_s)
        elif self.state == LiftState.PRELOAD_SHIFT:
            if not s.feasible:
                self.recovery_required = True
                self._transition(LiftState.ABORT_RECOVERY, now_s)
            elif s.safe_triangle and s.low_fr_load:
                self._transition(LiftState.LIFT_READY_HOLD, now_s)
        elif self.state == LiftState.LIFT_READY_HOLD:
            ready = s.feasible and s.safe_triangle and s.low_fr_load
            if not ready:
                self._transition(LiftState.PRELOAD_SHIFT, now_s)
            elif self._dwelled(ready and stable, now_s, self.ready_s):
                self._transition(LiftState.LIFTING, now_s)
        elif self.state == LiftState.LIFTING:
            if not (s.safe_triangle and s.low_fr_load):
                self.recovery_required = True
                self._transition(LiftState.ABORT_RECOVERY, now_s)
            elif s.lift_height_reached and s.lift_clearance_confirmed:
                self._transition(LiftState.THREE_WHEEL_HOLD, now_s)
        elif self.state == LiftState.THREE_WHEEL_HOLD:
            if not (s.safe_triangle and s.low_fr_load and s.lift_clearance_confirmed):
                self.recovery_required = True
                self._transition(LiftState.ABORT_RECOVERY, now_s)
            elif now_s - self.entered_s >= self.hold_s:
                self._transition(LiftState.LOWERING, now_s)
        elif self.state == LiftState.LOWERING:
            if s.lowering_complete and s.four_wheel_loaded:
                self._transition(LiftState.RETURN_TO_FOUR_WHEEL, now_s)
        elif self.state == LiftState.RETURN_TO_FOUR_WHEEL:
            if s.four_wheel_recovered:
                self._transition(LiftState.COMPLETE, now_s)
        return self.state
