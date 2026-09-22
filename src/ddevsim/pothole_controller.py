from __future__ import annotations

import math
from typing import Sequence, Tuple


class WheelLiftController:
    """Deterministic sequential wheel-lift expert for the first pothole run.

    Position gates reproduce the published front-right/rear-right sequential
    lift concept. Force magnitudes are re-derived from the HD Utility vehicle's
    measured wheel loads rather than copied from the paper vehicle. The three
    non-target corners share the equal-and-opposite support force, preserving a
    three-point support base without adding net vertical actuator force.
    """

    front_right_x_window_m = (100.9, 102.5)
    rear_right_x_window_m = (104.8, 106.4)
    target_speed_kph = 2.8
    wheel_load_target_n = 1500.0
    unload_gain = 0.85
    force_min_n = -20000.0
    force_max_n = 20000.0
    torque_min_nm = -400.0
    torque_max_nm = 700.0

    def _target_index(self, time_s: float, exports: Sequence[float]) -> int | None:
        if len(exports) >= 18 and math.isfinite(float(exports[16])):
            xo = float(exports[16])
            if self.front_right_x_window_m[0] <= xo < self.front_right_x_window_m[1]:
                return 1
            if self.rear_right_x_window_m[0] <= xo < self.rear_right_x_window_m[1]:
                return 3
            return None
        if 1.4 <= time_s < 3.1:
            return 1
        if 5.0 <= time_s < 6.9:
            return 3
        return None

    def __call__(self, time_s: float, exports: Sequence[float]) -> Tuple[float, ...]:
        target = self._target_index(float(time_s), exports)
        vx = float(exports[17]) if len(exports) >= 18 else 0.0
        if not math.isfinite(vx):
            vx = 0.0
        base_torque = 80.0 + 450.0 * (self.target_speed_kph - vx)
        base_torque = max(self.torque_min_nm, min(self.torque_max_nm, base_torque))
        if target is None:
            return (base_torque, base_torque, base_torque, base_torque, 0.0, 0.0, 0.0, 0.0)

        support_torque = max(
            self.torque_min_nm,
            min(self.torque_max_nm, base_torque * 4.0 / 3.0),
        )
        torques = [support_torque] * 4
        torques[target] = 0.0
        active = [0.0] * 4
        fz_index = 4 + target
        measured_fz = float(exports[fz_index]) if len(exports) > fz_index else self.wheel_load_target_n
        if not math.isfinite(measured_fz):
            measured_fz = self.wheel_load_target_n
        unload = self.unload_gain * (self.wheel_load_target_n - measured_fz)
        unload = max(self.force_min_n, min(0.0, unload))
        active[target] = unload
        support_each = min(self.force_max_n, max(0.0, -unload / 3.0))
        for index in range(4):
            if index != target:
                active[index] = support_each
        return tuple(torques + active)
