"""Check whether oscillatory native data still has a usable local mean."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


WHEELS = ("L1", "R1", "L2", "R2")


@dataclass(frozen=True)
class IdentificationSettleLimits:
    window_s: float = 1.0
    min_samples_per_window: int = 50
    max_mean_fz_change_n: float = 20.0
    max_fz_std_n: float = 100.0
    max_mean_attitude_change_deg: float = 0.02
    max_attitude_std_deg: float = 0.05
    max_mean_travel_change_mm: float = 1.0
    max_mean_com_change_m: float = 0.002
    max_abs_vx_kph: float = 0.036
    max_abs_wheel_rpm: float = 1.0


@dataclass(frozen=True)
class IdentificationSettleResult:
    status: str
    metrics: dict[str, float]


def assess_identification_settle(rows, *, start_s: float,
                                  limits: IdentificationSettleLimits,
                                  ) -> IdentificationSettleResult:
    """Compare adjacent one-second means and scatter without a rate peak gate.

    Online support, wheel-load, travel and body-angle limits still apply.
    This only determines whether an average response is usable for M3 gains.
    """
    if not np.isfinite(start_s) or limits.window_s <= 0:
        raise ValueError("invalid identification window")
    intervals = [
        [r for r in rows if start_s + i * limits.window_s <= float(r["time_s"])
         < start_s + (i + 1) * limits.window_s]
        for i in (0, 1)
    ]
    if min(map(len, intervals)) < limits.min_samples_per_window:
        return IdentificationSettleResult("NOT_SETTLED", {"samples": float(min(map(len, intervals)))})

    def array(part, fields):
        result = np.asarray([[float(r[f"exp_{field}"]) for field in fields]
                             for r in part], dtype=float)
        if not np.isfinite(result).all():
            raise ValueError("nonfinite identification telemetry")
        return result

    def change_and_std(fields):
        first, second = (array(part, fields) for part in intervals)
        return (float(np.max(np.abs(first.mean(axis=0) - second.mean(axis=0)))),
                float(max(np.max(first.std(axis=0)), np.max(second.std(axis=0)))))

    fz_change, fz_std = change_and_std([f"Fz_{w}" for w in WHEELS])
    att_change, att_std = change_and_std(["Roll_E", "Pitch"])
    travel_change, _ = change_and_std([f"Jnc_{w}" for w in WHEELS])
    com_change, _ = change_and_std(["XCG_TM", "YCG_TM"])
    both = intervals[0] + intervals[1]
    max_vx = float(max(abs(float(r["exp_Vx"])) for r in both))
    max_wheel = float(max(abs(float(r[f"exp_AVy_{w}"]))
                          for r in both for w in WHEELS))
    metrics = {
        "max_mean_fz_change_n": fz_change,
        "max_fz_std_n": fz_std,
        "max_mean_attitude_change_deg": att_change,
        "max_attitude_std_deg": att_std,
        "max_mean_travel_change_mm": travel_change,
        "max_mean_com_change_m": com_change,
        "max_abs_vx_kph": max_vx,
        "max_abs_wheel_rpm": max_wheel,
    }
    permitted = (
        fz_change <= limits.max_mean_fz_change_n and
        fz_std <= limits.max_fz_std_n and
        att_change <= limits.max_mean_attitude_change_deg and
        att_std <= limits.max_attitude_std_deg and
        travel_change <= limits.max_mean_travel_change_mm and
        com_change <= limits.max_mean_com_change_m and
        max_vx <= limits.max_abs_vx_kph and
        max_wheel <= limits.max_abs_wheel_rpm
    )
    return IdentificationSettleResult("SETTLED" if permitted else "NOT_SETTLED", metrics)
