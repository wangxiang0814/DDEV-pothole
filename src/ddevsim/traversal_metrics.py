"""Measured-dynamics acceptance, independent of the expert's phase labels."""

from __future__ import annotations

from typing import Mapping, Sequence

CORNERS = {"FL": "L1", "FR": "R1", "RL": "L2", "RR": "R2"}


def run_exit_code(solver_status: str, verification: Mapping[str, object]) -> int:
    if solver_status != "COMPLETED":
        return 2
    return 0 if verification.get("passed") is True else 3


def evaluate_traversal(
    rows: Sequence[Mapping[str, str]], scenario, *, safe_stop: bool,
    lifted_load_max_n: float = 150.0, support_floor_n: float = 300.0,
    roll_limit_deg: float = 7.0, yaw_limit_deg: float = 2.0,
    lateral_limit_m: float = 0.15, jounce_stop_m: float | None = None,
    landing_load_limit_n: float | None = None,
) -> dict:
    """Reject false DONE states if contact, support or attitude data disagree."""
    if not rows:
        raise ValueError("dynamics trace is empty")
    report = {}
    for corner in ("FR", "RR"):
        suffix = CORNERS[corner]
        in_pit = [row for row in rows if (
            scenario.leading_edge_m <= float(row["exp_X_" + suffix]) <= scenario.trailing_edge_m
            and scenario.center_y_m - scenario.width_m / 2 <= float(row["exp_Y_" + suffix])
            <= scenario.center_y_m + scenario.width_m / 2
        )]
        lifted = [row for row in in_pit if float(row["exp_Fz_" + suffix])
                  < lifted_load_max_n]
        support_loads = [float(row["exp_Fz_" + other_suffix])
                         for row in lifted for other, other_suffix in CORNERS.items()
                         if other != corner]
        report[corner] = {
            "seen": bool(in_pit),
            "in_pit_samples": len(in_pit),
            "lifted_samples": len(lifted),
            "lifted_fraction": len(lifted) / len(in_pit) if in_pit else 0.0,
            "min_support_load_n": min(support_loads) if support_loads else 0.0,
            "min_target_load_n": min(
                (float(row["exp_Fz_" + suffix]) for row in in_pit), default=0.0
            ),
        }
    max_roll = max(abs(float(row.get("exp_Roll_E", 0))) for row in rows)
    max_yaw = max(abs(float(row.get("exp_Yaw", 0))) for row in rows)
    max_lateral = max(abs(float(row.get("exp_Yo", 0))) for row in rows)
    # TruckSim exports Jnc_* in millimetres; the vehicle stop limit is metres.
    jounce_values = [abs(float(row["exp_Jnc_" + suffix])) / 1000.0
                     for row in rows for suffix in CORNERS.values()
                     if "exp_Jnc_" + suffix in row]
    max_jounce = max(jounce_values, default=None)
    landing_loads = []
    for corner in ("FR", "RR"):
        suffix = CORNERS[corner]
        has_entered = False
        for row in rows:
            x = float(row["exp_X_" + suffix])
            has_entered = has_entered or x >= scenario.leading_edge_m
            if has_entered and x > scenario.trailing_edge_m:
                landing_loads.append(float(row["exp_Fz_" + suffix]))
    max_landing_load = max(landing_loads, default=None)
    reasons = []
    if safe_stop:
        reasons.append("safe_stop")
    for corner in ("FR", "RR"):
        item = report[corner]
        if not item["seen"]:
            reasons.append("%s_not_in_pit" % corner)
        if item["lifted_fraction"] < 0.9:
            reasons.append("%s_not_lifted_through_pit" % corner)
        if item["min_support_load_n"] < support_floor_n:
            reasons.append("%s_support_load_below_floor" % corner)
    if max_roll > roll_limit_deg:
        reasons.append("roll_limit")
    if max_yaw > yaw_limit_deg:
        reasons.append("yaw_limit")
    if max_lateral > lateral_limit_m:
        reasons.append("lateral_limit")
    if jounce_stop_m is not None and (
        max_jounce is None or max_jounce > jounce_stop_m
    ):
        reasons.append("jounce_stop_exceeded" if max_jounce is not None
                       else "jounce_data_missing")
    if landing_load_limit_n is not None and (
        max_landing_load is None or max_landing_load > landing_load_limit_n
    ):
        reasons.append("landing_load_limit" if max_landing_load is not None
                       else "landing_load_data_missing")
    report.update({
        "passed": not reasons,
        "failure_reasons": reasons,
        "max_abs_roll_deg": max_roll,
        "max_abs_yaw_deg": max_yaw,
        "max_abs_lateral_m": max_lateral,
        "max_jounce_m": max_jounce,
        "max_landing_load_n": max_landing_load,
    })
    return report
