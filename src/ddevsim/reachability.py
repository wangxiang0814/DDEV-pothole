"""Bounded, evidence-first native reachability checks for one lifted wheel.

An unsuccessful finite scan is `feasibility_unresolved`, not a proof that the
plant cannot traverse the pit. All geometry comes from native contact/CG
exports and the physical pit's piecewise-linear height map.
"""

from __future__ import annotations

import math
from typing import Callable, Mapping, Sequence

from .contact_geometry import (
    native_support_geometry,
    pothole_tyre_envelope_gap_m,
)
from .pothole_case import PotholeScenario
from .support_geometry import support_triangle_margin_m

_CORNERS = ("FL", "FR", "RL", "RR")
_TYRES = ("L1", "R1", "L2", "R2")


def isolated_rear_start_station(
    pit_exit_m: float, tyre_radius_m: float,
    front_center_offset_m: float, clearance_m: float,
) -> float:
    """Minimum body-origin station with the front tyre clear of the exit lip."""
    if tyre_radius_m <= 0.0 or clearance_m < 0.0:
        raise ValueError("radius must be positive and clearance nonnegative")
    return pit_exit_m + tyre_radius_m + clearance_m - front_center_offset_m


def build_profile(
    corner: str,
    force_kn: tuple[float, float, float, float],
    t_preload_s: float,
    t_lift_s: float,
    force_limit_n: float,
    slew_n_per_s: float,
) -> Callable[[float, Sequence[float]], tuple[float, ...]]:
    """Four-contact posture ramp, then a single target-wheel unloading ramp.

    The four torque channels are deliberately zero in this *suspension*
    reachability probe.  `force_kn` is ordered FL, FR, RL, RR. Values are
    clipped to the force authority and to a target attainable within each
    phase's registered slew/time budget; neither force nor slew is exceeded.
    """
    if corner not in _CORNERS or len(force_kn) != 4:
        raise ValueError("corner and four force targets required")
    if min(t_preload_s, t_lift_s, force_limit_n, slew_n_per_s) <= 0.0:
        raise ValueError("phase durations, force and slew must be positive")
    if not all(math.isfinite(float(f)) for f in force_kn):
        raise ValueError("force targets must be finite")
    target_index = _CORNERS.index(corner)
    applied = [
        math.copysign(
            min(abs(1000.0 * float(f)), force_limit_n,
                slew_n_per_s * (t_lift_s if index == target_index else t_preload_s)),
            float(f),
        )
        for index, f in enumerate(force_kn)
    ]

    def command(time_s: float, _exports: Sequence[float]) -> tuple[float, ...]:
        t = max(0.0, float(time_s))
        support_progress = min(1.0, t / t_preload_s)
        target_progress = min(1.0, max(0.0, (t - t_preload_s) / t_lift_s))
        forces = tuple(
            value * (target_progress if index == target_index else support_progress)
            for index, value in enumerate(applied)
        )
        return (0.0, 0.0, 0.0, 0.0) + forces

    return command


def classify_reachability(
    rows: Sequence[Mapping[str, str]], corner: str, limits: Mapping[str, float]
) -> dict:
    """Classify one *measured* pit-interior trial, preserving first failure.

    A candidate needs target unloading, all three support loads, direct
    native CG/contact triangle margin, exact tyre/pit clearance, and posture
    bounds at every registered interior sample. This is a Gate-2 search
    classifier, not full sequential expert-strategy acceptance.
    """
    if corner not in _CORNERS:
        raise ValueError("unknown target corner")
    required = (
        "pit_start_m", "pit_end_m", "pit_center_y_m", "pit_width_m",
        "pit_depth_m", "edge_transition_m", "tyre_radius_m",
        "interior_start_m", "interior_end_m", "cg_margin_m", "cg_error_m",
        "support_floor_n", "target_fz_max_n", "roll_max_deg",
        "yaw_max_deg", "lateral_max_m", "jounce_min_m", "jounce_max_m",
    )
    missing = [key for key in required if key not in limits]
    if missing:
        raise ValueError("missing reachability limits: " + ", ".join(missing))
    target_index = _CORNERS.index(corner)
    target_tyre = _TYRES[target_index]
    supports = tuple(c for c in _CORNERS if c != corner)
    scenario = PotholeScenario(
        start_station_m=float(limits["pit_start_m"]),
        length_m=float(limits["pit_end_m"] - limits["pit_start_m"]),
        center_y_m=float(limits["pit_center_y_m"]),
        width_m=float(limits["pit_width_m"]),
        depth_m=float(limits["pit_depth_m"]),
        edge_transition_m=float(limits["edge_transition_m"]),
    )
    interior = [
        row for row in rows
        if float(limits["interior_start_m"])
        <= float(row["exp_X_" + target_tyre])
        <= float(limits["interior_end_m"])
    ]
    if not interior:
        return {"status": "feasibility_unresolved",
                "binding_constraint": "missing_pit_window",
                "evidence": {"interior_samples": 0}}

    worst = {"target_fz_n": 0.0, "min_support_load_n": math.inf,
             "cg_margin_m": math.inf, "gap_m": math.inf,
             "roll_deg": 0.0, "yaw_deg": 0.0, "lateral_m": 0.0,
             "min_jounce_m": math.inf, "max_jounce_m": -math.inf}
    first_failure = None
    first_time = None
    for row in interior:
        fz_target = float(row["exp_Fz_" + target_tyre])
        support_loads = [
            float(row["exp_Fz_" + _TYRES[_CORNERS.index(c)]]) for c in supports
        ]
        contacts, cg = native_support_geometry(row, supports)
        margin = support_triangle_margin_m(contacts, cg) - float(limits["cg_error_m"])
        wheel_xyz = tuple(float(row["exp_" + axis + "_" + target_tyre])
                          for axis in ("X", "Y", "Z"))
        gap = pothole_tyre_envelope_gap_m(
            scenario, wheel_xyz, float(limits["tyre_radius_m"])
        ) - float(limits.get("gap_error_m", 0.0))
        roll = abs(float(row["exp_Roll_E"]))
        yaw = abs(float(row["exp_Yaw"]))
        lateral = abs(float(row["exp_Yo"]))
        jounces = [float(row["exp_Jnc_" + tyre]) / 1000.0 for tyre in _TYRES]
        worst["target_fz_n"] = max(worst["target_fz_n"], fz_target)
        worst["min_support_load_n"] = min(worst["min_support_load_n"], *support_loads)
        worst["cg_margin_m"] = min(worst["cg_margin_m"], margin)
        worst["gap_m"] = min(worst["gap_m"], gap)
        worst["roll_deg"] = max(worst["roll_deg"], roll)
        worst["yaw_deg"] = max(worst["yaw_deg"], yaw)
        worst["lateral_m"] = max(worst["lateral_m"], lateral)
        worst["min_jounce_m"] = min(worst["min_jounce_m"], *jounces)
        worst["max_jounce_m"] = max(worst["max_jounce_m"], *jounces)
        checks = (
            ("target_fz", fz_target <= float(limits["target_fz_max_n"])),
            ("support_load", min(support_loads) >= float(limits["support_floor_n"])),
            ("cg_margin", margin >= float(limits["cg_margin_m"])),
            ("tyre_gap", gap > 0.0),
            ("suspension_travel",
             min(jounces) > float(limits["jounce_min_m"])
             and max(jounces) < float(limits["jounce_max_m"])),
            ("roll", roll <= float(limits["roll_max_deg"])),
            ("yaw", yaw <= float(limits["yaw_max_deg"])),
            ("lateral", lateral <= float(limits["lateral_max_m"])),
        )
        if first_failure is None:
            for name, passed in checks:
                if not passed:
                    first_failure = name
                    first_time = float(row["time_s"])
                    break
    return {
        "status": "admissible_candidate" if first_failure is None else "feasibility_unresolved",
        "binding_constraint": first_failure,
        "first_failure_time_s": first_time,
        "evidence": {"interior_samples": len(interior), **worst},
    }
