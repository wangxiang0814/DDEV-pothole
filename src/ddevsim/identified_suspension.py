"""One-step, measured-gain allocation for four-corner suspension thrust."""

from __future__ import annotations

from typing import Any, Mapping

from .bounded_allocation import bounded_weighted_least_squares

CORNERS = ("FL", "FR", "RL", "RR")


def roll_gain_from_probe(report: Mapping[str, Any]) -> dict[str, float]:
    amplitude = float(report["force_amplitude_n"])
    if amplitude <= 0:
        raise ValueError("probe amplitude must be positive")
    return {corner: float(report["roll_response_deg"][corner]) / amplitude
            for corner in CORNERS}


def allocate_identified_force(
    *, measured_load_n: Mapping[str, float], target_load_n: Mapping[str, float],
    measured_roll_deg: float, target_roll_deg: float,
    gain_command_to_load: Mapping[str, Mapping[str, float]],
    roll_gain_deg_per_n: Mapping[str, float],
    previous_force_n: Mapping[str, float],
    force_min_n: float, force_max_n: float, max_force_step_n: float,
    roll_weight_n_per_deg: float = 1000.0,
    measured_roll_rate_deg_s: float = 0.0,
    roll_rate_damping_s: float = 0.0,
) -> dict[str, float]:
    """Fit incremental wheel-load/roll demand under absolute force and slew bounds.

    The identified gains predict *changes* around the current measurement.  This is
    a one-step constrained allocator, not an unverified exact CarSim/TruckSim inverse.
    """
    if max_force_step_n <= 0 or force_min_n >= force_max_n:
        raise ValueError("invalid actuator bounds")
    rows = [
        [float(gain_command_to_load[command][load]) for command in CORNERS]
        for load in CORNERS
    ]
    target = [
        float(target_load_n[c] - measured_load_n[c]) for c in CORNERS
    ]
    rows.append([roll_weight_n_per_deg * float(roll_gain_deg_per_n[c])
                 for c in CORNERS])
    target.append(roll_weight_n_per_deg * (
        target_roll_deg - measured_roll_deg
        - roll_rate_damping_s * measured_roll_rate_deg_s
    ))
    lower = [max(force_min_n - previous_force_n[c], -max_force_step_n)
             for c in CORNERS]
    upper = [min(force_max_n - previous_force_n[c], max_force_step_n)
             for c in CORNERS]
    increment = bounded_weighted_least_squares(
        rows, target, lower=lower, upper=upper, effort_weight=0.005
    )
    return {c: float(previous_force_n[c]) + increment[i]
            for i, c in enumerate(CORNERS)}
