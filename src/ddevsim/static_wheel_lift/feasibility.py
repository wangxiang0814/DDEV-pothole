"""Conservative local, force-limited three-support feasibility certificate.

This is a linearized diagnostic. An infeasible result forbids progressing to
large-force native lift trials under this model; it is not a proof of global
nonlinear physical impossibility.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import linprog


@dataclass(frozen=True)
class FeasibilityResult:
    status: str
    best_lambda_min: float
    best_force_n: tuple[float, float, float, float]
    predicted_com_xy: tuple[float, float]
    method: str = "linearized_contact_fixed_lp"


@dataclass(frozen=True)
class PreloadFeasibilityResult:
    status: str
    best_lambda_min: float
    force_n: tuple[float, float, float, float] | None
    predicted_fz_n: tuple[float, float, float, float] | None
    predicted_com_xy: tuple[float, float] | None
    predicted_zmp_xy: tuple[float, float] | None
    predicted_travel_m: tuple[float, float, float, float] | None
    reason: str


def check_preload_feasibility(
    support_xy, com_xy, g_com, zmp_xy, g_zmp, fz_n, g_fz,
    travel_m, g_travel, attitude_rad, g_attitude, *,
    force_bound_n: float, travel_lower_m, travel_upper_m,
    attitude_bound_rad: float, support_floor_n: float,
    fr_max_n: float | None, lambda_safe: float,
) -> PreloadFeasibilityResult:
    """Optimistic fixed-contact linear endpoint gate for I_I preload.

    Maximises the lesser of measured CoM and load-based ZMP barycentric
    margins. An endpoint candidate still requires a safe native path test.
    """
    arrays = dict(
        support_xy=(support_xy, (3, 2)), com_xy=(com_xy, (2,)),
        g_com=(g_com, (2, 4)), zmp_xy=(zmp_xy, (2,)),
        g_zmp=(g_zmp, (2, 4)), fz_n=(fz_n, (4,)),
        g_fz=(g_fz, (4, 4)), travel_m=(travel_m, (4,)),
        g_travel=(g_travel, (4, 4)), attitude_rad=(attitude_rad, (2,)),
        g_attitude=(g_attitude, (2, 4)),
        travel_lower_m=(travel_lower_m, (4,)),
        travel_upper_m=(travel_upper_m, (4,)),
    )
    a = {name: np.asarray(value, dtype=float) for name, (value, _) in arrays.items()}
    if any(a[name].shape != shape or not np.isfinite(a[name]).all()
           for name, (_, shape) in arrays.items()):
        raise ValueError("invalid preload model dimensions or values")
    if (force_bound_n <= 0 or attitude_bound_rad <= 0 or support_floor_n < 0
            or not 0 <= lambda_safe < 1/3
            or np.any(a["travel_lower_m"] >= a["travel_upper_m"])
            or fr_max_n is not None and fr_max_n < 0
            or not all(np.isfinite(v) for v in (force_bound_n, attitude_bound_rad,
                                                 support_floor_n, lambda_safe))
            or fr_max_n is not None and not np.isfinite(fr_max_n)):
        raise ValueError("invalid preload bounds")
    points = a["support_xy"]
    # Translate world coordinates before solving geometry (TruckSim X is ~98 m).
    geometry = np.vstack(((points - points[0]).T, np.ones(3)))
    if abs(np.linalg.det(geometry)) < 1e-9:
        raise ValueError("degenerate support triangle")
    constraints, limits = [], []
    for point_key, gain_key in (("com_xy", "g_com"), ("zmp_xy", "g_zmp")):
        initial = np.linalg.solve(geometry, np.r_[a[point_key] - points[0], 1.])
        delta = np.linalg.solve(geometry, np.vstack((a[gain_key], np.zeros((1, 4)))))
        constraints.extend(np.c_[-delta, np.ones(3)])
        limits.extend(initial)
    floor = np.array([support_floor_n, 0., support_floor_n, support_floor_n])
    constraints.extend(np.c_[-a["g_fz"], np.zeros(4)])
    limits.extend(a["fz_n"] - floor)
    if fr_max_n is not None:
        constraints.append(np.r_[a["g_fz"][1], 0.])
        limits.append(fr_max_n - a["fz_n"][1])
    for gain, rhs in (
        (a["g_travel"], a["travel_upper_m"] - a["travel_m"]),
        (-a["g_travel"], a["travel_m"] - a["travel_lower_m"]),
        (a["g_attitude"], attitude_bound_rad - a["attitude_rad"]),
        (-a["g_attitude"], attitude_bound_rad + a["attitude_rad"]),
    ):
        constraints.extend(np.c_[gain, np.zeros(gain.shape[0])])
        limits.extend(rhs)
    result = linprog(np.r_[np.zeros(4), -1.],
                     A_ub=np.asarray(constraints), b_ub=np.asarray(limits),
                     bounds=[(-force_bound_n, force_bound_n)] * 4 + [(None, None)],
                     method="highs")
    if not result.success:
        return PreloadFeasibilityResult(
            "STATIC_LIFT_INFEASIBLE", float("-inf"), None, None, None, None,
            None, result.message)
    force = result.x[:4]
    best = float(result.x[4])
    return PreloadFeasibilityResult(
        "LOCAL_ENDPOINT_CANDIDATE" if best >= lambda_safe - 1e-9
        else "STATIC_LIFT_INFEASIBLE", best,
        tuple(float(v) for v in force),
        tuple(float(v) for v in a["fz_n"] + a["g_fz"] @ force),
        tuple(float(v) for v in a["com_xy"] + a["g_com"] @ force),
        tuple(float(v) for v in a["zmp_xy"] + a["g_zmp"] @ force),
        tuple(float(v) for v in a["travel_m"] + a["g_travel"] @ force),
        "linear endpoint margin below configured safe margin" if best < lambda_safe - 1e-9
        else "endpoint only; native trajectory and settling unverified",
    )


def check_local_feasibility(
    support_xy, com_xy, g_com, fz_n, g_fz, travel_m, g_travel,
    attitude_rad, g_attitude, *, force_bound_n: float,
    travel_bound_m: float, attitude_bound_rad: float,
    support_floor_n: float, lambda_safe: float,
) -> FeasibilityResult:
    """Maximise minimum barycentric coordinate over bounded suspension forces."""
    points = np.asarray(support_xy, dtype=float)
    com = np.asarray(com_xy, dtype=float)
    gc = np.asarray(g_com, dtype=float)
    fz = np.asarray(fz_n, dtype=float)
    gf = np.asarray(g_fz, dtype=float)
    travel = np.asarray(travel_m, dtype=float)
    gt = np.asarray(g_travel, dtype=float)
    attitude = np.asarray(attitude_rad, dtype=float)
    ga = np.asarray(g_attitude, dtype=float)
    if any(a.shape != s for a, s in ((points, (3, 2)), (com, (2,)), (gc, (2, 4)),
                                      (fz, (4,)), (gf, (4, 4)), (travel, (4,)),
                                      (gt, (4, 4)), (attitude, (2,)), (ga, (2, 4)))):
        raise ValueError("invalid local model dimensions")
    if not all(np.isfinite(a).all() for a in (points, com, gc, fz, gf, travel, gt, attitude, ga)):
        raise ValueError("nonfinite local model")
    if min(force_bound_n, travel_bound_m, attitude_bound_rad) <= 0 or not 0 <= lambda_safe < 1/3 or support_floor_n < 0:
        raise ValueError("invalid bounds")
    geometry = np.vstack((points.T, np.ones(3)))
    if abs(np.linalg.det(geometry)) < 1e-9:
        raise ValueError("degenerate support triangle")
    lambda0 = np.linalg.solve(geometry, np.r_[com, 1.])
    d_lambda = np.linalg.solve(geometry, np.vstack((gc, np.zeros((1, 4)))))
    # Variables: four force changes and their attainable minimum lambda.
    zeros = np.zeros((4, 1))
    ab = np.vstack((np.c_[-d_lambda, np.ones(3)],
                    np.c_[-gf, zeros], np.c_[gt, zeros], np.c_[-gt, zeros],
                    np.c_[ga, np.zeros((2, 1))], np.c_[-ga, np.zeros((2, 1))]))
    bb = np.r_[lambda0, fz - np.array([support_floor_n, 0., support_floor_n, support_floor_n]),
               travel_bound_m - travel, travel_bound_m + travel,
               attitude_bound_rad - attitude, attitude_bound_rad + attitude]
    objective = np.r_[np.zeros(4), -1.]
    result = linprog(objective, A_ub=ab, b_ub=bb,
                     bounds=[(-force_bound_n, force_bound_n)] * 4 + [(None, None)], method="highs")
    if not result.success:
        return FeasibilityResult("STATIC_LIFT_INFEASIBLE", float("-inf"),
                                 (0., 0., 0., 0.), tuple(com))
    force = result.x[:4]
    best = float(result.x[4])
    return FeasibilityResult(
        "LOCAL_MODEL_FEASIBLE" if best >= lambda_safe - 1e-9 else "STATIC_LIFT_INFEASIBLE",
        best, tuple(float(v) for v in force), tuple(float(v) for v in com + gc @ force),
    )
