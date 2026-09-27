"""One trusted, constrained suspension-force increment for static FR unloading.

The identified gain is valid locally. This allocator plans only one short
increment; the TruckSim response must be measured before another increment.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import LinearConstraint, linprog, minimize


@dataclass(frozen=True)
class LocalUnloadStep:
    status: str
    force_n: tuple[float, float, float, float] | None
    predicted_fz_n: tuple[float, float, float, float] | None
    predicted_lambda_min: float | None
    reason: str


def plan_local_unload_step(
    support_xy, com_xy, g_com, zmp_xy, g_zmp, fz_n, g_fz,
    travel_m, g_travel, attitude_rad, g_attitude, *, force_bound_n,
    travel_lower_m, travel_upper_m, attitude_bound_rad,
    support_floor_n, lambda_safe, margin_reserve, fr_drop_target_n,
    stability_margin_target=None,
) -> LocalUnloadStep:
    """Find a minimum-effort FR unload step with a hard stability target."""
    shapes = dict(support_xy=(3, 2), com_xy=(2,), g_com=(2, 4),
                  zmp_xy=(2,), g_zmp=(2, 4), fz_n=(4,), g_fz=(4, 4),
                  travel_m=(4,), g_travel=(4, 4), attitude_rad=(2,),
                  g_attitude=(2, 4), travel_lower_m=(4,), travel_upper_m=(4,))
    values = locals()
    a = {key: np.asarray(values[key], dtype=float) for key in shapes}
    if any(a[key].shape != shape or not np.isfinite(a[key]).all()
           for key, shape in shapes.items()):
        raise ValueError("invalid local allocator model")
    bounds = np.asarray(force_bound_n, dtype=float)
    if bounds.ndim == 0:
        bounds = np.full(4, float(bounds))
    scalars = (attitude_bound_rad, support_floor_n,
               lambda_safe, margin_reserve, fr_drop_target_n)
    if (bounds.shape != (4,) or not np.isfinite(bounds).all() or
            np.any(bounds <= 0) or not all(np.isfinite(scalars)) or
            attitude_bound_rad <= 0 or support_floor_n < 0 or
            fr_drop_target_n <= 0 or margin_reserve < 0 or
            not 0 <= lambda_safe + margin_reserve < 1 / 3 or
            np.any(a["travel_lower_m"] >= a["travel_upper_m"])):
        raise ValueError("invalid local allocator bounds")
    if stability_margin_target is not None and (
            not np.isfinite(stability_margin_target) or
            not 0 <= stability_margin_target < 1 / 3):
        raise ValueError("invalid stability margin target")
    required_margin = max(lambda_safe + margin_reserve,
                          stability_margin_target if stability_margin_target is not None
                          else 0.)

    points = a["support_xy"]
    frame = np.vstack(((points - points[0]).T, np.ones(3)))
    if abs(np.linalg.det(frame)) < 1e-9:
        raise ValueError("degenerate support triangle")
    rows, rhs = [], []
    min_lambda = []
    for xy_key, gain_key in (("com_xy", "g_com"), ("zmp_xy", "g_zmp")):
        initial = np.linalg.solve(frame, np.r_[a[xy_key] - points[0], 1.])
        jacobian = np.linalg.solve(frame, np.vstack((a[gain_key], np.zeros((1, 4)))))
        rows.extend(-jacobian)
        rhs.extend(initial - required_margin)
        min_lambda.append((initial, jacobian))
    floor = np.array([support_floor_n, 0., support_floor_n, support_floor_n])
    rows.extend(-a["g_fz"])
    rhs.extend(a["fz_n"] - floor)
    for gain, bound in (
        (a["g_travel"], a["travel_upper_m"] - a["travel_m"]),
        (-a["g_travel"], a["travel_m"] - a["travel_lower_m"]),
        (a["g_attitude"], attitude_bound_rad - a["attitude_rad"]),
        (-a["g_attitude"], attitude_bound_rad + a["attitude_rad"]),
    ):
        rows.extend(gain)
        rhs.extend(bound)
    rows.append(a["g_fz"][1])
    rhs.append(-fr_drop_target_n)
    matrix = np.asarray(rows, dtype=float) * bounds
    limits = np.asarray(rhs, dtype=float)
    box = [(-1., 1.)] * 4
    feasible = linprog(np.zeros(4), A_ub=matrix, b_ub=limits,
                       bounds=box, method="highs")
    if not feasible.success:
        return LocalUnloadStep("INFEASIBLE", None, None, None, feasible.message)
    fitted = minimize(lambda x: 0.5 * float(x @ x), feasible.x,
                      jac=lambda x: x, bounds=box, method="SLSQP",
                      constraints=LinearConstraint(matrix, -np.inf, limits),
                      options={"ftol": 1e-12, "maxiter": 200})
    if not fitted.success or np.max(matrix @ fitted.x - limits) > 1e-6:
        return LocalUnloadStep("SOLVER_FAILED", None, None, None, fitted.message)
    force = fitted.x * bounds
    predicted_load = a["fz_n"] + a["g_fz"] @ force
    margin = min(float(np.min(initial + jacobian @ force))
                 for initial, jacobian in min_lambda)
    return LocalUnloadStep("OPTIMAL", tuple(force), tuple(predicted_load),
                           margin, "local endpoint only; native path unverified")
