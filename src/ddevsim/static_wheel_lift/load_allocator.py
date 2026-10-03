"""Small convex wheel-load QP with exact vertical and ZMP moment balance."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import linprog, minimize


@dataclass(frozen=True)
class LoadAllocation:
    status: str
    fz_ref_n: np.ndarray | None
    cost: float | None


def allocate_loads(
    contacts_xy, total_load_n, target_zmp_xy, previous_n, *, fr_target_n=None,
    lower_n, upper_n, nominal_n=None, smooth_weight=1.0,
    nominal_weight=0.1, fr_weight=10.0, regularization=1e-6,
    lifted_corner='FR', lift_target_n=None,
) -> LoadAllocation:
    """Allocate ordered loads; preserve equilibrium for either contact schedule.

    fr_target_n/fr_weight remain compatible aliases for the historical FR API.
    """
    if lifted_corner not in ('FL', 'FR', 'RL', 'RR'):
        raise ValueError('invalid lifted wheel')
    if lift_target_n is not None and fr_target_n is not None:
        raise ValueError('supply one lift schedule target')
    scheduled_n = lift_target_n if lift_target_n is not None else fr_target_n
    if scheduled_n is None or not np.isfinite(scheduled_n) or scheduled_n < 0:
        raise ValueError('finite nonnegative lift load target required')
    lifted_index = ('FL', 'FR', 'RL', 'RR').index(lifted_corner)
    xy = np.asarray(contacts_xy, dtype=float)
    target = np.asarray(target_zmp_xy, dtype=float)
    prev = np.asarray(previous_n, dtype=float)
    low = np.asarray(lower_n, dtype=float)
    high = np.asarray(upper_n, dtype=float)
    nominal = prev if nominal_n is None else np.asarray(nominal_n, dtype=float)
    if any(a.shape != s for a, s in ((xy, (4, 2)), (target, (2,)),
                                      (prev, (4,)), (low, (4,)),
                                      (high, (4,)), (nominal, (4,)))):
        raise ValueError("invalid wheel-load QP dimensions")
    if not all(np.isfinite(a).all() for a in (xy, target, prev, low, high, nominal)):
        raise ValueError("nonfinite wheel-load QP input")
    if (total_load_n <= 0 or not np.isfinite(total_load_n) or
            np.any(low < 0) or np.any(high < low) or
            min(smooth_weight, nominal_weight, fr_weight, regularization) < 0):
        raise ValueError("invalid wheel-load QP bounds or weights")
    # Normalize loads to avoid poorly scaled N and N*m rows in SLSQP.
    scale = float(total_load_n)
    matrix = np.vstack((np.ones(4), xy.T))
    rhs = np.r_[1.0, target]
    bounds = list(zip(low / scale, high / scale))
    feasible = linprog(np.zeros(4), A_eq=matrix, b_eq=rhs,
                       bounds=bounds, method="highs")
    if not feasible.success:
        return LoadAllocation("INFEASIBLE", None, None)
    p = prev / scale
    n = nominal / scale
    f = scheduled_n / scale

    def cost(x):
        return float(smooth_weight * np.sum((x - p)**2) +
                     nominal_weight * np.sum((x - n)**2) +
                     fr_weight * (x[lifted_index] - f)**2 + regularization * np.sum(x**2))

    result = minimize(cost, feasible.x, method="SLSQP", bounds=bounds,
                      constraints={"type": "eq", "fun": lambda x: matrix @ x - rhs},
                      options={"ftol": 1e-12, "maxiter": 200})
    if not result.success or np.max(np.abs(matrix @ result.x - rhs)) > 1e-7:
        return LoadAllocation("SOLVER_FAILED", None, None)
    return LoadAllocation("OPTIMAL", result.x * scale, cost(result.x))
