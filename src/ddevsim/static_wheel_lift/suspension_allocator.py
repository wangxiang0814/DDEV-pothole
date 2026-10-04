"""One trusted, constrained suspension-force increment for static FR unloading.

The identified gain is valid locally. This allocator plans only one short
increment; the TruckSim response must be measured before another increment.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import LinearConstraint, linprog, minimize
from .config import SUPPORT_QP


@dataclass(frozen=True)
class LocalUnloadStep:
    status: str
    force_n: tuple[float, float, float, float] | None
    predicted_fz_n: tuple[float, float, float, float] | None
    predicted_lambda_min: float | None
    reason: str


@dataclass(frozen=True)
class SupportIncrement:
    status: str
    force_n: np.ndarray | None
    predicted_fz_n: np.ndarray | None
    predicted_margin: float | None
    cost: float | None
    solver: str = 'NONE'


def allocate_support_increment(*, lifted_corner, contacts_xy, fz_n, fz_target_n,
                               com_xy, zmp_xy, attitude_rad, attitude_target_rad,
                               travel_m, wheel_height_m, gains, base_force_n, previous_applied_n,
                               correction_n, dt_s, config=SUPPORT_QP):
    """Constrained three-input local QP with known lift-input coupling.

    The predicted endpoint is quasi-static, not a dynamic MPC prediction.
    Bounds apply to this increment; the nonlinear plant remains authoritative.
    """
    order = ('FL', 'FR', 'RL', 'RR')
    if lifted_corner not in order or not np.isfinite(dt_s) or dt_s <= 0:
        raise ValueError('invalid support QP contact/time')
    support = np.array([i for i, n in enumerate(order) if n != lifted_corner])
    lifted = order.index(lifted_corner)
    arrays = {
        'xy': (contacts_xy, (4, 2)), 'fz': (fz_n, (4,)),
        'target': (fz_target_n, (4,)), 'com': (com_xy, (2,)),
        'zmp': (zmp_xy, (2,)), 'att': (attitude_rad, (2,)),
        'att_target': (attitude_target_rad, (2,)), 'travel': (travel_m, (4,)),
        'height': (wheel_height_m, (4,)),
        'base': (base_force_n, (4,)), 'last': (previous_applied_n, (4,)),
        'corr': (correction_n, (3,)),
    }
    a = {k: np.asarray(v, dtype=float) for k, (v, _) in arrays.items()}
    if any(a[k].shape != shape or not np.isfinite(a[k]).all()
           for k, (_, shape) in arrays.items()):
        raise ValueError('invalid support QP observation')
    shapes = {'Fz_n': (4, 4), 'CoM_xy_m': (2, 4), 'ZMP_xy_m': (2, 4),
              'attitude_rad': (2, 4), 'travel_m': (4, 4), 'wheel_height_m': (4, 4)}
    g = {k: np.asarray(gains[k], dtype=float) for k in shapes}
    if any(g[k].shape != shape or not np.isfinite(g[k]).all() for k, shape in shapes.items()):
        raise ValueError('invalid support QP gains')
    expansion = np.eye(4)[:, support]
    current_force = a['base'] + expansion @ a['corr']
    known = current_force - a['last']
    scale = config.force_slew_n_s * dt_s
    lower = np.maximum.reduce([np.full(3, -scale),
                               -config.correction_limit_n - a['corr'],
                               -config.force_limit_n - current_force[support]]) / scale
    upper = np.minimum.reduce([np.full(3, scale),
                               config.correction_limit_n - a['corr'],
                               config.force_limit_n - current_force[support]]) / scale
    if np.any(lower > upper) or abs(current_force[lifted]) > config.force_limit_n:
        return SupportIncrement('INFEASIBLE', None, None, None, None)
    start = {key: a[obs] + g[key] @ known for key, obs in
             [('Fz_n', 'fz'), ('CoM_xy_m', 'com'), ('ZMP_xy_m', 'zmp'),
              ('attitude_rad', 'att'), ('travel_m', 'travel'), ('wheel_height_m', 'height')]}
    models = {k: v @ expansion * scale for k, v in g.items()}
    rows, rhs = [], []
    floor = np.full(4, config.support_floor_n)
    floor[lifted] = 0.
    # The lifted output has zero gain in a validated swing model.
    # Keep zero-load rows out of the solver to avoid a degenerate constraint.
    for indices, bound, sign in [(support, floor[support], -1.),
                                  (np.arange(4), np.full(4, config.max_load_n), 1.)]:
        rows.extend(sign * models['Fz_n'][indices])
        rhs.extend(sign * (bound - start['Fz_n'][indices]))
    rows.append(models['Fz_n'][lifted]); rhs.append(config.swing_load_ceiling_n - start['Fz_n'][lifted])
    rows.append(-models['wheel_height_m'][lifted])
    rhs.append(start['wheel_height_m'][lifted] - config.clearance_floor_m)
    for key, low, high in [('travel_m', config.travel_lower_m, config.travel_upper_m),
                           ('attitude_rad', -np.deg2rad(config.attitude_limit_deg),
                            np.deg2rad(config.attitude_limit_deg))]:
        rows.extend(models[key]); rhs.extend(high - start[key])
        rows.extend(-models[key]); rhs.extend(start[key] - low)
    triangle = a['xy'][support]
    frame = np.vstack(((triangle - triangle[0]).T, np.ones(3)))
    margins = []
    for key in ('CoM_xy_m', 'ZMP_xy_m'):
        bary = np.linalg.solve(frame, np.r_[start[key] - triangle[0], 1.])
        derivative = np.linalg.solve(frame, np.vstack((models[key], np.zeros((1, 3)))))
        rows.extend(-derivative); rhs.extend(bary - config.lambda_floor)
        margins.append((bary, derivative))
    matrix, limit = np.asarray(rows), np.asarray(rhs)
    box = list(zip(lower, upper))
    load_scale = config.load_scale_n
    att_scale = np.deg2rad(config.attitude_scale_deg)
    m = np.vstack((np.sqrt(config.load_weight) * models['Fz_n'] / load_scale,
                   np.sqrt(config.attitude_weight) * models['attitude_rad'] / att_scale,
                   np.sqrt(config.increment_weight) * np.eye(3),
                   np.sqrt(config.correction_weight) * scale / config.correction_limit_n * np.eye(3)))
    target = np.r_[np.sqrt(config.load_weight) * config.tracking_gain * (a['target'] - start['Fz_n']) / load_scale,
                   np.sqrt(config.attitude_weight) * config.tracking_gain * (a['att_target'] - start['attitude_rad']) / att_scale,
                   np.zeros(3), -np.sqrt(config.correction_weight) * a['corr'] / config.correction_limit_n]
    if config.direct_feasible_solve:
        # A feasible unconstrained least-squares minimum is also the constrained
        # global minimum. SVD avoids an inverse or squared condition number.
        direct = np.linalg.lstsq(m, target, rcond=None)[0]
        if (np.isfinite(direct).all() and np.all(direct >= lower) and
                np.all(direct <= upper) and
                np.max(matrix @ direct - limit) <= config.constraint_tolerance):
            force = current_force + expansion @ direct * scale
            predicted = start['Fz_n'] + models['Fz_n'] @ direct
            margin = min(float(min(b + d @ direct)) for b, d in margins)
            return SupportIncrement('OPTIMAL', force, predicted, margin,
                                    .5 * float(np.sum((m @ direct - target)**2)), 'DIRECT')
    initial = np.clip(np.zeros(3), lower, upper)
    if np.max(matrix @ initial - limit) > config.constraint_tolerance:
        feasible = linprog(np.zeros(3), A_ub=matrix, b_ub=limit, bounds=box, method='highs')
        if not feasible.success:
            return SupportIncrement('INFEASIBLE', None, None, None, None)
        initial = feasible.x
    fitted = minimize(lambda v: .5 * float(np.sum((m @ v - target)**2)), initial,
                      jac=lambda v: m.T @ (m @ v - target), bounds=box,
                      constraints=LinearConstraint(matrix, -np.inf, limit), method='SLSQP',
                      options={'ftol': config.solver_tolerance, 'maxiter': config.max_iterations})
    if not fitted.success or np.max(matrix @ fitted.x - limit) > config.constraint_tolerance:
        return SupportIncrement('SOLVER_FAILED', None, None, None, None, 'SLSQP')
    force = current_force + expansion @ fitted.x * scale
    predicted = start['Fz_n'] + models['Fz_n'] @ fitted.x
    margin = min(float(min(b + d @ fitted.x)) for b, d in margins)
    return SupportIncrement('OPTIMAL', force, predicted, margin, float(fitted.fun), 'SLSQP')


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
