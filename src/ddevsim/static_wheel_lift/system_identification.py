"""Small symmetric force perturbation identification and damped allocation math."""

from __future__ import annotations

import numpy as np


def central_difference(positive, negative, amplitude_n: float) -> np.ndarray:
    """Return response per newton from matching positive/negative settled states."""
    if not np.isfinite(amplitude_n) or amplitude_n <= 0:
        raise ValueError("amplitude_n must be positive")
    pos, neg = np.asarray(positive, dtype=float), np.asarray(negative, dtype=float)
    if pos.shape != neg.shape or not np.isfinite(pos).all() or not np.isfinite(neg).all():
        raise ValueError("matching finite response arrays required")
    return (pos - neg) / (2.0 * amplitude_n)


def damped_least_squares(gain, desired_change, ridge: float) -> np.ndarray:
    """Solve the ridge-regularized least-squares problem without an ordinary inverse."""
    g = np.asarray(gain, dtype=float)
    target = np.asarray(desired_change, dtype=float)
    if g.ndim != 2 or target.shape != (g.shape[0],) or not np.isfinite(g).all() or not np.isfinite(target).all():
        raise ValueError("finite matrix and matching target required")
    if not np.isfinite(ridge) or ridge <= 0:
        raise ValueError("ridge must be positive")
    return np.linalg.solve(g.T @ g + ridge * np.eye(g.shape[1]), g.T @ target)


def diagnose_gain(gain, *, relative_cutoff: float) -> dict:
    """Report algebraic and noise-aware rank without treating a tiny mode as control authority."""
    g = np.asarray(gain, dtype=float)
    if g.ndim != 2 or not np.isfinite(g).all() or not 0 < relative_cutoff < 1:
        raise ValueError("invalid gain or relative cutoff")
    singular = np.linalg.svd(g, compute_uv=False)
    effective_rank = int(np.count_nonzero(singular > singular[0] * relative_cutoff))
    condition = float(np.linalg.cond(g))
    return {
        "algebraic_rank": int(np.linalg.matrix_rank(g)),
        "effective_rank": effective_rank,
        "relative_cutoff": relative_cutoff,
        "singular_values": singular.tolist(),
        "condition_number": condition,
        "requires_damping": effective_rank < min(g.shape) or condition > 1 / relative_cutoff,
    }
