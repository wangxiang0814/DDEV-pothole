"""Guarded rank-one local-gain update from a native TruckSim step."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class OnlineGainUpdate:
    status: str
    gains: dict[str, list[list[float]]] | None
    max_prediction_error: dict[str, float]


def secant_gain_update(gains, baseline, observed, delta_force_n, *,
                       max_prediction_error) -> OnlineGainUpdate:
    """Use a measured secant only when the previous prediction was accurate.

    The correction is a damp-free rank-one secant along the applied direction;
    it never inverts the ill-conditioned four-wheel load matrix.
    """
    force = np.asarray(delta_force_n, dtype=float)
    if force.ndim != 1 or not np.isfinite(force).all() or force @ force <= 0:
        raise ValueError("nonzero finite force increment required")
    if set(gains) != set(baseline) or set(gains) != set(observed) or set(gains) != set(max_prediction_error):
        raise ValueError("gain and response keys must match")
    errors = {}
    corrections = {}
    for key, matrix in gains.items():
        gain = np.asarray(matrix, dtype=float)
        before = np.asarray(baseline[key], dtype=float)
        after = np.asarray(observed[key], dtype=float)
        limit = float(max_prediction_error[key])
        if (gain.ndim != 2 or gain.shape != (before.size, force.size) or
                before.shape != after.shape or before.ndim != 1 or
                not all(np.isfinite(x).all() for x in (gain, before, after)) or
                not np.isfinite(limit) or limit <= 0):
            raise ValueError("invalid gain update dimensions or limits")
        residual = after - before - gain @ force
        errors[key] = float(np.max(np.abs(residual)))
        corrections[key] = (gain + np.outer(residual, force) / float(force @ force)).tolist()
    if any(errors[key] > max_prediction_error[key] for key in errors):
        return OnlineGainUpdate("REIDENTIFY_REQUIRED", None, errors)
    return OnlineGainUpdate("PASS", corrections, errors)
