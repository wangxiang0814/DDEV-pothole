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
        "condition_number": condition if np.isfinite(condition) else None,
        "condition_is_infinite": not np.isfinite(condition),
        "requires_damping": effective_rank < min(g.shape) or condition > 1 / relative_cutoff,
    }


def assemble_settled_gains(observations: dict, *, amplitude_n: float) -> dict:
    """Publish central differences only when every paired native probe settled."""
    wheels = ('FL', 'FR', 'RL', 'RR')
    for wheel in wheels:
        for sign in ('pos', 'neg'):
            key = f'{wheel}_{sign}'
            case = observations[key]
            if (case['native']['status'] != 'COMPLETED' or
                    case['settle_status'] != 'SETTLED'):
                raise ValueError(f'unusable identification probe: {key}')
    fields = set(observations['FL_pos']['response'])
    if any(set(case['response']) != fields for case in observations.values()):
        raise ValueError('probe response fields differ')
    gains = {field: np.column_stack([
        central_difference(observations[f'{wheel}_pos']['response'][field],
                           observations[f'{wheel}_neg']['response'][field],
                           amplitude_n) for wheel in wheels]).tolist()
             for field in sorted(fields)}
    return {'gains': gains, 'G_F_diagnostic': diagnose_gain(
        gains['Fz_n'], relative_cutoff=1e-3)}


def validated_probe_window(native, rows, *, start_s, end_s, min_samples):
    """Reject an incomplete native run before reductions on its sample window."""
    if native['status'] != 'COMPLETED':
        raise ValueError('native probe did not complete')
    tail = [r for r in rows if start_s <= float(r['time_s']) <= end_s]
    if len(tail) < min_samples:
        raise ValueError('insufficient native identification window')
    return tail


def validated_contact_gains(bundle, *, model_sha256):
    """Reject stale or failed contact-mode gains before constructing feedback."""
    result = {}
    for mode in ('FOUR_CONTACT', 'FR', 'RR'):
        record = bundle['modes'][mode]
        if record['source_model_sha256'] != model_sha256:
            raise ValueError(f'identified model mismatch: {mode}')
        if record['status'] != 'PASS' or record['mode'] != mode:
            raise ValueError(f'unusable identified contact mode: {mode}')
        gain = np.asarray(record['gains']['Fz_n'], dtype=float)
        if gain.shape != (4, 4) or not np.isfinite(gain).all():
            raise ValueError(f'invalid wheel-load gain: {mode}')
        result[mode] = gain
    return result
