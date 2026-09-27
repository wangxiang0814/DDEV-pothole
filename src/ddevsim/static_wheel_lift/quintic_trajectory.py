"""Time-stamped smooth transitions; no wall-clock sleeps."""

from __future__ import annotations


def quintic_step(time_s: float, start_s: float, duration_s: float) -> tuple[float, float, float]:
    """Return normalized position, velocity and acceleration for a 0-to-1 move."""
    if duration_s <= 0:
        raise ValueError("duration_s must be positive")
    u = (time_s - start_s) / duration_s
    if u <= 0:
        return 0.0, 0.0, 0.0
    if u >= 1:
        return 1.0, 0.0, 0.0
    p = 10 * u**3 - 15 * u**4 + 6 * u**5
    v = (30 * u**2 - 60 * u**3 + 30 * u**4) / duration_s
    a = (60 * u - 180 * u**2 + 120 * u**3) / duration_s**2
    return p, v, a
