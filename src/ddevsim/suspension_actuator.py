"""Bounded external suspension thrust; native spring and damper stay in TruckSim."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class SuspensionActuator:
    force_min_n: float
    force_max_n: float
    slew_n_per_s: float
    time_constant_s: float
    actual_n: float = 0.0

    def __post_init__(self) -> None:
        if (self.force_min_n >= self.force_max_n or self.slew_n_per_s <= 0
                or self.time_constant_s <= 0):
            raise ValueError("invalid active-suspension force or response limits")

    def step(self, requested_n: float, dt_s: float) -> float:
        if dt_s <= 0 or not math.isfinite(requested_n):
            raise ValueError("force command and time step must be finite and valid")
        requested_n = min(self.force_max_n, max(self.force_min_n, requested_n))
        lagged = self.actual_n + (requested_n - self.actual_n) * (
            1.0 - math.exp(-dt_s / self.time_constant_s)
        )
        delta = min(self.slew_n_per_s * dt_s,
                    max(-self.slew_n_per_s * dt_s, lagged - self.actual_n))
        self.actual_n = min(self.force_max_n,
                            max(self.force_min_n, self.actual_n + delta))
        return self.actual_n


def signed_probe_cases(amplitude_n: float) -> tuple[tuple[str, float], ...]:
    if amplitude_n <= 0 or not math.isfinite(amplitude_n):
        raise ValueError("probe amplitude must be finite and positive")
    return tuple((corner, sign * amplitude_n)
                 for corner in ("FL", "FR", "RL", "RR")
                 for sign in (1.0, -1.0))
