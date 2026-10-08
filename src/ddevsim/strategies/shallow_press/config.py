"""Shallow controller settings are independent of wheel-lift settings."""
from dataclasses import dataclass


@dataclass(frozen=True)
class ShallowPressConfig:
    control_period_s: float = .02
    # No force/comfort thresholds are assumed before the controller is developed.


DEFAULT_CONFIG = ShallowPressConfig()
