"""Version 1 strategy boundary: FL/FR/RL/RR, SI values, instance state."""
from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from types import MappingProxyType
from typing import Mapping, Protocol


def freeze(value):
    """Copy nested JSON configuration so callers cannot mutate another run."""
    if isinstance(value, Mapping):
        return MappingProxyType({key: freeze(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(freeze(item) for item in value)
    if value is None or isinstance(value, (str, bool, int, float, Path)):
        if isinstance(value, float) and not math.isfinite(value):
            raise ValueError('configuration must contain finite numbers')
        return value
    raise TypeError('configuration must contain immutable JSON values or paths')


@dataclass(frozen=True)
class StrategyContext:
    strategy_id: str
    vehicle_id: str
    scenario_id: str
    control_period_s: float
    config: Mapping
    assets: Mapping
    contract_version: int = 1

    def __post_init__(self):
        if not math.isfinite(self.control_period_s) or self.control_period_s <= 0:
            raise ValueError('control_period_s must be positive and finite')
        if self.contract_version != 1:
            raise ValueError('unsupported strategy contract version')
        object.__setattr__(self, 'config', freeze(self.config))
        object.__setattr__(self, 'assets', freeze(self.assets))


@dataclass(frozen=True)
class Observation:
    time_s: float
    observed_at_s: float
    valid: bool
    values_si: Mapping[str, float]

    def __post_init__(self):
        if (not math.isfinite(self.time_s) or not math.isfinite(self.observed_at_s)
                or self.observed_at_s > self.time_s + 1e-9):
            raise ValueError('observation timestamps must be finite and not in the future')
        object.__setattr__(self, 'values_si', freeze(self.values_si))

    def require(self, names) -> None:
        if not self.valid:
            raise ValueError('observation is invalid')
        for name in names:
            if name not in self.values_si or not math.isfinite(self.values_si[name]):
                raise ValueError('missing or invalid observation: ' + name)


@dataclass(frozen=True)
class ControlCommand:
    wheel_torque_nm: tuple[float, ...]
    suspension_force_n: tuple[float, ...]
    steering_wheel_angle_rad: float | None = None

    def __post_init__(self):
        for name in ('wheel_torque_nm', 'suspension_force_n'):
            values = tuple(float(v) for v in getattr(self, name))
            if len(values) != 4 or not all(math.isfinite(v) for v in values):
                raise ValueError(name + ' requires four finite FL/FR/RL/RR values')
            object.__setattr__(self, name, values)
        if self.steering_wheel_angle_rad is not None and not math.isfinite(self.steering_wheel_angle_rad):
            raise ValueError('steering_wheel_angle_rad must be finite')


@dataclass(frozen=True)
class StrategyStatus:
    phase: str
    completed: bool = False
    abort_reason: str | None = None


class Strategy(Protocol):
    def reset(self, context: StrategyContext) -> None: ...
    def step(self, observation: Observation) -> ControlCommand: ...
    def status(self) -> StrategyStatus: ...
    def summary(self) -> dict: ...
