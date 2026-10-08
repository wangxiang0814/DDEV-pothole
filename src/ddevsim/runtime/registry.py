"""Load only the selected strategy; no cross-strategy imports."""
from importlib import import_module

_STRATEGIES = {
    'wheel_lift': ('ddevsim.strategies.wheel_lift.adapter', 'WheelLiftStrategy'),
    'shallow_press': ('ddevsim.strategies.shallow_press.controller', 'ShallowPressStrategy'),
}


def load_strategy(strategy_id: str) -> type:
    if strategy_id not in _STRATEGIES:
        raise ValueError('unknown strategy: ' + strategy_id)
    module, name = _STRATEGIES[strategy_id]
    return getattr(import_module(module), name)
