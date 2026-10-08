from dataclasses import FrozenInstanceError
import math

import pytest


def test_command_requires_four_finite_values():
    from ddevsim.runtime.contracts import ControlCommand
    for values in ((0.,) * 3, (0., 0., 0., float('nan')), (0., 0., 0., float('inf'))):
        with pytest.raises(ValueError):
            ControlCommand(values, (0.,) * 4)
        with pytest.raises(ValueError):
            ControlCommand((0.,) * 4, values)
    with pytest.raises(ValueError):
        ControlCommand((0.,) * 4, (0.,) * 4, float('nan'))


def test_context_freezes_nested_configuration_and_rejects_invalid_period():
    from ddevsim.runtime.contracts import StrategyContext
    source = {'limits': {'force_n': [100., 200.]}}
    context = StrategyContext('wheel_lift', 'vehicle', 'scene', .02, source, {})
    source['limits']['force_n'][0] = 999.
    assert context.config['limits']['force_n'][0] == 100.
    with pytest.raises(TypeError):
        context.config['limits']['force_n'] = (0.,)
    for period in (0., -1., math.nan, math.inf):
        with pytest.raises(ValueError):
            StrategyContext('wheel_lift', 'vehicle', 'scene', period, {}, {})


def test_missing_observations_are_not_zero_filled():
    from ddevsim.runtime.contracts import Observation
    observation = Observation(1., .96, True, {'Vx': 1.})
    with pytest.raises(ValueError, match='Fz_R1'):
        observation.require(('Vx', 'Fz_R1'))
    with pytest.raises(ValueError):
        Observation(1., 2., True, {})
    with pytest.raises(ValueError):
        Observation(1., .96, True, {'Vx': math.nan})
    with pytest.raises(ValueError):
        Observation(1., .96, False, {}).require(('Vx',))


def test_unknown_strategy_rejected_and_shallow_control_explicitly_unimplemented():
    from ddevsim.runtime.registry import load_strategy
    from ddevsim.runtime.contracts import Observation, StrategyContext
    with pytest.raises(ValueError):
        load_strategy('typo')
    controller = load_strategy('shallow_press')()
    controller.reset(StrategyContext('shallow_press', 'vehicle', 'scene', .02, {}, {}))
    assert not controller.status().completed
    with pytest.raises(NotImplementedError, match='shallow_press'):
        controller.step(Observation(0., 0., True, {}))
    assert controller.summary()['task_status'] == 'NOT_IMPLEMENTED'
