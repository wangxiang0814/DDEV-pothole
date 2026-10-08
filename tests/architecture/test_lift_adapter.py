import math
import pytest


def test_si_adapter_preserves_raw_controller_commands():
    from ddevsim.runtime.contracts import StrategyContext, Observation
    from ddevsim.runtime.registry import load_strategy
    from ddevsim.strategies.wheel_lift.front import EXPORTS
    from ddevsim.units import to_si
    import json
    from pathlib import Path
    data = json.loads((Path(__file__).parent / 'fixtures/lift_baseline.json').read_text())
    adapter = load_strategy('wheel_lift')()
    adapter.reset(StrategyContext('wheel_lift', 'tuned_ii', 'scene', .02,
                                 {'scenario': data['scenario']}, {}))
    for event in data['cases'][0]['events'][:30]:
        observation = Observation(event['time_s'], event['time_s'], True,
                                  {name: to_si(name, value) for name, value in zip(EXPORTS, event['exports'])})
        command = adapter.step(observation)
        assert command.wheel_torque_nm + command.suspension_force_n == pytest.approx(event['command'])
        assert command.steering_wheel_angle_rad == pytest.approx(0.)
    assert not adapter.status().completed
    assert adapter.summary()['task_status'] != 'PASS'


def test_adapter_refuses_uninitialized_and_missing_observations():
    from ddevsim.runtime.registry import load_strategy
    from ddevsim.runtime.contracts import StrategyContext, Observation
    adapter = load_strategy('wheel_lift')()
    with pytest.raises(RuntimeError):
        adapter.step(Observation(0., 0., True, {}))
    adapter.reset(StrategyContext('wheel_lift', 'v', 's', .02,
                                 {'scenario': {'friction': .7}}, {}))
    with pytest.raises(ValueError):
        adapter.step(Observation(0., 0., True, {}))


def test_rear_coupled_gain_uses_instance_configuration():
    import json
    from ddevsim.runtime.contracts import StrategyContext
    from ddevsim.strategies.wheel_lift.adapter import WheelLiftStrategy
    from ddevsim.strategies.wheel_lift.front import EVIDENCE
    adapter = WheelLiftStrategy()
    adapter.reset(StrategyContext('wheel_lift', 'v', 's', .02,
                                 {'scenario': {'friction': .7},
                                  'rear': {'preload_feedback_fl_per_fr': 0.}}, {}))
    gain = json.loads((EVIDENCE / 'gain_matrix.json').read_text())['gains']['Fz_n']
    assert adapter.controller.rear.unload_feedback.gain == pytest.approx(gain[3][1])
