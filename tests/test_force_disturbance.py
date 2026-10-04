import pytest
import numpy as np
from ddevsim.static_wheel_lift.force_disturbance import SupportForcePulse
from ddevsim.static_wheel_lift.config import SupportForcePulseConfig


def test_pulse_is_phase_triggered_smooth_and_single_use():
    pulse = SupportForcePulse(SupportForcePulseConfig(amplitude_n=200.))
    base = (0.,) * 9
    assert pulse.apply(10., base, stage='FR', phase='THREE_WHEEL_HOLD') == base
    assert pulse.apply(20., base, stage='RR', phase='RR_HOLD') == base
    assert pulse.apply(20.5, base, stage='RR', phase='RR_HOLD') == base
    assert pulse.apply(20.75, base, stage='RR', phase='RR_HOLD')[4] == pytest.approx(100.)
    assert pulse.apply(21., base, stage='RR', phase='RR_HOLD')[4] == pytest.approx(200.)
    assert pulse.apply(22.25, base, stage='RR', phase='RR_HOLD')[4] == pytest.approx(100.)
    assert pulse.apply(22.5, base, stage='RR', phase='RR_HOLD') == base
    assert pulse.apply(30., base, stage='RR', phase='RR_HOLD') == base
    assert pulse.summary()['completed']


def test_rear_abort_snapshot_already_contains_the_force_bias():
    pulse = SupportForcePulse(SupportForcePulseConfig(amplitude_n=200.))
    base = (0.,) * 9
    pulse.apply(0., base, stage='RR', phase='RR_HOLD')
    actual = pulse.apply(1., base, stage='RR', phase='RR_HOLD')
    assert pulse.apply(1.02, actual, stage='RR', phase='RR_ABORT_STOP') == actual
    assert pulse.summary()['interrupted']


def test_front_abort_keeps_bias_until_smooth_lowering_release():
    pulse = SupportForcePulse(SupportForcePulseConfig(phase='THREE_WHEEL_HOLD',
        corner='RL', amplitude_n=-200.))
    base = (0.,) * 9
    pulse.apply(0., base, stage='FR', phase='THREE_WHEEL_HOLD')
    actual = pulse.apply(1., base, stage='FR', phase='THREE_WHEEL_HOLD')
    assert pulse.apply(1.02, base, stage='FR', phase='ABORT_STOP') == actual
    assert pulse.apply(2., base, stage='FR', phase='LOWERING') == actual
    half = pulse.apply(4., base, stage='FR', phase='LOWERING')
    assert half[6] == pytest.approx(-100.)
    assert pulse.apply(6., base, stage='FR', phase='RETURN_TO_FOUR_WHEEL') == base


def test_clipping_is_reported_and_actual_force_does_not_exceed_limit():
    pulse = SupportForcePulse(SupportForcePulseConfig(amplitude_n=200.))
    base = [0.] * 9
    base[4] = 18100.
    pulse.apply(0., base, stage='RR', phase='RR_HOLD')
    actual = pulse.apply(1., base, stage='RR', phase='RR_HOLD')
    assert actual[4] == 18200.
    assert pulse.summary()['clipped']
    assert pulse.telemetry['disturbance_applied_n'] == 100.


@pytest.mark.parametrize('kwargs', [{'amplitude_n': 0.}, {'amplitude_n': float('nan')},
    {'ramp_s': 0.}, {'start_delay_s': -.1}, {'phase':'RR_CRAWL'},
    {'phase':'RR_HOLD', 'corner':'RR'}])
def test_invalid_disturbance_config_rejected(kwargs):
    with pytest.raises(ValueError):
        SupportForcePulseConfig(**kwargs)

def test_full_loop_records_actual_force_and_recovery_snapshot_without_changing_qp_history():
    import sys
    from pathlib import Path
    from types import SimpleNamespace
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
    from run_right_side_full_cycle import FullRightSideController
    controller = object.__new__(FullRightSideController)
    controller.front = SimpleNamespace(mode='THREE_WHEEL_HOLD', rows=[])
    controller.rear = SimpleNamespace(mode='RR_HOLD', rows=[{'time_s': 0.}],
                                      last_suspension_force_n=np.zeros(4))
    controller.disturbance = SupportForcePulse(SupportForcePulseConfig(amplitude_n=200.))
    controller.allocation = SimpleNamespace(last_applied=np.zeros(4))
    base = (0.,) * 9
    controller._disturb(0., base, 'RR')
    controller.rear.rows.append({'time_s': 1.})
    actual = controller._disturb(1., base, 'RR')
    assert actual[4] == 200.
    assert controller.rear.rows[-1]['fact_fl_n'] == 200.
    assert controller.rear.rows[-1]['command_fact_fl_n'] == 0.
    assert controller.rear.last_suspension_force_n[0] == 200.
    assert controller.allocation.last_applied[0] == 0.

def test_skipped_entire_pulse_is_not_a_delivered_trial():
    pulse = SupportForcePulse(SupportForcePulseConfig(amplitude_n=200.))
    pulse.apply(0., (0.,) * 9, stage='RR', phase='RR_HOLD')
    pulse.apply(3., (0.,) * 9, stage='RR', phase='RR_HOLD')
    assert pulse.summary()['completed']
    assert not pulse.summary()['delivered']
