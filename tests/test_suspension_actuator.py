import pytest


def test_zero_request_keeps_passive_suspension_unchanged():
    from ddevsim.suspension_actuator import SuspensionActuator

    actuator = SuspensionActuator(-5000.0, 5000.0, 1000.0, 0.05)
    assert actuator.step(0.0, 0.01) == 0.0


def test_positive_and_negative_force_obey_slew_and_force_limit():
    from ddevsim.suspension_actuator import SuspensionActuator

    actuator = SuspensionActuator(-100.0, 100.0, 1000.0, 0.05)
    first = actuator.step(5000.0, 0.01)
    assert 0.0 < first <= 10.0
    for _ in range(100):
        actuator.step(5000.0, 0.01)
    assert actuator.actual_n == pytest.approx(100.0, abs=0.01)
    before = actuator.actual_n
    backwards = actuator.step(-5000.0, 0.01)
    assert 0.0 < before - backwards <= 10.00001


def test_invalid_actuator_time_or_limits_are_rejected():
    from ddevsim.suspension_actuator import SuspensionActuator

    with pytest.raises(ValueError):
        SuspensionActuator(100.0, -100.0, 1000.0, 0.05)
    with pytest.raises(ValueError):
        SuspensionActuator(-100.0, 100.0, 1000.0, 0.0)


def test_probe_plan_has_both_force_signs_at_each_corner():
    from ddevsim.suspension_actuator import signed_probe_cases

    cases = signed_probe_cases(1000.0)
    assert len(cases) == 8
    assert {(corner, force > 0) for corner, force in cases} == {
        (corner, positive) for corner in ("FL", "FR", "RL", "RR")
        for positive in (False, True)
    }
