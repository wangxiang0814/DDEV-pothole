import numpy as np

from ddevsim.static_wheel_lift.load_allocator import allocate_loads
from ddevsim.static_wheel_lift.quintic_trajectory import quintic_step
from ddevsim.static_wheel_lift.state_machine import LiftFSM, LiftSignals, LiftState


CONTACTS = np.array([[1., 1.], [1., -1.], [-1., 1.], [-1., -1.]])


def test_quintic_has_zero_endpoint_velocity_and_acceleration():
    for t, expected in ((0., 0.), (2., 1.)):
        p, v, a = quintic_step(t, 0., 2.)
        assert (p, v, a) == (expected, 0., 0.)
    p, v, a = quintic_step(1., 0., 2.)
    assert p == 0.5 and v > 0 and abs(a) < 1e-12


def test_load_allocator_respects_total_moments_and_support_floor():
    target = np.array([-0.2, 0.2])
    out = allocate_loads(CONTACTS, 100., target, np.ones(4) * 25.,
                         fr_target_n=0., lower_n=np.array([5., 0., 5., 5.]),
                         upper_n=np.array([90., 0., 90., 90.]))
    assert out.status == "OPTIMAL"
    np.testing.assert_allclose(out.fz_ref_n.sum(), 100., atol=1e-6)
    np.testing.assert_allclose(out.fz_ref_n @ CONTACTS / 100., target, atol=1e-6)
    assert np.all(out.fz_ref_n[[0, 2, 3]] >= 5. - 1e-6)
    assert out.fz_ref_n[1] <= 1e-5


def test_load_allocator_reports_infeasible_without_relaxing_stability():
    out = allocate_loads(CONTACTS, 100., np.array([0.9, -0.9]),
                         np.ones(4) * 25., fr_target_n=0.,
                         lower_n=np.array([20., 0., 20., 20.]),
                         upper_n=np.ones(4) * 30.)
    assert out.status == "INFEASIBLE"
    assert out.fz_ref_n is None


def test_fsm_requires_measured_dwell_and_feasibility_to_start_lift():
    fsm = LiftFSM(settle_s=0.1, ready_s=0.2, hold_s=5.)
    signal = LiftSignals(stationary=True, four_wheel_loaded=True,
                         feasible=False, safe_triangle=True, low_fr_load=True,
                         support_loaded=True, attitude_safe=True,
                         actuator_safe=True, lift_height_reached=False,
                         lift_clearance_confirmed=False, four_wheel_recovered=False)
    for i in range(20):
        fsm.update(i * 0.05, signal)
    assert fsm.state == LiftState.INIT_SETTLE
    signal = LiftSignals(**{**vars(signal), "feasible": True})
    fsm.update(1.0, signal)
    fsm.update(1.11, signal)
    assert fsm.state == LiftState.PRELOAD_SHIFT
    fsm.update(1.12, signal)
    assert fsm.state == LiftState.LIFT_READY_HOLD
    fsm.update(1.13, signal)
    fsm.update(1.34, signal)
    assert fsm.state == LiftState.LIFTING


def test_fsm_aborts_on_unsafe_support():
    fsm = LiftFSM(settle_s=0.1, ready_s=0.2, hold_s=5.)
    signal = LiftSignals(stationary=True, four_wheel_loaded=True,
                         feasible=True, safe_triangle=True, low_fr_load=True,
                         support_loaded=True, attitude_safe=True,
                         actuator_safe=True, lift_height_reached=False,
                         lift_clearance_confirmed=False, four_wheel_recovered=False)
    for t in (0., 0.11, 0.12, 0.13, 0.34):
        fsm.update(t, signal)
    assert fsm.state == LiftState.LIFTING
    unsafe = LiftSignals(**{**vars(signal), "support_loaded": False})
    fsm.update(0.35, unsafe)
    assert fsm.state == LiftState.ABORT_RECOVERY
    assert fsm.recovery_required


def test_fsm_requires_confirmed_clearance_and_five_second_hold():
    fsm = LiftFSM(settle_s=0.1, ready_s=0.2, hold_s=5.)
    base = LiftSignals(stationary=True, four_wheel_loaded=True,
                       feasible=True, safe_triangle=True, low_fr_load=True,
                       support_loaded=True, attitude_safe=True,
                       actuator_safe=True, lift_height_reached=True,
                       lift_clearance_confirmed=False, four_wheel_recovered=False)
    for t in (0., 0.11, 0.12, 0.13, 0.34, 1.0):
        fsm.update(t, base)
    assert fsm.state == LiftState.LIFTING
    confirmed = LiftSignals(**{**vars(base), "lift_clearance_confirmed": True})
    fsm.update(1.01, confirmed)
    assert fsm.state == LiftState.THREE_WHEEL_HOLD
    fsm.update(6.0, confirmed)
    assert fsm.state == LiftState.THREE_WHEEL_HOLD
    fsm.update(6.02, confirmed)
    assert fsm.state == LiftState.LOWERING


def test_fsm_rejects_nonmonotonic_simulation_time():
    import pytest

    fsm = LiftFSM(settle_s=0.1, ready_s=0.2, hold_s=5.)
    s = LiftSignals(False, False, False, False, False, False, False,
                    False, False, False, False)
    fsm.update(0.0, s)
    with pytest.raises(ValueError, match="timestamps"):
        fsm.update(0.0, s)


def test_fsm_aborts_if_feasibility_is_lost_during_lift():
    fsm = LiftFSM(settle_s=0.1, ready_s=0.2, hold_s=5.)
    s = LiftSignals(True, True, True, True, True, True, True,
                    True, False, False, False)
    for t in (0., 0.11, 0.12, 0.13, 0.34):
        fsm.update(t, s)
    assert fsm.state == LiftState.LIFTING
    fsm.update(0.35, LiftSignals(**{**vars(s), "feasible": False}))
    assert fsm.state == LiftState.ABORT_RECOVERY


def test_abort_recovery_requires_stable_four_wheel_contact():
    fsm = LiftFSM(settle_s=0.1, ready_s=0.2, hold_s=5.)
    s = LiftSignals(True, True, True, True, True, True, True,
                    True, False, False, False)
    for t in (0., 0.11, 0.12, 0.13, 0.34):
        fsm.update(t, s)
    fsm.update(0.35, LiftSignals(**{**vars(s), "support_loaded": False}))
    fsm.update(0.36, LiftSignals(**{**vars(s), "support_loaded": False,
                                  "four_wheel_recovered": True}))
    assert fsm.state == LiftState.ABORT_RECOVERY
