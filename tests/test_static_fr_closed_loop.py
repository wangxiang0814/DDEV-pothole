import numpy as np
import pytest

from ddevsim.static_wheel_lift.config import CLOSED_LOOP_RUN, speed_trial_config
from ddevsim.static_wheel_lift.closed_loop import (
    CrawlTorqueFeedback, PreloadProgress, ScalarLoadFeedback,
    SupportForceFeedback, blend_contact_gain,
    WheelLoadFilter,
)


def test_support_feedback_uses_measured_rl_load_and_respects_travel():
    gains = np.array([[-0.12, 0., 0.], [0.173, -0.015, 0.019],
                      [-0.054, 0.011, -0.019]])
    ctl = SupportForceFeedback(gains, control_period_s=0.02,
                               correction_limit_n=300.,
                               rebound_guard_mm=-95.)
    ref = np.array([6400., 1200., 5700.])
    measured = np.array([6400., 1000., 5700.])
    out = ctl.update(1., measured, ref, travel_mm=np.array([-80., 40., 20.]))
    assert out[0] > 0.  # FL push raises RL load in the measured swing gain.
    assert np.max(np.abs(out)) <= 300.
    guarded = ctl.update(1.02, measured, ref,
                         travel_mm=np.array([-96., 40., 20.]))
    assert guarded[0] <= out[0]


def test_support_feedback_changes_output_when_load_changes():
    gains = np.array([[-0.12, 0., 0.], [0.173, -0.015, 0.019],
                      [-0.054, 0.011, -0.019]])
    ctl = SupportForceFeedback(gains, control_period_s=0.02,
                               correction_limit_n=300.,
                               rebound_guard_mm=-95.)
    ref = np.array([6400., 1200., 5700.])
    a = ctl.update(1., np.array([6400., 1000., 5700.]), ref,
                   travel_mm=np.array([-80., 40., 20.]))
    for step in range(1, 6):
        b = ctl.update(1. + step * 0.02,
                       np.array([6400., 1600., 5700.]), ref,
                       travel_mm=np.array([-80., 40., 20.]))
    assert b[0] < a[0]


def test_crawl_torque_tracks_speed_and_corrects_right_drift():
    ctl = CrawlTorqueFeedback(control_period_s=0.02, tyre_radius_m=0.263,
                              half_track_m=0.625, friction=0.7)
    loads = np.array([6400., 1200., 5700.])
    out = ctl.update(0., vx_kph=0., yaw_deg=0., yaw_rate_deg_s=0.,
                     lateral_m=0., loads_n=loads, target_kph=0.7)
    assert all(t >= 0. for t in out)
    assert sum(out) > 0.
    for step in range(1, 15):
        right_drift = ctl.update(step * 0.02, vx_kph=0., yaw_deg=-1.,
                                 yaw_rate_deg_s=0., lateral_m=-0.05,
                                 loads_n=loads, target_kph=0.7)
    assert right_drift[2] > right_drift[0]
    assert right_drift[1] < 0.7 * loads[1] * 0.263


def test_crawl_torque_stops_when_target_is_zero():
    ctl = CrawlTorqueFeedback(control_period_s=0.02, tyre_radius_m=0.263,
                              half_track_m=0.625, friction=0.7)
    loads = np.array([6400., 1200., 5700.])
    out = ctl.update(0., vx_kph=0.8, yaw_deg=0., yaw_rate_deg_s=0.,
                     lateral_m=0., loads_n=loads, target_kph=0.)
    assert sum(out) < 0.


def test_crawl_integrator_does_not_wind_up_against_traction_cap():
    ctl = CrawlTorqueFeedback(control_period_s=0.02, tyre_radius_m=0.263,
                              half_track_m=0.625, friction=0.7)
    loads = np.array([100., 100., 100.])
    for i in range(100):
        out = ctl.update(i * 0.02, vx_kph=0., yaw_deg=0.,
                         yaw_rate_deg_s=0., lateral_m=0.,
                         loads_n=loads, target_kph=8.)
    assert ctl.integral_n == 0.
    assert np.max(out) <= 0.8 * 0.7 * 100. * 0.263 + 1e-8


def test_wheel_load_filter_rejects_single_sample_FR_drop():
    filt = WheelLoadFilter(cutoff_hz=5., wheel_count=4)
    initial = filt.update(0., [3000., 150., 3000., 3000.])
    assert initial[1] == 150.
    transient = filt.update(.02, [3000., 0., 3000., 3000.])
    assert 50. < transient[1] < 150.
    recovered = filt.update(.04, [3000., 150., 3000., 3000.])
    assert recovered[1] > transient[1]


def test_preload_progress_slows_or_pauses_on_tracking_error():
    clock = PreloadProgress(start_s=7., end_s=48., max_rate=1.5,
                            slow_error_n=200., pause_error_n=500.)
    assert clock.update(0., load_error_n=0.) == 0.
    assert clock.update(7., load_error_n=0.) == 7.
    assert clock.update(8., load_error_n=100.) == 8.5
    assert clock.update(9., load_error_n=300.) == 9.5
    assert clock.update(10., load_error_n=600.) == 9.5
    assert clock.update(40., load_error_n=0.) == 48.


def test_FR_load_feedback_uses_identified_sign_and_slew():
    ctl = ScalarLoadFeedback(gain_load_per_force=0.2, control_period_s=0.02,
                             correction_limit_n=300.,
                             slew_n_s=400., tracking_gain=0.1,
                             deadband_n=50.)
    assert ctl.update(.0, measured_n=200., reference_n=0.) == -8.
    assert ctl.update(.02, measured_n=200., reference_n=0.) == -16.
    for i in range(2, 100):
        ctl.update(.02 * i, measured_n=200., reference_n=0.)
    assert ctl.correction_n == -300.


def test_FR_feedback_resolves_small_error_near_lift_gate():
    ctl = ScalarLoadFeedback(gain_load_per_force=0.2, control_period_s=.02,
                             correction_limit_n=500., slew_n_s=400.,
                             tracking_gain=.1, deadband_n=10.)
    output = ctl.update(0., measured_n=104., reference_n=78.)
    assert output < 0.
    assert abs(output) <= 8.


def test_support_gain_blends_as_FR_contact_unloads():
    static = np.eye(3)
    swing = 2. * np.eye(3)
    assert np.allclose(blend_contact_gain(static, swing, 400.,
                                          swing_n=50., stance_n=300.), static)
    assert np.allclose(blend_contact_gain(static, swing, 50.,
                                          swing_n=50., stance_n=300.), swing)
    midpoint = blend_contact_gain(static, swing, 175.,
                                  swing_n=50., stance_n=300.)
    assert np.allclose(midpoint, 1.5 * np.eye(3))


def test_speed_trial_updates_target_and_acceptance_minimum_together():
    trial = speed_trial_config(3.3, 3.0)
    assert trial.crawl_speed_kph == 3.3
    assert trial.crawl_min_speed_kph == 3.0
    assert CLOSED_LOOP_RUN.crawl_speed_kph == 2.2
    with pytest.raises(ValueError):
        speed_trial_config(3.0, 3.5)
