import numpy as np

from ddevsim.static_wheel_lift.closed_loop import (
    CrawlTorqueFeedback, SupportForceFeedback,
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
