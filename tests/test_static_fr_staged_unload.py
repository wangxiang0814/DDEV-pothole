import numpy as np

from ddevsim.static_wheel_lift.staged_unload import (
    IncrementalUnloadController, UnloadSample, WheelLoadLowPass,
)


def _sample(t, *, fr=3000., margin=0.06, roll=0.):
    return UnloadSample(
        time_s=t, fz_filtered_n=(3000., fr, 3000., 3000.),
        lambda_min=margin, travel_mm=(0., 0., 0., 0.),
        roll_deg=roll, pitch_deg=0., roll_rate_deg_s=0.,
        pitch_rate_deg_s=0., vx_kph=0., wheel_speed_rpm=(0.,) * 4,
    )


def _controller():
    return IncrementalUnloadController(
        target_force_n=(200., -100., 0., 0.), baseline_fr_n=3000.,
        fr_drop_target_n=150., earliest_start_s=1., settle_s=0.5,
        ramp_s=2., hold_s=1., recovery_s=1.,
        lambda_safe=0.05, support_floor_n=1000.,
        travel_lower_mm=-80., travel_upper_mm=140.,
        max_attitude_deg=3., max_rate_deg_s=0.2,
        max_vx_kph=0.036, max_wheel_rpm=1., fr_tolerance_n=20.,
    )


def test_staged_unload_requires_measured_dwell_then_quintic_and_hold():
    c = _controller()
    assert c.update(_sample(0.)).status == "WAIT_STABLE"
    assert c.update(_sample(1.)).status == "WAIT_STABLE"
    assert c.update(_sample(1.4)).status == "WAIT_STABLE"
    assert c.update(_sample(1.5)).status == "RAMPING"
    half = c.update(_sample(2.5))
    np.testing.assert_allclose(half.force_n, [100., -50., 0., 0.])
    assert c.update(_sample(3.5, fr=2840.)).status == "HOLDING"
    final = c.update(_sample(4.5, fr=2840.))
    assert final.status == "READY_REIDENTIFY"
    np.testing.assert_allclose(final.force_n, [200., -100., 0., 0.])


def test_staged_unload_aborts_and_recovers_smoothly_on_margin_loss():
    c = _controller()
    c.update(_sample(1.))
    c.update(_sample(1.5))
    c.update(_sample(2.5))
    aborted = c.update(_sample(2.6, margin=0.049))
    assert aborted.status == "ABORT_RECOVERY"
    np.testing.assert_allclose(aborted.force_n, [100., -50., 0., 0.])
    mid = c.update(_sample(3.1, margin=0.049))
    np.testing.assert_allclose(mid.force_n, [50., -25., 0., 0.])
    end = c.update(_sample(3.6, margin=0.049))
    assert end.status == "ABORTED"
    np.testing.assert_allclose(end.force_n, [0., 0., 0., 0.])


def test_staged_unload_does_not_start_on_time_alone():
    c = _controller()
    for t in (1., 2., 3.):
        assert c.update(_sample(t, margin=0.04)).status == "WAIT_STABLE"
    assert c.update(_sample(3.5)).status == "WAIT_STABLE"
    assert c.update(_sample(4.)).status == "RAMPING"


def test_wheel_load_filter_preserves_raw_measurement_and_timestamps():
    f = WheelLoadLowPass(cutoff_hz=10.)
    np.testing.assert_allclose(f.update(0., (100.,) * 4), (100.,) * 4)
    filtered = f.update(0.01, (200.,) * 4)
    assert all(100. < v < 200. for v in filtered)
    with np.testing.assert_raises(ValueError):
        f.update(0.01, (200.,) * 4)
