import numpy as np

from ddevsim.static_wheel_lift.online_identification import secant_gain_update


def test_secant_update_matches_measured_increment_without_inverse():
    gains = {"Fz_n": [[1., 0.], [0., 1.]]}
    baseline = {"Fz_n": [100., 100.]}
    observed = {"Fz_n": [112., 95.]}
    result = secant_gain_update(gains, baseline, observed, [10., 0.],
                                max_prediction_error={"Fz_n": 6.})
    assert result.status == "PASS"
    np.testing.assert_allclose(np.asarray(result.gains["Fz_n"]) @ [10., 0.],
                               [12., -5.])


def test_secant_update_rejects_bad_prediction():
    result = secant_gain_update({"Fz_n": [[1., 0.]]},
                                {"Fz_n": [0.]}, {"Fz_n": [100.]},
                                [1., 0.], max_prediction_error={"Fz_n": 20.})
    assert result.status == "REIDENTIFY_REQUIRED"
    assert result.gains is None
