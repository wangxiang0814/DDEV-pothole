import math

import pytest

from ddevsim.static_wheel_lift.support_geometry import assess_support


CONTACTS = {
    "FL": (1.0, 1.0), "FR": (1.0, -1.0),
    "RL": (-1.0, 1.0), "RR": (-1.0, -1.0),
}


def test_zmp_and_safe_projection_use_native_contact_points():
    loads = {"FL": 10.0, "FR": 10.0, "RL": 10.0, "RR": 10.0}
    result = assess_support(CONTACTS, loads, (0.0, 0.0), 0.05)
    assert result.zmp_xy == pytest.approx((0.0, 0.0))
    assert result.lambda_min == pytest.approx(0.0)
    assert not result.safe_inside
    assert result.target_zmp_xy == pytest.approx((-0.05, 0.05))
    assert result.target_distance_m == pytest.approx(math.sqrt(0.005))
    assert result.target_lambda_min >= 0.05 - 1e-12


def test_inside_point_is_unchanged_and_bad_load_is_rejected():
    loads = {"FL": 10.0, "FR": 0.0, "RL": 10.0, "RR": 10.0}
    result = assess_support(CONTACTS, loads, (-1/3, 1/3), 0.05)
    assert result.safe_inside
    assert result.target_zmp_xy == pytest.approx(result.zmp_xy)
    assert result.edge_distance_m == pytest.approx(math.sqrt(2) / 3)
    with pytest.raises(ValueError):
        assess_support(CONTACTS, {**loads, "FR": -1.0}, (0, 0), 0.05)


def test_rr_lift_uses_fl_fr_rl_triangle():
    loads = {"FL": 10.0, "FR": 10.0, "RL": 10.0, "RR": 10.0}
    result = assess_support(CONTACTS, loads, (0.0, 0.0), 0.05,
                            lifted_corner="RR")
    assert result.support_corners == ("FL", "FR", "RL")
    assert result.zmp_xy == pytest.approx((0.0, 0.0))
    assert result.lambda_min == pytest.approx(0.0)
    assert result.target_zmp_xy == pytest.approx((0.05, 0.05))
