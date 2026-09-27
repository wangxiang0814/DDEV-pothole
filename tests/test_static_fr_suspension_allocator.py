import numpy as np

from ddevsim.static_wheel_lift.suspension_allocator import plan_local_unload_step


def _case():
    support = np.array([[-1., 1.], [-1., -1.], [1., -1.]])
    point = np.array([-0.2, -0.6])
    gain_fz = np.eye(4)
    zeros_xy = np.zeros((2, 4))
    zeros_four = np.zeros((4, 4))
    zeros_att = np.zeros((2, 4))
    return dict(
        support_xy=support, com_xy=point, g_com=zeros_xy,
        zmp_xy=point, g_zmp=zeros_xy, fz_n=np.full(4, 3000.),
        g_fz=gain_fz, travel_m=np.zeros(4), g_travel=zeros_four,
        attitude_rad=np.zeros(2), g_attitude=zeros_att,
        force_bound_n=500., travel_lower_m=np.full(4, -0.08),
        travel_upper_m=np.full(4, 0.14), attitude_bound_rad=np.deg2rad(3.),
        support_floor_n=1000., lambda_safe=0.05, margin_reserve=0.005,
        fr_drop_target_n=150.,
    )


def test_local_step_reduces_fr_with_hard_stability_and_minimum_effort():
    result = plan_local_unload_step(**_case())
    assert result.status == "OPTIMAL"
    assert result.predicted_fz_n[1] <= 2850.01
    np.testing.assert_allclose(result.force_n, [0., -150., 0., 0.], atol=0.01)
    assert result.predicted_lambda_min >= 0.055 - 1e-9


def test_local_step_rejects_unreachable_safe_triangle():
    case = _case()
    case["com_xy"] = np.array([0.5, 0.5])
    result = plan_local_unload_step(**case)
    assert result.status == "INFEASIBLE"
    assert result.force_n is None


def test_local_step_rejects_fr_target_that_breaks_attitude_limit():
    case = _case()
    case["g_attitude"] = np.zeros((2, 4))
    case["g_attitude"][0, 1] = 0.001
    result = plan_local_unload_step(**case)
    assert result.status == "INFEASIBLE"


def test_local_step_respects_corner_specific_trust_bounds():
    case = _case()
    case["g_fz"] = np.zeros((4, 4))
    case["g_fz"][1, 0] = -1.
    case["force_bound_n"] = (100., 500., 500., 500.)
    assert plan_local_unload_step(**case).status == "INFEASIBLE"
    case["force_bound_n"] = (200., 500., 500., 500.)
    result = plan_local_unload_step(**case)
    assert result.status == "OPTIMAL"
    assert 149.9 <= result.force_n[0] <= 150.1


def test_local_step_requires_margin_gain_even_when_basic_safe_gate_is_met():
    case = _case()
    case["g_zmp"] = np.zeros((2, 4))
    case["g_zmp"][1, 0] = -0.002
    case["stability_margin_target"] = 0.055
    assert plan_local_unload_step(**case).status == "OPTIMAL"
    case["stability_margin_target"] = 0.25
    assert plan_local_unload_step(**case).status == "INFEASIBLE"
