import numpy as np

from ddevsim.static_wheel_lift.feasibility import check_local_feasibility, check_preload_feasibility


TRIANGLE = np.array([[1., 1.], [-1., 1.], [-1., -1.]])


def test_local_feasibility_rejects_unreachable_safe_triangle():
    result = check_local_feasibility(
        TRIANGLE, np.array([0., 0.]),
        np.array([[0., 0., 0., 0.], [0.01, 0., 0., 0.]]),
        np.ones(4) * 10., np.zeros((4, 4)),
        np.zeros(4), np.zeros((4, 4)),
        np.zeros(2), np.zeros((2, 4)),
        force_bound_n=1., travel_bound_m=1., attitude_bound_rad=1.,
        support_floor_n=1., lambda_safe=0.05,
    )
    assert result.status == "STATIC_LIFT_INFEASIBLE"
    assert result.best_lambda_min < 0.05


def test_local_feasibility_accepts_reachable_safe_triangle():
    result = check_local_feasibility(
        TRIANGLE, np.array([-1/3, 1/3]),
        np.zeros((2, 4)), np.ones(4) * 10., np.zeros((4, 4)),
        np.zeros(4), np.zeros((4, 4)),
        np.zeros(2), np.zeros((2, 4)),
        force_bound_n=1., travel_bound_m=1., attitude_bound_rad=1.,
        support_floor_n=1., lambda_safe=0.05,
    )
    assert result.status == "LOCAL_MODEL_FEASIBLE"
    assert result.best_lambda_min == 1/3


def test_preload_gate_enforces_fr_half_load_and_asymmetric_travel():
    gain_fz = np.zeros((4, 4))
    gain_fz[1, 0] = 1.0
    gain_travel = np.zeros((4, 4))
    gain_travel[0, 0] = -0.2
    kwargs = dict(
        support_xy=TRIANGLE, com_xy=(-1/3, 1/3),
        g_com=np.zeros((2, 4)), zmp_xy=(-1/3, 1/3),
        g_zmp=np.zeros((2, 4)), fz_n=np.ones(4) * 10.,
        g_fz=gain_fz, travel_m=np.zeros(4), g_travel=gain_travel,
        attitude_rad=np.zeros(2), g_attitude=np.zeros((2, 4)),
        force_bound_n=10., travel_lower_m=np.ones(4) * -0.5,
        travel_upper_m=np.ones(4) * 0.5, attitude_bound_rad=1.,
        support_floor_n=1., fr_max_n=5., lambda_safe=0.05,
    )
    blocked = check_preload_feasibility(**kwargs)
    assert blocked.status == "STATIC_LIFT_INFEASIBLE"
    allowed = check_preload_feasibility(**{**kwargs, "travel_upper_m": np.ones(4) * 2.})
    assert allowed.status == "LOCAL_ENDPOINT_CANDIDATE"
    assert allowed.predicted_fz_n[1] <= 5. + 1e-8


def test_preload_gate_requires_both_com_and_zmp_inside():
    result = check_preload_feasibility(
        TRIANGLE, (-1/3, 1/3), np.zeros((2, 4)),
        (0., 0.), np.zeros((2, 4)),
        np.ones(4) * 10., np.zeros((4, 4)),
        np.zeros(4), np.zeros((4, 4)),
        np.zeros(2), np.zeros((2, 4)),
        force_bound_n=1., travel_lower_m=np.ones(4) * -1.,
        travel_upper_m=np.ones(4), attitude_bound_rad=1.,
        support_floor_n=1., fr_max_n=None, lambda_safe=0.05,
    )
    assert result.status == "STATIC_LIFT_INFEASIBLE"
    assert result.best_lambda_min < 0.05
