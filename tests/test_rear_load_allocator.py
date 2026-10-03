import numpy as np

from ddevsim.static_wheel_lift.load_allocator import allocate_loads


def test_rear_swing_allocation_balances_front_front_left_support():
    xy = np.array([[1., 1.], [1., -1.], [-1., 1.], [-1., -1.]])
    result = allocate_loads(xy, 12000., [1/3, 1/3], np.full(4, 3000.),
                            lifted_corner='RR', lift_target_n=0.,
                            lower_n=[500., 500., 500., 0.],
                            upper_n=[10000., 10000., 10000., 0.])
    assert result.status == 'OPTIMAL'
    np.testing.assert_allclose(result.fz_ref_n, [4000., 4000., 4000., 0.], atol=1e-5)


def test_rear_target_outside_support_cannot_sacrifice_equilibrium():
    result = allocate_loads([[1., 1.], [1., -1.], [-1., 1.], [-1., -1.]],
                            12000., [-0.9, -0.9], np.full(4, 3000.),
                            lifted_corner='RR', lift_target_n=0.,
                            lower_n=[500., 500., 500., 0.], upper_n=[10000., 10000., 10000., 0.])
    assert result.status == 'INFEASIBLE'
