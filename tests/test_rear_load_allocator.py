import numpy as np
import pytest
import ddevsim.static_wheel_lift.load_allocator as module

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


@pytest.mark.parametrize('corner', ['FR', 'RR'])
def test_fixed_lift_load_has_unique_solution_without_iterative_solvers(monkeypatch, corner):
    def forbidden(*args, **kwargs):
        raise AssertionError('unique three-support balance needs no iterative solver')
    monkeypatch.setattr(module, 'linprog', forbidden)
    monkeypatch.setattr(module, 'minimize', forbidden)
    xy = np.array([[101., 51.], [101., 49.], [99., 51.], [99., 49.]])
    idx = ['FL', 'FR', 'RL', 'RR'].index(corner)
    expected = np.full(4, 4000.)
    expected[idx] = 100.
    total = expected.sum()
    low, high = np.full(4, 500.), np.full(4, 10000.)
    low[idx] = high[idx] = 100.
    out = allocate_loads(xy, total, expected @ xy / total, np.full(4, 3000.),
                         lifted_corner=corner, lift_target_n=100., lower_n=low, upper_n=high)
    assert out.status == 'OPTIMAL'
    np.testing.assert_allclose(out.fz_ref_n, expected, atol=1e-7)
    assert np.isfinite(out.cost)
