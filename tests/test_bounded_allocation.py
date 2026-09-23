import math

from ddevsim.bounded_allocation import bounded_weighted_least_squares


def test_identity_problem_obeys_active_box_bounds():
    result = bounded_weighted_least_squares(
        [[1.0, 0.0], [0.0, 1.0]],
        [2.0, -3.0],
        lower=[0.0, -2.0],
        upper=[1.0, 2.0],
        effort_weight=0.0,
    )
    assert result == [1.0, -2.0]


def test_coupled_problem_recovers_hand_solved_unconstrained_solution():
    result = bounded_weighted_least_squares(
        [[1.0, 1.0], [1.0, -1.0]],
        [2.0, 0.0],
        lower=[-10.0, -10.0],
        upper=[10.0, 10.0],
        effort_weight=0.0,
    )
    assert result == [1.0, 1.0]


def test_infeasible_target_returns_finite_best_bounded_command():
    result = bounded_weighted_least_squares(
        [[1.0, 1.0]], [100.0], lower=[-2.0, -3.0], upper=[2.0, 3.0]
    )
    assert result == [2.0, 3.0]
    assert all(math.isfinite(value) for value in result)

