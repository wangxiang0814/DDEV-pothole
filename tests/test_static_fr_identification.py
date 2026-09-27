import numpy as np
import pytest

from ddevsim.static_wheel_lift.system_identification import (
    central_difference, damped_least_squares, diagnose_gain,
)


def test_central_difference_recovers_cross_corner_gain():
    positive = np.array([[2.0, 5.0], [3.0, 7.0]])
    negative = np.array([[0.0, 1.0], [1.0, 3.0]])
    np.testing.assert_allclose(central_difference(positive, negative, 1000.0),
                               [[0.001, 0.002], [0.001, 0.002]])


def test_damped_solver_handles_singular_gain_without_inverse():
    gain = np.array([[1.0, 1.0], [0.0, 0.0]])
    command = damped_least_squares(gain, np.array([2.0, 0.0]), ridge=0.1)
    assert np.isfinite(command).all()
    assert command[0] == pytest.approx(command[1])
    assert 0.0 < command[0] < 1.0


def test_gain_diagnostic_reports_effective_rank_with_physical_tolerance():
    result = diagnose_gain(np.diag([1., 0.05, 0.02, 1e-7]), relative_cutoff=1e-3)
    assert result["effective_rank"] == 3
    assert result["requires_damping"]
    assert result["condition_number"] > 1e6
