import pytest

from ddevsim.static_wheel_lift.path_reference import StaticPathReference


def test_reference_averages_static_samples_not_one_noisy_last_sample():
    estimator = StaticPathReference(window_s=1., period_s=.02, min_samples=30)
    for i in range(151):
        now = i * .02
        estimator.update(now, .001 if i % 2 else -.001, .02 if i % 2 else -.02)
    y, yaw = estimator.freeze()
    assert abs(y) < .0001
    assert abs(yaw) < .001
    assert estimator.freeze() == (y, yaw)


def test_native_substeps_do_not_duplicate_one_observation():
    estimator = StaticPathReference(window_s=1., period_s=.02, min_samples=30)
    for i in range(40):
        estimator.update(i * .0005, 0., 0.)
    assert estimator.sample_count == 1
    with pytest.raises(ValueError, match='samples'):
        estimator.freeze()


def test_reference_freeze_is_preserved_when_vehicle_moves_later():
    estimator = StaticPathReference(window_s=1., period_s=.02, min_samples=30)
    for i in range(51):
        estimator.update(i * .02, 2., 5.)
    assert estimator.freeze() == (2., 5.)
    estimator.update(3., 12., 20.)
    assert estimator.freeze() == (2., 5.)
