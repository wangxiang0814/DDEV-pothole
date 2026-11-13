import pytest

from ddevsim.strategies.wheel_lift.lift_scene import LiftScene, length_adjusted_crawl_timing


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("length_cm, brake, accel, feedback", [
    (50, .5, 2.5, 500.), (80, .5, 2.5, 500.), (90, .55, 2.3, 525.), (100, .60, 2.1, 550.),
])
def test_crawl_timing_uses_actual_length_and_is_independent_of_side(side, length_cm, brake, accel, feedback):
    scene = LiftScene(side, length_cm=length_cm)
    assert length_adjusted_crawl_timing(scene, .5, 2.5, 500.) == pytest.approx(
        (brake, accel, feedback))


def test_nondefault_timing_trials_are_preserved():
    assert length_adjusted_crawl_timing(LiftScene("right", length_cm=100), .4, 3., 600.) == (.4, 3., 600.)
