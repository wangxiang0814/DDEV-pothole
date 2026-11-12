import pytest

from ddevsim.strategies.wheel_lift.lift_scene import LiftScene, SideMirror


def test_left_observation_and_action_are_involutions():
    bridge = SideMirror(('Fz_L1', 'Fz_R1', 'Y_L1', 'Y_R1', 'Roll_E', 'AVy', 'AVz'))
    data = (12, 30, .6, -.7, 3, 4, 5)
    reflected = (30, 12, .7, -.6, -3, 4, -5)
    assert bridge.observation(data) == reflected
    assert bridge.observation(reflected) == data
    action = tuple(range(1, 10))
    assert bridge.action(action) == (2, 1, 4, 3, 6, 5, 8, 7, -9)
    assert bridge.action(bridge.action(action)) == action


def test_lift_scene_checks_offset_against_width():
    scene = LiftScene('left', 100, 50, 40, 5)
    assert scene.center_y_m == pytest.approx(.68)
    with pytest.raises(ValueError, match='offset'):
        LiftScene('left', 100, 50, 40, 6)
    with pytest.raises(ValueError, match='integer'):
        LiftScene('left', 100.5, 50, 40, 0)
