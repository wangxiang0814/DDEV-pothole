from pathlib import Path

import pytest

from ddevsim.scenarios.approach import approach_distance, select_expert
from ddevsim.scenarios.pit import load_case
from ddevsim.strategies.wheel_lift.approach import PitApproach


def observation(x, speed):
    return dict(X_R1=x, Vx=speed, Yo=0., Yaw=0.,
                **{f'AVy_{w}': 0. for w in ('L1', 'R1', 'L2', 'R2')},
                **{f'Fz_{w}': 3000. for w in ('L1', 'R1', 'L2', 'R2')})


def test_shared_start_and_scene_selection():
    assert approach_distance() == 12.
    assert approach_distance(10.) == 10.
    assert approach_distance(15.) == 15.
    with pytest.raises(ValueError):
        approach_distance(float('nan'))
    assert select_expert('right_track', .20, .8, .9).strategy == 'wheel_lift'
    assert select_expert('right_track', .04, .8, .9).strategy == 'shallow_press'
    with pytest.raises(ValueError):
        select_expert('wide', .20, .8, 1.8)


@pytest.mark.parametrize('gap', [10., 12., 15.])
def test_fixed_origin_keeps_road_and_friction_while_pit_moves(gap):
    root = Path(__file__).resolve().parents[1]
    scene = load_case(root / 'scenarios/shallow_press/right_track.json', depth_m=.04,
                      length_m=.8, width_m=.9, approach_distance_m=gap, fixed_origin=True)
    assert scene.initial_state['vehicle_start_station_m'] == 0.
    assert scene.geometry.leading_edge_m == gap
    assert scene.geometry.road_start_m == -20.
    assert scene.geometry.road_end_m == pytest.approx(65.)
    assert scene.geometry.friction == .70


def test_approach_brakes_and_waits_for_a_loaded_stationary_dwell():
    approach = PitApproach(101.1, 99.4)
    for tick in range(71):
        command = approach.step(tick * .02, observation(95.1, 0.))
    assert all(torque > 0 for torque in command[:4])
    assert command[4:8] == (0.,) * 4
    approach.step(1.42, observation(99.36, 1.))
    assert approach.phase == 'APPROACH_BRAKE'
    assert not approach.ready
    for tick in range(72, 108):
        sample = observation(99.42, 0.)
        sample['Fz_R1'] = 100.
        approach.step(tick * .02, sample)
    assert not approach.ready
    for tick in range(108, 154):
        approach.step(tick * .02, observation(99.42, 0.))
    assert approach.ready
    assert approach.command == (0.,) * 9


def test_overshoot_blocks_lift_handoff():
    approach = PitApproach(101.1, 99.4)
    approach.step(0., observation(95.1, 0.))
    approach.step(1., observation(99.8, .5))
    assert approach.phase == 'APPROACH_ABORT'
    assert not approach.ready
