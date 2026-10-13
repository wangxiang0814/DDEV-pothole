import numpy as np
import pytest

from ddevsim.strategies.shallow_press.validation import ValidationExcitation, preview_road


NAMES = ['Vx','X_L1','X_R1','X_L2','X_R2','Yo','Yaw','AVz','Roll_E','Pitch']


def test_excitation_starts_at_rest_and_final_summed_commands_are_bounded():
    control = ValidationExcitation(NAMES,speed_kph=2,seed=8,excite=True,end_station_m=120)
    x = [0,98,98,96,96,0,0,0,0,0]
    prev = np.zeros(9)
    for k in range(2000):
        cmd = np.asarray(control(k*.02+.0005,x))
        assert np.max(abs(cmd[:4])) <= 500
        assert np.max(abs(cmd[4:8])) <= 18200
        assert abs(cmd[8]) <= 220
        assert np.max(abs(cmd[:4]-prev[:4])) <= 20+1e-8
        assert np.max(abs(cmd[4:8]-prev[4:8])) <= 908+1e-8
        if k*.02 < 5:
            assert np.max(abs(cmd)) == 0
        prev = cmd
    assert max(np.ptp(np.asarray(control.command_history)[:,4:8],axis=0)) > 100


def test_callback_records_state_before_new_interval_command_and_holds_until_tick():
    c = ValidationExcitation(NAMES,speed_kph=2,seed=1,excite=False,end_station_m=120)
    x = [0,98,98,96,96,0,0,0,0,0]
    first = c(.0005,x)
    assert c.records[0]['observed_time_s'] == 0
    assert c(.001,x) == first
    assert len(c.records) == 1
    c(.0205,x)
    assert len(c.records) == 2
    assert c.records[1]['observed_time_s'] == pytest.approx(.02)


def test_excess_attitude_does_not_truncate_diagnostic():
    c = ValidationExcitation(NAMES,speed_kph=1,seed=1,excite=False,end_station_m=120)
    c(8,[1,101,101,99,99,0,0,0,30,30])
    assert c.attitude_exceeded
    assert not c.completed


def test_spatial_preview_uses_start_pose_and_speed_only_not_future_native_ground():
    scene = dict(start_station_m=100,length_m=.8,width_m=1.8,depth_m=.04,
                 center_y_m=0,edge_transition_m=.05)
    result = preview_road(np.array([[99.9,.63],[99.9,-.63],[98,.63],[98,-.63]]),
                          speed_mps=.5,dt=.02,steps=30,scene=scene)
    assert result.shape == (31,4)
    assert result[0].tolist() == [0,0,0,0]
    assert result[20,0] == pytest.approx(-.04)
    assert np.all(result[:,2:] == 0)


def test_excitation_and_integrators_are_instance_local():
    a = ValidationExcitation(NAMES,speed_kph=2,seed=7,excite=True,end_station_m=120)
    b = ValidationExcitation(NAMES,speed_kph=2,seed=7,excite=True,end_station_m=120)
    x=[0,98,98,96,96,0,0,0,0,0]
    a(8,x)
    assert b.records == []
    assert b.integral == 0
