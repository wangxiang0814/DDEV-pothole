import importlib.util
from pathlib import Path
import sys
import numpy as np
import pytest


def load(name):
    scripts = Path(__file__).resolve().parents[1] / 'scripts'
    if str(scripts) not in sys.path:
        sys.path.insert(0, str(scripts))
    spec = importlib.util.spec_from_file_location(name, scripts / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_native_step_override_preserves_model_and_one_ms_logging():
    m = load('run_chassis_contact_diagnostic')
    text = 'PARSFILE\ntstep 0.0005\nTSTOP 30\nEND\n'
    updated, decimation = m.configure_timestep(text, .000125)
    assert updated == 'PARSFILE\ntstep 0.000125\nTSTOP 30\nEND\n'
    assert decimation == 8
    with pytest.raises(ValueError):
        m.configure_timestep(text, .0003)
    with pytest.raises(ValueError):
        m.configure_timestep(text + 'tstep 0.001\n', .00025)


def test_force_impulse_and_contact_duration_follow_selected_native_step():
    m = load('run_chassis_contact_diagnostic')
    names = ['Vx','X_L2','X_R2','X_R1','Roll_E','Pitch','Yo','Yaw','AVz','X_RP1','Y_RP1','Z_RP1']
    values = [1.,9.,9.,9.,0.,0.,0.,0.,0.,9.,0.,-.01]
    scene = dict(start_station_m=10.,length_m=.8,width_m=1.8,center_y_m=0.,depth_m=.2,edge_transition_m=.05)
    config = dict(total_stiffness_n_m=100000.,total_damping_n_s_m=0.,friction=0.,slip_regularization_m_s=.05)
    c = m.ContactDiagnostic(names,np.array([[0.,0.,0.]]),scene,config,10.,True,0.,step_s=.00025)
    c(.00025, values); c(.0005, values)
    assert c.impulse[2] == pytest.approx(.5)
    assert c.contact_duration_s == pytest.approx(.0005)


def test_phase_windows_keep_axles_and_post_exit_braking_separate():
    m = load('analyze_passive_problem_study')
    times = np.arange(8.)
    front = np.array([8.,9.8,10.4,11.2,12.,13.2,14.,15.])
    rear = front - 2.
    phases, events = m.phase_windows(times, front, front, rear, rear, 10., 10.8)
    assert events['front_entry_s'] == 1.
    assert events['front_exit_s'] == 3.
    assert events['rear_entry_s'] == 4.
    assert events['rear_exit_s'] == 5.
    assert phases['front_axle'].tolist() == [False,True,True,False,False,False,False,False]
    assert phases['between_axles'][3]
    assert phases['rear_axle'][4]
    assert phases['post_exit'][5]
    _, clipped = m.phase_windows(times[:4],front[:4],front[:4],rear[:4],rear[:4],10.,10.8)
    assert clipped['rear_exit_s'] is None
