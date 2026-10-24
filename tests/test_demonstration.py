import json
import numpy as np
import pytest

from ddevsim.demonstration.recording import TransitionRecorder, TransitionReader
from ddevsim.demonstration.cameras import validate_config, camera_par
from ddevsim.demonstration.shared_buffer import synchronized_frames


def test_observation_action_causality_and_units(tmp_path):
    rec = TransitionRecorder(tmp_path, ['IMP_MYUSM_L1', 'IMP_STEER_SW'],
                             ['Vx', 'Roll_E', 'XCG_TM'])
    rec.append(0., .01, [3.6, 0., 12.], [10., 90.])
    rec.append(.01, .02, [7.2, 10., 13.], [20., -90.])
    rec.finish(.02, [10.8, 20., 14.])
    data = TransitionReader(tmp_path)
    mid = data.sample(.005)
    assert mid['state_si']['Vx'] == pytest.approx(1.5)
    assert mid['action_si']['IMP_MYUSM_L1'] == 10.
    assert mid['action_si']['IMP_STEER_SW'] == pytest.approx(np.pi / 2)
    assert 'XCG_TM' not in mid['state_si']
    assert data.sample(.01)['action_si']['IMP_MYUSM_L1'] == 20.
    with pytest.raises(ValueError):
        data.sample(-.001)
    with pytest.raises(ValueError):
        data.sample(.021)


def test_recorder_does_not_accept_nonfinite_or_overwrite(tmp_path):
    rec = TransitionRecorder(tmp_path, ['IMP_MYUSM_L1'], ['Vx'])
    with pytest.raises(ValueError):
        rec.append(0, .01, [float('nan')], [1])
    rec.close()
    with pytest.raises(FileExistsError):
        TransitionRecorder(tmp_path, ['IMP_MYUSM_L1'], ['Vx'])


def test_camera_contract_body_mount_and_validation():
    cfg = json.load(open('configs/demonstration/five_camera.json', encoding='utf-8'))
    validate_config(cfg)
    text = camera_par(cfg, 'unit_test')
    assert 'SET_X_NAME Xo' in text and 'SET_ROLL_NAME Roll_E' in text
    assert 'Rot_R1' not in text
    assert text.count('BUFFER_NAME') == 5
    cfg['cameras'][1]['name'] = 'front'
    with pytest.raises(ValueError):
        validate_config(cfg)


def test_mixed_stale_missing_camera_groups_are_rejected():
    frames = [({'time_s': .1}, None)]*5
    assert synchronized_frames(frames)
    assert not synchronized_frames(frames, .1)
    assert not synchronized_frames(frames[:4])
    assert not synchronized_frames(frames[:4]+[None])
    assert not synchronized_frames(frames[:4]+[({'time_s': .11}, None)])


def test_native_boundary_tap_preserves_commands_and_pre_step_state(monkeypatch, tmp_path):
    from ddevsim import cosim
    submitted = []

    class Function:
        def __init__(self, fn):
            self.fn = fn
        def __call__(self, *args):
            return self.fn(*args)

    def configure(_name, ni, ne, start, stop, step):
        ni._obj.value = ne._obj.value = 1
        start._obj.value, stop._obj.value, step._obj.value = 0., .03, .01

    def initial(exports):
        exports[0] = 3.6

    def integrate(t, imports, exports):
        submitted.append((t.value, imports[0]))
        exports[0] += 3.6
        return 0

    class Solver:
        vs_set_opt_error_dialog = Function(lambda *_: None)
        vs_get_error_message = Function(lambda: b'')
        vs_read_configuration = Function(configure)
        vs_copy_export_vars = Function(initial)
        vs_integrate_io = Function(integrate)
        vs_terminate_run = Function(lambda *_: None)

    monkeypatch.setattr(cosim.ctypes.cdll, 'LoadLibrary', lambda _: Solver())
    monkeypatch.setattr(cosim, 'parse_simfile', lambda _: {'DLLFILE': 'fake.dll'})
    monkeypatch.setattr(cosim, 'solver_input_name', lambda _: b'fake.sim')
    command = lambda t, obs: (obs[0]*2,)
    cosim.run_stepwise(tmp_path/'fake.sim', command, tmp_path/'off.csv',
                      ['IMP_MYUSM_L1'], ['Vx'], log_decimation=1)
    baseline = submitted.copy()
    submitted.clear()
    cosim.run_stepwise(tmp_path/'fake.sim', command, tmp_path/'on.csv',
                      ['IMP_MYUSM_L1'], ['Vx'], log_decimation=1,
                      demonstration_dir=tmp_path/'demo')
    assert submitted == baseline
    assert (tmp_path/'off.csv').read_bytes() == (tmp_path/'on.csv').read_bytes()
    sample = TransitionReader(tmp_path/'demo').sample(0.)
    assert sample['state_si']['Vx'] == pytest.approx(1.)
    assert sample['action_si']['IMP_MYUSM_L1'] == pytest.approx(7.2)
    import ddevsim.demonstration.recording as recording
    class BrokenRecorder:
        def __init__(self, *_):
            pass
        def append(self, *_):
            raise OSError('disk unavailable')
        def close(self):
            raise OSError('disk still unavailable')
    monkeypatch.setattr(recording, 'TransitionRecorder', BrokenRecorder)
    submitted.clear()
    result = cosim.run_stepwise(tmp_path/'fake.sim', command, tmp_path/'broken.csv',
                               ['IMP_MYUSM_L1'], ['Vx'], log_decimation=1,
                               demonstration_dir=tmp_path/'broken')
    assert submitted == baseline
    assert result['demonstration_recording']['status'] == 'ERROR'


def test_unfinished_log_cannot_be_used_as_demonstration(tmp_path):
    rec = TransitionRecorder(tmp_path, ['IMP_MYUSM_L1'], ['Vx'])
    rec.append(0., .01, [1.], [2.])
    rec.close()
    with pytest.raises(ValueError, match='incomplete'):
        TransitionReader(tmp_path)


def test_surface_ray_uses_real_triangle_not_bounding_box():
    from ddevsim.demonstration.mounts import surface_coordinate
    triangles = np.array([[[0., 1., 0.], [1., 1., 0.], [0., 1., 1.]],
                          [[0., -1., 0.], [1., -1., 0.], [0., -1., 1.]]])
    assert surface_coordinate(triangles, [.2, 1.02, .3], 1, 1) == 1.
    assert surface_coordinate(triangles, [.2, -1.02, .3], 1, -1) == -1.
    with pytest.raises(ValueError, match='no body surface'):
        surface_coordinate(triangles, [.9, 1.02, .9], 1, 1)


def test_camera_poll_skips_pixel_copy_until_sample_due():
    from ddevsim.demonstration.shared_buffer import SharedBuffer
    buf = object.__new__(SharedBuffer)
    buf.handle = 1
    calls = []
    def function(index, *_):
        def invoke(*args):
            calls.append(index)
            if index == 24:
                return 0
            if index == 26:
                return 1
            if index == 29:
                args[1]._obj.number = 10
                args[1]._obj.time_s = .5
            if index in (21, 22, 23, 31):
                raise AssertionError('unnecessary RGB read')
        return invoke
    buf.fn = function
    assert buf.read(after_time=.5) is None
    assert buf.read(minimum_time=.6) is None
    assert calls.count(25) == 2  # Both early returns unlock the native buffer.


def test_mount_audit_rejects_floating_lens(tmp_path):
    from ddevsim.demonstration.mounts import verify_mounts
    (tmp_path/'body.obj').write_text('v 0 1 0\nv 1 1 0\nv 0 1 1\nf 1 2 3\n')
    history = tmp_path/'scene.par'
    history.write_text('add_obj body.obj\nset_offset_z 0.1\n')
    camera = dict(name='front', position_m=[.2, 1.02, .3], mount=dict(
        surface_asset='body.obj', asset_offset_m=[0, 0, .1], surface_axis=1,
        outward_sign=1, lens_standoff_m=.02, max_standoff_m=.025))
    report = verify_mounts({'cameras': [camera]}, history, tmp_path)
    assert report['front']['standoff_m'] == pytest.approx(.02)
    camera['position_m'][1] = 1.3
    with pytest.raises(ValueError, match='floating away'):
        verify_mounts({'cameras': [camera]}, history, tmp_path)


def test_shallow_capture_uses_its_own_identity_and_phase_log(tmp_path):
    from ddevsim.demonstration.capture import phase_rows,episode_identity
    (tmp_path/'run_manifest.json').write_text(json.dumps({'strategy_id':'shallow_press'}))
    (tmp_path/'control_diagnostics.json').write_text(json.dumps([
        {'time_s':3,'phase':'APPROACH'},{'time_s':7,'phase':'CROSSING'}]))
    assert phase_rows(tmp_path)==[(3.,'APPROACH'),(7.,'CROSSING')]
    plant={'parent_profile_id':'tuned_ii_20261005','comparison':{'adopted':False}}
    capability={'diagnostic':True,'actual_torque_rate_nm_s':10000}
    identity=episode_identity(tmp_path,{'scene':{'depth_m':.04},'status':'SIMULATION_PASS','plant':plant,'capability':capability})
    assert identity['capability']==capability
    assert identity['strategy_id']=='shallow_press'
    assert identity['scene']=={'depth_m':.04}
    assert identity['plant']==plant
