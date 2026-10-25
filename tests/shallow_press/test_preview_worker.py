import numpy as np
from ddevsim.strategies.shallow_press.native import NativePreviewBridge
from ddevsim.strategies.shallow_press.native import release_constant_brake
from ddevsim.strategies.shallow_press.controller import REQUIRED
from ddevsim.units import si_factor


class Recorder:
    def __init__(self): self.observations=[]
    def step(self,observation):
        from ddevsim.runtime.contracts import ControlCommand
        self.observations.append(observation)
        return ControlCommand((20,)*4,(100,)*4,.1)


def test_native_bridge_has_causal_si_observation_and_native_steering():
    s=Recorder(); bridge=NativePreviewBridge(s,REQUIRED)
    raw=np.array([1/si_factor(n) for n in REQUIRED])
    command=bridge(.0005,raw)
    assert s.observations[0].observed_at_s==0
    assert all(abs(v-1)<1e-8 for v in s.observations[0].values_si.values())
    assert command[-1]==.2  #400deg/s *0.5ms, toward0.1rad native target
    bridge(.001,raw)
    assert len(s.observations)==1


def test_delay_is_actual_observation_age_and_does_not_replay_commands():
    s=Recorder(); b=NativePreviewBridge(s,REQUIRED,delay_s=.04)
    raw=np.zeros(len(REQUIRED))
    for t in [.0005,.0205,.0405,.0605,.0805]: b(t,raw)
    assert abs(s.observations[-1].time_s-s.observations[-1].observed_at_s-.04)<1e-8


def test_assessment_is_json_serializable_and_empty_crossing_is_not_pass():
    import json
    from ddevsim.strategies.shallow_press.assessment import assess
    from test_preview_control import SCENE
    values={'time_s':np.arange(0,10,.001)}
    for name in REQUIRED: values['exp_'+name]=np.zeros(10000)
    result=assess(values,SCENE,dict(completed=True,failure_reasons=[]))
    json.dumps(result,allow_nan=False)
    assert not result['simulation_pass']


def test_fast_motor_loop_limits_rate_between_outer_updates():
    from ddevsim.strategies.shallow_press.config import ShallowPressConfig
    s=Recorder(); s.config=ShallowPressConfig()
    b=NativePreviewBridge(s,REQUIRED)
    raw=np.zeros(len(REQUIRED))
    for w in ('L1','R1','L2','R2'):
        raw[REQUIRED.index('Fz_'+w)]=3300
        raw[REQUIRED.index('CmpT_'+w+'i')]=15
        raw[REQUIRED.index('RRE_'+w+'i')]=263
    old=np.zeros(4)
    for t in np.arange(.0005,.02,.0005):
        u=np.array(b(float(t),raw)[:4])
        assert np.max(abs(u-old))<=.5+1e-9
        old=u
    assert len(s.observations)==1


def test_force_and_steering_are_slew_limited_at_native_step():
    s=Recorder(); b=NativePreviewBridge(s,REQUIRED)
    raw=np.zeros(len(REQUIRED)); old=np.zeros(5)
    for t in np.arange(.0005,.02,.0005):
        u=np.array(b(float(t),raw)[4:])
        assert np.max(abs(u[:4]-old[:4]))<=22.7+1e-8
        assert abs(u[4]-old[4])<=.2+1e-8
        old=u


def test_motor_slip_frame_does_not_substitute_countersteering_for_actual_yaw():
    from ddevsim.strategies.shallow_press.config import ShallowPressConfig
    s=Recorder(); s.config=ShallowPressConfig(); s.speed_reference=.3
    b=NativePreviewBridge(s,REQUIRED)
    raw=np.zeros(len(REQUIRED)); raw[REQUIRED.index('AVz')]=np.rad2deg(-.1)
    raw[REQUIRED.index('Vx')]=.3*3.6
    for i,w in enumerate(('L1','R1','L2','R2')):
        raw[REQUIRED.index('Y_'+w)]=.63 if i%2==0 else -.63
        raw[REQUIRED.index('Fz_'+w)]=3300
        raw[REQUIRED.index('CmpT_'+w+'i')]=15
        raw[REQUIRED.index('RRE_'+w+'i')]=263
        local=.3+.1*(.63 if i%2==0 else -.63)
        raw[REQUIRED.index('AVy_'+w)]=local/.263*30/np.pi
    for t in np.arange(.0005,.2,.0005):command=b(float(t),raw)
    # Freely rolling through the measured turn has zero longitudinal slip.
    # Yaw correction belongs to outer torque allocation and steering feedback.
    assert np.allclose(command[:4],20.)


def test_crawl_procedure_releases_brake_without_changing_hardware():
    text='IW 0.9\nENTER_PARSFILE Procedures\\test.par\nOPT_SC 0\nPBK_CON_CONSTANT 1\nEXIT_PARSFILE Procedures\\test.par\nEND\n'
    result=release_constant_brake(text)
    assert 'PBK_CON_CONSTANT 0' in result
    assert result.replace('PBK_CON_CONSTANT 0','PBK_CON_CONSTANT 1')==text


def test_contact_comparison_is_explicit_and_leaves_original_text_untouched():
    from ddevsim.strategies.shallow_press.native import contact_patch_comparison
    text=('L_CONTACT_X_CONSTANT 0\nL_CONTACT_Y_CONSTANT 0\n'*4)+'IW 0.9\n'
    assert contact_patch_comparison(text,0)==text
    changed=contact_patch_comparison(text,.15)
    assert changed.count('L_CONTACT_X_CONSTANT 150\n')==4  # native echo: mm
    assert changed.replace('L_CONTACT_X_CONSTANT 150','L_CONTACT_X_CONSTANT 0')==text
    import pytest
    with pytest.raises(ValueError):contact_patch_comparison(text,.7)


def test_low_speed_comparison_converts_si_to_native_kph():
    from ddevsim.strategies.shallow_press.native import low_speed_comparison
    text='VLOW_KAPPA 0.2\nVLOW_ALPHA 0.2\n'*4
    amended=low_speed_comparison(text,2/3.6)
    assert amended.count('VLOW_KAPPA 2\n')==4
    assert amended.count('VLOW_ALPHA 2\n')==4


def test_input_provenance_refreshes_generated_hashes_and_carries_comparison(tmp_path):
    from pathlib import Path
    import runpy,json
    from ddevsim.runtime.run_workspace import sha256_file
    freeze=runpy.run_path(str(Path(__file__).resolve().parents[2]/'scripts/run_shallow_preview.py'))['freeze_run_inputs']
    (tmp_path/'model').mkdir()
    (tmp_path/'model/run_all.par').write_text('amended physics')
    (tmp_path/'model/simfile.sim').write_text('amended ports')
    record=dict(generated_assets={'model/run_all.par':'old','model/simfile.sim':'old'},
        vehicle=dict(profile_id='parent',vehicle_sha256='parent-hash'),
        comparison_vehicle=dict(modification='external',adopted=False,ascii_stage='C:/Temp/run'),
        source_sha256={},effective_config={},diagnostic_options={},capability={'diagnostic':True})
    plant=freeze(tmp_path,record)
    assert json.loads((tmp_path/'input_provenance.json').read_text())['capability']=={'diagnostic':True}
    assert record['generated_assets']['model/run_all.par']==sha256_file(tmp_path/'model/run_all.par')
    assert record['prepared_assets']['model/run_all.par']=='old'
    assert not plant['same_vehicle_as_parent']
    assert not plant['comparison']['adopted']
    assert record['input_provenance_sha256']==sha256_file(tmp_path/'input_provenance.json')
