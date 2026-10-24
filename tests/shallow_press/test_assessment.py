import numpy as np
from ddevsim.strategies.shallow_press.assessment import input_screen


def test_numeric_native_log_loader_preserves_names_and_float_values(tmp_path):
    from ddevsim.strategies.shallow_press.assessment import read_native_log
    path=tmp_path/'native.csv'
    path.write_text('time_s,exp_Vx,imp_IMP_MYUSM_L1\n0.0005,1.23456789,-2.5e-6\n.0015,-.2,100\n')
    reference=np.genfromtxt(path,delimiter=',',names=True,encoding='utf-8')
    actual=read_native_log(path)
    assert actual.dtype.names==reference.dtype.names
    for name in actual.dtype.names:assert np.array_equal(actual[name],reference[name])


def test_faster_torque_capability_is_diagnostic_and_never_a_baseline_pass():
    from ddevsim.strategies.shallow_press.assessment import capability_diagnostic
    data={'time_s':np.array([0.,.001,.002])}
    for root in ('IMP_MYUSM_','IMP_FS_'):
        for w in ('L1','R1','L2','R2'):data['imp_'+root+w]=np.zeros(3)
    data['imp_IMP_STEER_SW']=np.zeros(3)
    data['imp_IMP_MYUSM_R1']=np.array([0.,2.,4.])
    original=dict(criteria=dict(input_amplitude=True,input_slew=False,contact_retention=True),simulation_pass=False)
    result=capability_diagnostic(original,data,10000.)
    assert not result['simulation_pass'] and not result['acceptance_pass']
    assert result['diagnostic_motion_pass']
    assert result['criteria']['input_slew'] is False
    assert result['status']=='CAPABILITY_DIAGNOSTIC_ONLY'


def test_input_screen_checks_slew_as_well_as_amplitude():
    data={'time_s':np.array([0.,.001,.002])}
    for root in ['IMP_MYUSM_','IMP_FS_']:
        for w in ['L1','R1','L2','R2']:data['imp_'+root+w]=np.zeros(3)
    data['imp_IMP_STEER_SW']=np.zeros(3)
    data['imp_IMP_MYUSM_R1']=np.array([0.,2.,2.])
    criteria,metrics=input_screen(data)
    assert criteria['input_amplitude']
    assert not criteria['input_slew']
    assert metrics['torque_rate_peak_nm_s']==2000
    data['imp_IMP_MYUSM_R1']=np.array([0.,.5,1.])
    assert all(input_screen(data)[0].values())


def test_assumed_clearance_is_geometry_only_and_uses_full_plane():
    from ddevsim.strategies.shallow_press.assessment import assumed_clearance
    from test_preview_control import SCENE
    data={'time_s':np.arange(0,5,.001)}
    for n in ['Xo','Yo','Zo','Yaw','Pitch','Roll_E']:data['exp_'+n]=np.zeros(5000)
    data['exp_Xo'][:]=99.;data['exp_Zo'][:]=.5;data['exp_Zo'][4000:]=.38
    result=assumed_clearance(data,SCENE)
    assert result['geometry_status']=='ASSUMED_NOT_MEASURED'
    assert result['sweep'][0]['min_gap_m']<0
    assert result['sweep'][1]['min_gap_m']>0
    assert result['physical_contact_force']=='NOT_MODELED'


def _complete_values():
    from ddevsim.strategies.shallow_press.controller import REQUIRED
    from ddevsim.units import si_factor
    from test_preview_control import SCENE
    t=np.arange(0,12,.001);x=99+np.clip(t-3,0,7)*.9
    data={'time_s':t}
    for n in REQUIRED:data['exp_'+n]=np.zeros(len(t))
    data['exp_Xo']=x;v=np.where((t>=3)&(t<10),.9,0)
    data['exp_Vx']=v*3.6
    for i,w in enumerate(('L1','R1','L2','R2')):
        data['exp_X_'+w]=x-(1.925 if i>1 else 0)
        data['exp_Y_'+w]=np.full(len(t),.63 if i%2==0 else -.63)
        data['exp_Jnc_'+w]=np.full(len(t),40.)
        data['exp_Fz_'+w]=np.full(len(t),3300.)
        data['exp_CmpT_'+w+'i']=np.full(len(t),15.)
        data['exp_RRE_'+w+'i']=np.full(len(t),263.)
        data['exp_AVy_'+w]=v/.263*30/np.pi
        for root in ('IMP_MYUSM_','IMP_FS_'):data['imp_'+root+w]=np.zeros(len(t))
    data['imp_IMP_STEER_SW']=np.zeros(len(t))
    return data


def test_permanently_lost_wheel_cannot_pass_research_screen():
    from ddevsim.strategies.shallow_press.assessment import assess
    from test_preview_control import SCENE
    data=_complete_values();t=data['time_s']
    data['exp_Fz_R1'][t>=3]=0;data['exp_CmpT_R1i'][t>=3]=0
    result=assess(data,SCENE,dict(completed=True,failure_reasons=[]))
    assert not result['simulation_pass']
    assert not result['criteria']['contact_retention']


def test_complete_crossing_at_wrong_speed_cannot_pass_speed_tracking():
    from ddevsim.strategies.shallow_press.assessment import assess
    from test_preview_control import SCENE
    result=assess(_complete_values(),SCENE,dict(completed=True,failure_reasons=[]),crossing_kph=1.)
    assert not result['criteria']['speed_tracking']
    assert result['metrics']['crossing_mean_kph']>3.


def test_exit_progress_catches_front_pause_even_when_rear_later_clears():
    from ddevsim.strategies.shallow_press.assessment import exit_progress_metrics
    from test_preview_control import SCENE
    t=np.arange(0,4,.001);data={'time_s':t,'exp_Vx':np.ones(len(t))}
    for i,w in enumerate(('L1','R1','L2','R2')):
        data['exp_X_'+w]=np.full(len(t),101.8 if i<2 else 103.)
        data['exp_Y_'+w]=np.full(len(t),.63 if i%2==0 else -.63)
    data['exp_Vx'][1000:2200]=0.
    metrics=exit_progress_metrics(data,SCENE,1.)
    assert metrics[0]['wheel']=='FR'
    assert metrics[0]['longest_pause_s']>1.19
    assert metrics[1]['observed'] is False
