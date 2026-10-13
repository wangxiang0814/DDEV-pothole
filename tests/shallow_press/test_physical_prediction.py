import numpy as np
import pytest
from ddevsim.strategies.shallow_press.physical_prediction import physical_rollout
from ddevsim.strategies.shallow_press.physical_prediction import physical_rollout_batch
from ddevsim.strategies.shallow_press.physical_prediction import preview_physical_road
from ddevsim.strategies.shallow_press.native_physical_probe import instrument_model
from ddevsim.strategies.shallow_press.native_physical_probe import validate_reference_matrices


def model():
    # One vertical mass: tyre stiffness 100 N/m, mass 2 kg.
    return dict(A=np.array([[0.,1.],[-50.,0.]]), B=np.array([[0.],[50.]]),
                C=np.array([[-100.,0.]]), D=np.array([[100.]]),
                f0=np.array([0.,-5.]), y0=np.array([10.]),
                load_rows=[0], height_cols=[0])


def test_affine_equilibrium_and_road_disturbance():
    m=model(); m['f0'][:]=0
    stationary=physical_rollout(m,[0,0],np.zeros((10,1)),.02)
    assert stationary[-1,0]==pytest.approx(10)
    moved=physical_rollout(m,[0,0],np.full((10,1),.02),.02)
    assert 10 < moved[0,0] < 12
    drift=physical_rollout(model(),[0,0],np.zeros((1,1)),.02)
    assert drift[0,0]>10


def test_unilateral_contact_changes_acceleration_not_just_output():
    m=model(); m['f0']=np.array([0.,0.])
    u=np.full((10,1),-.2)
    y=physical_rollout(m,[0,0],u,.02,unilateral=True)
    assert np.all(y>=0)
    # Free fall reaches -0.10 m in 0.2 s. Road is -0.2; still detached.
    assert y[-1,0]==pytest.approx(0,abs=1e-6)
    raw=physical_rollout(m,[0,0],u,.02)
    assert raw[0,0]<0


def test_malformed_model_and_nonfinite_rejected():
    m=model(); m['D']=np.zeros((2,1))
    with pytest.raises(ValueError): physical_rollout(m,[0,0],[[0]],.02)
    with pytest.raises(ValueError): physical_rollout(model(),[0,0],[[np.nan]],.02)


def test_instrumentation_is_scoped_and_preserves_original_commands():
    text='PARSFILE\nIMPORT IMP_FS_L1 Add 0.0\nEXPORT Vx\nEND\n'
    sim='SIMFILE\nPORTS_IMP 9\nPORTS_EXP 1\nEND\n'
    amended,ports,names=instrument_model(text,sim,['Vx'])
    assert 'IMPORT IMP_FS_L1 Add 0.0' in amended
    assert 'PORTS_IMP 21' in ports
    assert len(names)==12
    assert amended.index('LINEAR_CONTROL IMP_ZGND_L1I')<amended.rindex('END')
    assert 'IMPORT IMP_ZGND_L1I Add 0.0' in amended


def test_invalid_instrumentation_rejected():
    with pytest.raises(ValueError): instrument_model('PARSFILE\n','PORTS_IMP 9\n',['Vx'])


@pytest.mark.parametrize('unilateral',[False,True])
def test_batch_has_independent_initial_states_and_commands(unilateral):
    m=model(); m['f0'][:]=0
    x=np.array([[0,0],[.1,-.1]])
    u=np.array([np.zeros((5,1)),np.full((5,1),-.2)])
    batch=physical_rollout_batch(m,x,u,.02,unilateral=unilateral)
    for j in range(2):
        assert np.allclose(batch[j],physical_rollout(m,x[j],u[j],.02,unilateral=unilateral),atol=1e-9)


def test_preview_uses_current_pose_and_height_slope_order():
    scene=dict(start_station_m=1.,length_m=.8,width_m=1.8,depth_m=.04,edge_transition_m=.05,center_y_m=0.)
    xy=np.array([[.99,.63],[.99,-.63],[-.935,.63],[-.935,-.63]])
    road=preview_physical_road(xy,speed_mps=1.,yaw_rad=0,dt=.02,steps=3,scene=scene)
    assert road.shape==(3,12)
    assert road[0,0]==0
    assert road[1,0]==pytest.approx(-.008)
    assert road[1,4]==pytest.approx(-.8)
    assert np.all(road[:,8:]==0)


def test_current_affine_bias_is_instance_local():
    m=dict(A=np.zeros((1,1)),B=np.zeros((1,1)),C=np.ones((1,1)),D=np.zeros((1,1)),
        f0=np.array([[1.],[2.]]),y0=np.array([[10.],[20.]]))
    y=physical_rollout_batch(m,np.zeros((2,1)),np.zeros((2,2,1)),.1)
    assert np.allclose(y[:,:,0],[[10.1,10.2],[20.2,20.4]])


def test_current_free_fall_drift_does_not_get_contact_correction_twice():
    m=model(); m['y0']=np.array([-20.,0.]); m['contact_correction_origin']=np.array([20.])
    m['C']=np.array([[-100.,0.],[1.,0.]])
    m['D']=np.array([[100.],[0.]])
    y=physical_rollout(m,[0,0],np.zeros((10,1)),.02,unilateral=True)
    assert np.all(y[:,0]==0)  # 0.2 m gap; fall only 0.1 m during horizon.
    assert y[-1,1]==pytest.approx(-.1,abs=1e-8)


def test_clamped_force_also_corrects_reported_acceleration():
    m=model(); m['f0'][:]=0; m['y0']=np.array([10.,0.])
    m['C']=np.array([[-100.,0.],[-50.,0.]])
    m['D']=np.array([[100.],[50.]])
    y=physical_rollout(m,[0,0],np.full((3,1),-.2),.02,unilateral=True)
    assert np.all(y[:,0]==0)
    assert np.allclose(y[:,1],-5.)  # gravity; no tensile tyre force


def test_reference_matrices_cannot_diverge_from_native_file():
    from ddevsim.strategies.shallow_press.identification import parse_native_linearization
    text='\n'.join('Matrix'+k+'=zeros(1,1);' for k in 'ABCD')+'\nMatrixA(1,1)=-1;'
    parsed=parse_native_linearization(text)
    validate_reference_matrices(parsed,text)
    parsed['A'][0,0]=2
    with pytest.raises(ValueError,match='matrix'):
        validate_reference_matrices(parsed,text)


@pytest.mark.parametrize('value',[np.nan,np.inf,0.,-1.])
def test_nonfinite_or_nonpositive_time_rejected(value):
    with pytest.raises(ValueError): physical_rollout(model(),[0,0],[[0]],value)
    with pytest.raises(ValueError): physical_rollout(model(),[0,0],[[0]],.02,max_step_s=value)
