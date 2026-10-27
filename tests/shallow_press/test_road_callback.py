import ctypes
import numpy as np
import pytest
from test_preview_control import SCENE


def test_circle_plane_is_flat_on_both_flat_road_and_pit_floor():
    from ddevsim.strategies.shallow_press.road_callback import circle_planes
    p=np.array([[99.,-.63],[101.5,-.63],[101.5,.63]])
    plane=circle_planes(p,SCENE)
    assert np.allclose(plane[:,1:],0.,atol=1e-10)
    assert np.allclose(plane[:,0],[0.,-.04,0.],atol=1e-10)


def test_callback_evaluates_the_plane_at_actual_query_not_a_previous_contact_point():
    from ddevsim.strategies.shallow_press.road_callback import CallbackRoad,circle_planes
    from ddevsim.chassis_contact import road_surface
    wheels=('L1','R1','L2','R2');names=[axis+'_'+w for axis in ('X','Y') for w in wheels]
    road=CallbackRoad(names,SCENE,None,None,'circle')
    p=np.array([[101.2,.63],[101.2,-.63],[99.275,.63],[99.275,-.63]])
    road.update(1.,np.r_[p[:,0],p[:,1]])
    def original(y,x,inst,z,dy,dx,mu):
        z[0]=road_surface(np.array([[x,y]]),SCENE)[0][0];dy[0]=0.;dx[0]=0.;mu[0]=.7
    road.original=original
    plane=circle_planes(p,SCENE)[1]
    for delta in [(0.,0.),(.07,-.03),(-.04,.02)]:
        q=p[1]+delta
        values=[ctypes.pointer(ctypes.c_double()) for _ in range(5)]
        road.query(q[1],q[0],2,*values)
        z,dy,dx,muy,mux=[v[0] for v in values]
        assert z==pytest.approx(plane[0]+plane[1]*delta[0]+plane[2]*delta[1])
        assert (dx,dy)==pytest.approx(tuple(plane[1:]))
        assert muy==mux==.7
    assert road.query_count==3 and road.error is None


def test_native_callback_errors_are_recorded_and_return_finite_physical_fallback():
    from ddevsim.strategies.shallow_press.road_callback import CallbackRoad
    names=[axis+'_'+w for axis in ('X','Y') for w in ('L1','R1','L2','R2')]
    road=CallbackRoad(names,SCENE,None,None,'passthrough')
    def fail(*args):raise RuntimeError('road getter failed')
    road.original=fail;values=[ctypes.pointer(ctypes.c_double()) for _ in range(5)]
    road.query(-.63,101.5,2,*values)
    assert 'road getter failed' in road.error
    assert values[0][0]==pytest.approx(-.04) and values[3][0]==values[4][0]==.7


def test_callback_tire_identity_cannot_silently_switch_to_another_hub():
    from ddevsim.strategies.shallow_press.road_callback import CallbackRoad
    names=[axis+'_'+w for axis in ('X','Y') for w in ('L1','R1','L2','R2')]
    road=CallbackRoad(names,SCENE,None,None,'circle')
    p=np.array([[101.2,.63],[101.2,-.63],[99.275,.63],[99.275,-.63]])
    road.update(1.,np.r_[p[:,0],p[:,1]])
    def original(y,x,inst,z,dy,dx,mu):z[0]=dy[0]=dx[0]=0.;mu[0]=.7
    road.original=original
    for q in [p[1],p[0]]:
        values=[ctypes.pointer(ctypes.c_double()) for _ in range(5)]
        road.query(q[1],q[0],2,*values)
    assert road.unsupported_queries==1


def test_flat_shortcut_preserves_complete_circle_plane_values_at_boundaries():
    from ddevsim.strategies.shallow_press.road_callback import CallbackRoad,circle_planes
    names=[axis+'_'+w for axis in ('X','Y') for w in ('L1','R1','L2','R2')]
    road=CallbackRoad(names,SCENE,None,None,'circle');rng=np.random.RandomState(27)
    for _ in range(100):
        p=np.column_stack((rng.uniform(100.5,102.5,4),rng.uniform(-1.2,1.2,4)))
        road.update(1.,np.r_[p[:,0],p[:,1]])
        assert np.array_equal(road.planes,circle_planes(p,SCENE))


@pytest.mark.parametrize('from_config',[False,True])
def test_callback_cannot_mix_an_interface_test_with_a_torque_capability_scan(tmp_path,from_config):
    import json
    from pathlib import Path
    import subprocess
    import sys
    root=Path(__file__).resolve().parents[2]
    args=[sys.executable,str(root/'scripts/run_shallow_preview.py'),'--case','wide','--depth-m','.1',
        '--run-id','callback_dry_no_claim','--road-callback','circle','--dry-run']
    if from_config:
        path=tmp_path/'config.json';path.write_text(json.dumps(dict(diagnostic_torque_rate_nm_s=5000.)))
        args+=['--config',str(path)]
    else:args+=['--diagnostic-torque-rate-nm-s','5000']
    result=subprocess.run(args,cwd=str(root),capture_output=True)
    assert result.returncode!=0 and b'callback' in result.stderr.lower()
