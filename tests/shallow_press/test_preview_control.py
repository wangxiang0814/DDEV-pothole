import math
import numpy as np
import pytest
from ddevsim.strategies.shallow_press.preview import wheel_support_height
from ddevsim.strategies.shallow_press.preview import exit_relief
from ddevsim.strategies.shallow_press.traction import allocate_torque
from ddevsim.strategies.shallow_press.controller import ShallowPressStrategy
from ddevsim.runtime.contracts import StrategyContext,Observation

SCENE=dict(start_station_m=101.1,length_m=.8,width_m=.9,depth_m=.04,
           edge_transition_m=.05,center_y_m=-.63,friction=.7)


def observation(t,x=99.):
    d=dict(Xo=x,Yo=0.,Zo=.5,Vx=0.,Roll_E=0.,Pitch=0.,Yaw=0.,AVx=0.,AVy=0.,AVz=0.,Az_SM=0.)
    for i,w in enumerate(('L1','R1','L2','R2')):
        d.update({'X_'+w:x-(1.925 if i>1 else 0),'Y_'+w:.63 if i%2==0 else -.63,
          'Jnc_'+w:.04,'JncR_'+w:0.,'Fz_'+w:3300.,'CmpT_'+w+'i':.015,
          'AVy_'+w:0.,'RRE_'+w+'i':.25,'Fx_'+w:0.})
    return Observation(t,t,True,d)


def controller(**config):
    s=ShallowPressStrategy()
    s.reset(StrategyContext('shallow_press','tuned_ii_20261005','right_track',.02,
                           dict(scenario=SCENE,controller=config),{}))
    return s


def test_rolling_circle_bridge_and_right_track_only():
    p=np.array([[101.105,-.63],[101.5,-.63],[101.8,-.63],[101.5,.63]])
    r=wheel_support_height(p,SCENE)
    assert -.01<r[0]<=0
    assert r[1]==pytest.approx(-.04,abs=.001)
    assert -.04<r[2]<0
    assert r[3]==0


def test_traction_removes_lost_wheel_and_keeps_side_balance():
    t=allocate_torque(200,[3300,0,3300,3300],[0]*4,0,0,.7,[.25]*4)
    assert t[1]==0
    assert sum(t)==pytest.approx(200)
    assert t[0]+t[2]==pytest.approx(t[1]+t[3])
    assert np.isfinite(allocate_torque(100,[0]*4,[0]*4,0,0,.7,[.25]*4)).all()


def test_zero_start_state_is_instance_local_and_resettable():
    a=controller(); b=controller()
    for t in np.arange(0,4,.02): a.step(observation(float(t)))
    assert a.status().phase!='SETTLE'
    assert b.step(observation(0)).wheel_torque_nm==(0,)*4
    assert not b.status().completed


def test_command_all_channels_rate_limited_even_after_stale_feedback():
    s=controller(); old=np.zeros(9)
    caps=np.array([500]*4+[18200]*4+[math.radians(220)])
    rates=np.array([1000]*4+[45400]*4+[math.radians(400)])
    for t in np.arange(0,6,.02):
        c=s.step(observation(float(t)))
        u=np.r_[c.wheel_torque_nm,c.suspension_force_n,c.steering_wheel_angle_rad]
        assert np.all(abs(u)<=caps+1e-9)
        assert np.all(abs(u-old)<=rates*.02+1e-8)
        old=u
    c=s.step(Observation(6.,5.,False,{}))
    u=np.r_[c.wheel_torque_nm,c.suspension_force_n,c.steering_wheel_angle_rad]
    assert np.all(abs(u-old)<=rates*.02+1e-8)
    assert 'STALE' in s.summary()['failure_reasons']


def test_finish_requires_observed_forward_and_rear_events_then_stop_dwell():
    s=controller()
    for t in np.arange(0,3.2,.02): s.step(observation(float(t)))
    s.step(observation(3.2,103.1))
    assert not s.status().completed
    for t in np.arange(3.22,6,.02): s.step(observation(float(t),105.))
    assert s.status().completed
    assert s.summary()['wheel_crossed']==[False,True,False,True]


def test_missing_scene_and_invalid_config_rejected():
    with pytest.raises(ValueError): ShallowPressStrategy().reset(StrategyContext('shallow_press','vehicle','x',.02,{},{}))
    with pytest.raises(ValueError): controller(preview_s=float('nan'))
    with pytest.raises(ValueError): controller(settle_s=.5)
    with pytest.raises(ValueError): controller(control_period_s=.01)


def test_exit_relief_is_smooth_and_only_on_covered_wheels():
    p=np.array([[101.82,-.63],[101.5,-.63],[101.82,.63],[102.3,-.63]])
    f=exit_relief(p,SCENE,2000.)
    assert f[0]<-1000
    assert np.allclose(f[1:],0)


def test_grade_feedforward_brakes_descent_and_drives_ascent_with_coverage():
    from ddevsim.strategies.shallow_press.preview import grade_torque_feedforward
    points=np.array([[101.22,.63],[101.22,-.63],[101.80,.63],[101.80,-.63]])
    torque=grade_torque_feedforward(points,SCENE,np.ones(4)*3300,np.ones(4)*.263,1.)
    assert torque[0]==torque[2]==0
    assert torque[1]<0<torque[3]
    assert np.max(abs(torque))<3300*.263


def test_exit_traction_preview_does_not_add_entry_braking():
    s=controller(grade_torque_gain=1.)
    for t in np.arange(0,3.2,.02):s.step(observation(float(t)))
    s.step(observation(3.2,101.22))
    assert s.records[-1]['climb_torque_feedforward_nm']==[0.,0.,0.,0.]
    s.step(observation(3.22,101.80))
    applied=s.records[-1]['climb_torque_feedforward_nm']
    assert min(applied)>=0 and applied[1]>0


def test_exit_assist_does_not_push_vehicle_already_above_reference():
    s=controller(grade_torque_gain=1.,preview_s=0.)
    for t in np.arange(0,3.2,.02):s.step(observation(float(t)))
    o=observation(3.2,101.8);d=dict(o.values_si);d['Vx']=1.
    s.step(Observation(3.2,3.2,True,d))
    assert s.records[-1]['climb_torque_feedforward_nm']==[0.,0.,0.,0.]


def test_exit_assist_covers_contact_uncertainty_after_geometric_lip():
    s=controller(grade_torque_gain=1.)
    for t in np.arange(0,3.2,.02):s.step(observation(float(t)))
    s.step(observation(3.2,101.91))
    # A finite-radius geometric preview is already flat; tire contact/resistance
    # can persist through the exit. The bounded exit window must not disappear
    # precisely when the vehicle slows to a crawl at the lip.
    assert s.records[-1]['climb_torque_feedforward_nm'][1]>0


