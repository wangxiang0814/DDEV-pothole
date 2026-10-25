import math
import numpy as np
from ddevsim.strategies.shallow_press.config import ShallowPressConfig


def feedback():
    d=dict(Vx=.3,AVz=0.,Yaw=0.,Yo=0.)
    for i,w in enumerate(('L1','R1','L2','R2')):
        d.update({'RRE_'+w+'i':.263,'AVy_'+w:.3/.263,'Y_'+w:.63 if i%2==0 else -.63,
                  'Fz_'+w:3300,'CmpT_'+w+'i':.015})
    return d


def test_motor_sample_period_does_not_change_with_integrator_step():
    from ddevsim.strategies.shallow_press.actuation import CrawlActuation
    counts=[]
    for dt in (.0005,.000125):
        a=CrawlActuation(ShallowPressConfig());old=np.zeros(9)
        for t in np.arange(0,.02,dt):
            u=a.step(float(t),float(t),feedback(),np.r_[[20]*4,[100]*4,.1],None,{'friction':.7},dt)
            assert np.max(abs(u[:4]-old[:4]))<=1000*dt+1e-8
            assert np.max(abs(u[4:8]-old[4:8]))<=45400*dt+1e-8
            assert abs(u[8]-old[8])<=math.radians(400)*dt+1e-8
            old=u
        counts.append(a.motor_ticks)
    assert counts==[10,10]


def test_motor_does_not_reaccelerate_on_stale_feedback():
    from ddevsim.strategies.shallow_press.actuation import CrawlActuation
    a=CrawlActuation(ShallowPressConfig())
    for t in np.arange(0,.3,.0005):
        u=a.step(float(t),0.,feedback(),np.ones(9)*100,None,{'friction':.7},.0005)
    assert np.allclose(u,0)


def test_slip_feedback_does_not_duplicate_body_speed_controller_braking():
    from ddevsim.strategies.shallow_press.actuation import CrawlActuation
    a=CrawlActuation(ShallowPressConfig());d=feedback()
    d['Vx']=.7
    for w in ('L1','R1','L2','R2'):d['AVy_'+w]=.7/.263
    # Freely rolling above planned crawl speed: body PI owns deceleration.
    # A slip loop must not apply a second speed error to the rolling wheels.
    for t in np.arange(0,.2,.0005):u=a.step(float(t),float(t),d,np.zeros(9),None,{'friction':.7},.0005)
    assert np.allclose(u[:4],0)


def test_recontact_drive_ramp_does_not_disable_corrective_slip_braking():
    from ddevsim.strategies.shallow_press.actuation import CrawlActuation
    a=CrawlActuation(ShallowPressConfig());d=feedback()
    for w in ('L1','R1','L2','R2'):d['AVy_'+w]=1.3/.263
    # Newly regained load: nominal drive must ramp, but a spinning wheel still
    # needs bounded corrective braking during that ramp, not 120ms of waiting.
    u=a.step(0.,0.,d,np.r_[[100]*4,[0]*5],None,{'friction':.7},.0005)
    assert np.all(u[:4]<0)
    assert np.max(abs(u[:4]))<=.5


def test_slip_acceleration_feedback_reacts_before_large_speed_error():
    from ddevsim.strategies.shallow_press.actuation import CrawlActuation
    outputs=[]
    for gain in (0.,5.):
        a=CrawlActuation(ShallowPressConfig(motor_slip_derivative=gain))
        for t in np.arange(0,.101,.0005):
            d=feedback()
            for w in ('L1','R1','L2','R2'):d['AVy_'+w]=(.3+3*t)/.263
            u=a.step(float(t),float(t),d,np.r_[[100]*4,[0]*5],None,{'friction':.7},.0005)
        outputs.append(u[:4])
    assert np.all(outputs[1]<outputs[0]-5.)


def test_capability_scan_keeps_baseline_and_only_changes_explicit_torque_slew():
    import pytest
    with pytest.raises(ValueError):ShallowPressConfig(torque_rate_nm_s=10000)
    c=ShallowPressConfig(diagnostic_torque_rate_nm_s=10000)
    assert c.torque_rate_nm_s==1000
    assert c.actuation_torque_rate_nm_s==10000
    assert c.force_rate_n_s==45400
    assert ShallowPressConfig().actuation_torque_rate_nm_s==1000


def test_slip_measurement_uses_actual_yaw_rate_not_corrective_yaw_demand():
    from ddevsim.strategies.shallow_press.actuation import CrawlActuation
    a=CrawlActuation(ShallowPressConfig());d=feedback();d['AVz']=.1
    for w in ('L1','R1','L2','R2'):
        d['AVy_'+w]=(d['Vx']-d['AVz']*(d['Y_'+w]-d['Yo']))/.263
    for t in np.arange(0,.2,.0005):
        u=a.step(float(t),float(t),d,np.zeros(9),None,{'friction':.7},.0005)
    assert np.allclose(u[:4],0.,atol=1e-10)
