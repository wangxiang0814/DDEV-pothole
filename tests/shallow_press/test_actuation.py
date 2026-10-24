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
