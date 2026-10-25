import numpy as np


def _feedback(t,vertical_speed=0.,spring=1000.,damper=0.):
    from test_actuation import feedback
    d=feedback()
    for w in ('L1','R1','L2','R2'):
        d['X_'+w]=d['Vx']*t;d['Z_'+w]=.25+vertical_speed*max(t-3.,0.)
        d['Xctc_'+w+'i']=d['X_'+w];d['Yctc_'+w+'i']=d['Y_'+w];d['Zgnd_'+w+'i']=0.
        d['Fs_'+w]=spring;d['Fd_'+w]=damper
    return d


def test_fast_groundhook_uses_fresh_hub_motion_and_opposes_upward_wheel_motion():
    from ddevsim.strategies.shallow_press.actuation import CrawlActuation
    from ddevsim.strategies.shallow_press.config import ShallowPressConfig
    a=CrawlActuation(ShallowPressConfig(suspension_mode='fast_groundhook',wheel_velocity_filter_s=0.))
    for t in np.arange(0,3.2,.0005):
        u=a.step(float(t),float(t),_feedback(t,.1),np.zeros(9),None,{'friction':.7},.0005)
    assert np.allclose(u[4:8],200.,atol=1e-6)
    assert np.all(-u[4:8]*.1<0)


def test_total_support_cancels_passive_force_change_and_obeys_native_slew():
    from ddevsim.strategies.shallow_press.actuation import CrawlActuation
    from ddevsim.strategies.shallow_press.config import ShallowPressConfig
    a=CrawlActuation(ShallowPressConfig(suspension_mode='support',groundhook_gain=0.))
    previous=np.zeros(9)
    for t in np.arange(0,3.2001,.0005):
        d=_feedback(t,spring=1000. if t<3 else 800.,damper=100. if t>=3.1 else 0.)
        u=a.step(float(t),float(t),d,np.zeros(9),None,{'friction':.7},.0005)
        assert np.max(abs(u[4:8]-previous[4:8]))<=45400*.0005+1e-8
        if 3.05<t<3.1:assert np.allclose(u[4:8],200.)
        previous=u
    assert np.allclose(u[4:8],100.)
    assert np.allclose(800+100+u[4:8],1000)


def test_new_suspension_modes_do_not_actuate_traction_only_ablation():
    from ddevsim.strategies.shallow_press.actuation import CrawlActuation
    from ddevsim.strategies.shallow_press.config import ShallowPressConfig
    a=CrawlActuation(ShallowPressConfig(mode='traction',suspension_mode='support'))
    for t in np.arange(0,3.2,.002):
        u=a.step(float(t),float(t),_feedback(t,.1,spring=800.),np.zeros(9),None,{'friction':.7},.002)
    assert np.allclose(u[4:8],0)
