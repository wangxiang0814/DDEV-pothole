import numpy as np
import pytest
from ddevsim.strategies.shallow_press.config import ShallowPressConfig


def test_fast_load_correction_is_local_bounded_and_increases_only_for_load_deficit():
    from ddevsim.strategies.shallow_press.fast_load import FastLoadFeedback
    c=ShallowPressConfig(fast_load_gain=1.,fast_load_limit_n=2000)
    a=FastLoadFeedback(c);b=FastLoadFeedback(c)
    j=np.full(4,.04);x=np.array([101.2]*4);y=np.array([.63,-.63,.63,-.63])
    scene=dict(start_station_m=101.1,length_m=.8,width_m=.9,edge_transition_m=.05,center_y_m=-.63)
    for model in (a,b):model.step(0.,np.full(4,3000.),j,x,y,scene,.002)
    for _ in range(50):out=a.step(5.,np.zeros(4),j,x,y,scene,.002)
    assert out==pytest.approx([0,2000,0,2000])
    assert b.step(5.,np.full(4,3000.),j,x,y,scene,.002)==pytest.approx(np.zeros(4))
    for _ in range(100):out=a.step(5.,np.full(4,9000.),j,x,y,scene,.002)
    assert out==pytest.approx([0,-2000,0,-2000])
    assert a.step(5.,np.zeros(4),j,x+5,y,scene,.002)==pytest.approx(np.zeros(4))


def test_fast_load_path_keeps_working_force_and_slew_and_unloads_on_fault():
    from ddevsim.strategies.shallow_press.actuation import CrawlActuation
    from test_actuation import feedback
    c=ShallowPressConfig(fast_load_gain=.5,fast_load_limit_n=2000)
    a=CrawlActuation(c);d=feedback();scene=dict(start_station_m=101.1,length_m=.8,width_m=3.,edge_transition_m=.05,center_y_m=0.,friction=.7)
    for w in ('L1','R1','L2','R2'):d.update({'X_'+w:101.2,'Jnc_'+w:.04})
    a.step(0.,0.,d,np.zeros(9),None,scene,.0005)
    for w in ('L1','R1','L2','R2'):d['Fz_'+w]=0.
    old=np.zeros(9)
    for t in np.arange(5,5.1,.0005):
        u=a.step(float(t),float(t),d,np.r_[np.zeros(4),np.full(4,3400.),0.],None,scene,.0005)
        assert np.max(abs(u[4:8]))<=c.force_working_limit_n
        assert np.max(abs(u[4:8]-old[4:8]))<=c.force_rate_n_s*.0005+1e-7
        old=u
    assert np.min(u[4:8])>3400
    for t in np.arange(5.1,5.3,.0005):u=a.step(float(t),float(t),d,np.zeros(9),None,scene,.0005,True)
    assert u==pytest.approx(np.zeros(9))
