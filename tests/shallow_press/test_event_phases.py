import numpy as np
import pytest
from test_preview_control import SCENE


def test_right_track_phases_require_both_axles_and_real_recontact_dwell():
    from ddevsim.strategies.shallow_press.event_phases import WheelPhaseTracker
    from ddevsim.strategies.shallow_press.event_config import EventCrawlConfig
    tracker=WheelPhaseTracker(EventCrawlConfig())
    y=np.array([.63,-.63,.63,-.63]);fz=np.full(4,3300.);cmp=np.full(4,.015)
    out=tracker.update(0.,np.full(4,99.),y,fz,cmp,SCENE)
    assert out['expected']==[False,True,False,True]
    assert out['phases']==['BYPASS','APPROACH','BYPASS','APPROACH']
    x=np.array([101.15,101.15,99.225,99.225]);fz[1]=0;cmp[1]=0
    out=tracker.update(.02,x,y,fz,cmp,SCENE)
    assert out['phases'][1]=='UNLOADED'
    fz[1]=3300;cmp[1]=.015
    out=tracker.update(.04,x,y,fz,cmp,SCENE)
    assert out['phases'][1]=='RECONTACT'
    out=tracker.update(.14,x,y,fz,cmp,SCENE)
    assert out['phases'][1]=='ENTRY'
    out=tracker.update(.16,np.array([102.3,102.3,100.375,100.375]),y,fz,cmp,SCENE)
    assert out['phases'][1]=='RECOVERED' and not all(out['crossed'][i] for i in [1,3])
    out=tracker.update(.18,np.full(4,99.),y,fz,cmp,SCENE)
    assert out['phases'][3]=='APPROACH' and not out['crossed'][3]


def test_wide_phase_geometry_and_instance_isolation():
    from ddevsim.strategies.shallow_press.event_phases import WheelPhaseTracker
    from ddevsim.strategies.shallow_press.event_config import EventCrawlConfig
    a=WheelPhaseTracker(EventCrawlConfig());b=WheelPhaseTracker(EventCrawlConfig())
    scene=dict(SCENE,width_m=1.8,center_y_m=0.)
    args=(np.array([100.8,101.2,101.5,101.8]),np.array([.63,-.63,.63,-.63]),np.full(4,3300.),np.full(4,.015),scene)
    out=a.update(0.,*args)
    assert out['base_phases']==['PREPARE','ENTRY','BOTTOM','CLIMB']
    assert not b.entered.any()
    with pytest.raises(ValueError):a.update(-1.,*args)


def test_boundary_noise_does_not_chatter_and_lateral_drift_cannot_drop_required_wheels():
    from ddevsim.strategies.shallow_press.event_phases import WheelPhaseTracker
    from ddevsim.strategies.shallow_press.event_config import EventCrawlConfig
    a=WheelPhaseTracker(EventCrawlConfig());y=np.array([.63,-.63,.63,-.63]);fz=np.full(4,3300.);cmp=np.full(4,.015)
    a.update(0.,np.full(4,101.08),y,fz,cmp,SCENE)
    for i,position in enumerate([101.101,101.099,101.102,101.098]):
        out=a.update((i+1)*.02,np.full(4,position),y,fz,cmp,SCENE)
        assert out['base_phases'][1]=='PREPARE'
    out=a.update(.12,np.full(4,101.12),y+3,fz,cmp,SCENE)
    assert out['expected']==[False,True,False,True] and out['base_phases'][1]=='ENTRY'
