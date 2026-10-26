import numpy as np
from test_assessment import _complete_values
from test_preview_control import SCENE


def complete_trial():
    d=_complete_values();d['time_s']*=6.;d['exp_Vx']/=6.
    for w in ('L1','R1','L2','R2'):d['exp_AVy_'+w]/=6.
    summary=dict(completed=True,failure_reasons=[],task_version='event_crawl_v1',
        wheel_state=dict(expected=[False,True,False,True],entered=[False,True,False,True],crossed=[False,True,False,True]))
    times=np.r_[np.arange(3.,d['time_s'][-1],.02),d['time_s'][-1]]
    diagnostics=[dict(time_s=float(t),speed_reference_m_s=.15,event=dict(phases=['BYPASS','ENTRY','BYPASS','ENTRY'])) for t in times]
    return d,summary,diagnostics


def test_slow_event_crossing_keeps_legacy_failure_but_uses_explicit_new_task():
    from ddevsim.strategies.shallow_press.event_assessment import assess_event
    from ddevsim.strategies.shallow_press.config import ShallowPressConfig
    d,s,rows=complete_trial();r=assess_event(d,SCENE,s,ShallowPressConfig(policy='event_crawl'),rows)
    assert not r['legacy_assessment']['criteria']['speed_tracking']
    assert 'speed_tracking' not in r['criteria'] and 'no_exit_pause' not in r['criteria']
    assert r['criteria']['event_speed_envelope'] and r['criteria']['finite_progress']
    assert r['simulation_pass'] and r['acceptance_pass'] is False


def test_missing_phase_rear_completion_or_overspeed_cannot_pass():
    from ddevsim.strategies.shallow_press.event_assessment import assess_event
    from ddevsim.strategies.shallow_press.config import ShallowPressConfig
    d,s,rows=complete_trial();c=ShallowPressConfig(policy='event_crawl')
    assert not assess_event(d,SCENE,s,c,[])['simulation_pass']
    s['wheel_state']['crossed'][3]=False
    assert not assess_event(d,SCENE,s,c,rows)['criteria']['phase_completion']
    s['wheel_state']['crossed'][3]=True
    d['exp_Vx'][(d['time_s']>35)&(d['time_s']<36)]=4.
    assert not assess_event(d,SCENE,s,c,rows)['criteria']['event_speed_envelope']
