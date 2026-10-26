import numpy as np
from test_assessment import _complete_values
from test_preview_control import SCENE


def complete_trial():
    d=_complete_values();d['time_s']*=6.;d['exp_Vx']/=6.
    for w in ('L1','R1','L2','R2'):d['exp_AVy_'+w]/=6.
    summary=dict(completed=True,failure_reasons=[],task_version='event_crawl_v1',
        wheel_state=dict(expected=[False,True,False,True],entered=[False,True,False,True],crossed=[False,True,False,True]))
    times=np.r_[np.arange(3.,d['time_s'][-1],.02),d['time_s'][-1]]
    diagnostics=[dict(time_s=float(t),speed_reference_m_s=.15 if 18<=t<60 else 0.,event=dict(phases=['BYPASS','ENTRY','BYPASS','ENTRY'])) for t in times]
    return d,summary,diagnostics


def test_slow_event_crossing_keeps_legacy_failure_but_uses_explicit_new_task():
    from ddevsim.strategies.shallow_press.event_assessment import assess_event
    from ddevsim.strategies.shallow_press.config import ShallowPressConfig
    d,s,rows=complete_trial();r=assess_event(d,SCENE,s,ShallowPressConfig(policy='event_crawl'),rows,case_id='right_track')
    assert not r['legacy_assessment']['criteria']['speed_tracking']
    assert 'speed_tracking' not in r['criteria'] and 'no_exit_pause' not in r['criteria']
    assert r['criteria']['event_speed_envelope'] and r['criteria']['finite_progress']
    assert r['simulation_pass'] and r['acceptance_pass'] is False


def test_missing_phase_rear_completion_or_overspeed_cannot_pass():
    from ddevsim.strategies.shallow_press.event_assessment import assess_event
    from ddevsim.strategies.shallow_press.config import ShallowPressConfig
    d,s,rows=complete_trial();c=ShallowPressConfig(policy='event_crawl')
    assert not assess_event(d,SCENE,s,c,[],case_id='right_track')['simulation_pass']
    s['wheel_state']['crossed'][3]=False
    assert not assess_event(d,SCENE,s,c,rows,case_id='right_track')['criteria']['phase_completion']
    s['wheel_state']['crossed'][3]=True
    d['exp_Vx'][(d['time_s']>35)&(d['time_s']<36)]=4.
    assert not assess_event(d,SCENE,s,c,rows,case_id='right_track')['criteria']['event_speed_envelope']


def test_commanded_stall_before_pit_is_not_hidden_by_event_region():
    from ddevsim.strategies.shallow_press.event_assessment import assess_event
    from ddevsim.strategies.shallow_press.config import ShallowPressConfig
    d,s,rows=complete_trial()
    for row in rows:
        if 5<=row['time_s']<14.2:row['speed_reference_m_s']=.15
    assert not assess_event(d,SCENE,s,ShallowPressConfig(policy='event_crawl'),rows,case_id='right_track')['criteria']['finite_progress']


def test_nonfinite_reference_cannot_supply_valid_event_evidence():
    from ddevsim.strategies.shallow_press.event_assessment import assess_event
    from ddevsim.strategies.shallow_press.config import ShallowPressConfig
    d,s,rows=complete_trial();rows[10]['speed_reference_m_s']=float('nan')
    assert not assess_event(d,SCENE,s,ShallowPressConfig(policy='event_crawl'),rows,case_id='right_track')['criteria']['event_diagnostics_complete']


def test_very_slow_positive_reference_still_requires_progress():
    from ddevsim.strategies.shallow_press.event_assessment import assess_event
    from ddevsim.strategies.shallow_press.config import ShallowPressConfig
    d,s,rows=complete_trial()
    for row in rows:
        if 5<=row['time_s']<14.2:row['speed_reference_m_s']=.01
    assert not assess_event(d,SCENE,s,ShallowPressConfig(policy='event_crawl'),rows,case_id='right_track')['criteria']['finite_progress']


def test_summary_cannot_forge_missing_rear_entry_in_raw_trajectory():
    from ddevsim.strategies.shallow_press.event_assessment import assess_event
    from ddevsim.strategies.shallow_press.config import ShallowPressConfig
    d,s,rows=complete_trial();d['exp_X_R2'][:]=SCENE['start_station_m']+SCENE['length_m']+1.
    r=assess_event(d,SCENE,s,ShallowPressConfig(policy='event_crawl'),rows,case_id='right_track')
    assert not r['simulation_pass'] and not r['criteria']['phase_completion']


def test_missing_rear_track_cannot_shrink_the_required_mission():
    from ddevsim.strategies.shallow_press.event_assessment import assess_event
    from ddevsim.strategies.shallow_press.config import ShallowPressConfig
    d,s,rows=complete_trial();d['exp_X_R2'][:]=103.;d['exp_Y_R2'][:]=.63
    s['wheel_state']['entered'][3]=False;s['wheel_state']['crossed'][3]=False
    assert not assess_event(d,SCENE,s,ShallowPressConfig(policy='event_crawl'),rows,case_id='right_track')['simulation_pass']


def test_crossing_station_outside_the_pit_track_is_not_a_pit_crossing():
    from ddevsim.strategies.shallow_press.event_assessment import assess_event
    from ddevsim.strategies.shallow_press.config import ShallowPressConfig
    d,s,rows=complete_trial();d['exp_Y_R2'][d['time_s']>18]=.63
    assert not assess_event(d,SCENE,s,ShallowPressConfig(policy='event_crawl'),rows,case_id='right_track')['criteria']['phase_completion']


def test_zero_reference_cannot_disguise_uncommanded_pit_motion_as_valid_progress():
    from ddevsim.strategies.shallow_press.event_assessment import assess_event
    from ddevsim.strategies.shallow_press.config import ShallowPressConfig
    d,s,rows=complete_trial()
    for row in rows:row['speed_reference_m_s']=0.
    assert not assess_event(d,SCENE,s,ShallowPressConfig(policy='event_crawl'),rows,case_id='right_track')['simulation_pass']


def test_reference_above_spatial_envelope_is_not_a_valid_plan_even_when_vehicle_is_slow():
    from ddevsim.strategies.shallow_press.event_assessment import assess_event
    from ddevsim.strategies.shallow_press.config import ShallowPressConfig
    d,s,rows=complete_trial()
    for row in rows:row['speed_reference_m_s']=4./3.6 if 18<=row['time_s']<60 else 0.
    assert not assess_event(d,SCENE,s,ShallowPressConfig(policy='event_crawl'),rows,case_id='right_track')['simulation_pass']


def test_tiny_positive_reference_cannot_hide_motion_above_the_planned_variable_speed():
    from ddevsim.strategies.shallow_press.event_assessment import assess_event
    from ddevsim.strategies.shallow_press.config import ShallowPressConfig
    d,s,rows=complete_trial()
    for row in rows:row['speed_reference_m_s']=.006 if 18<=row['time_s']<60 else 0.
    assert not assess_event(d,SCENE,s,ShallowPressConfig(policy='event_crawl'),rows,case_id='right_track')['simulation_pass']
