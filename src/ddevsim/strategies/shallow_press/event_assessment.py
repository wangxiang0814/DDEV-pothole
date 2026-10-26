"""Versioned variable-speed task screen; legacy physical thresholds are retained."""
import numpy as np
from .assessment import assess
from .controller import WHEELS
from ddevsim.units import si_factor
from .event_phases import required_wheels


def longest_interval(mask,dt):
    edges=np.diff(np.r_[False,mask,False].astype(int))
    return float(np.max(np.flatnonzero(edges==-1)-np.flatnonzero(edges==1))*dt) if np.any(mask) else 0.


def envelope_series(xs,expected,scene,c):
    start=scene['start_station_m'];end=start+scene['length_m'];caps=[]
    for i in np.flatnonzero(expected):
        x=xs[:,i]
        before=np.sqrt((c.entry_kph/3.6)**2+2*c.braking_acceleration_m_s2*np.maximum(start-c.braking_margin_m-x,0))
        bottom=np.minimum(c.pit_kph/3.6,np.sqrt((c.climb_kph/3.6)**2+
            2*c.braking_acceleration_m_s2*np.maximum(end-c.radius_m-c.braking_margin_m-x,0)))
        caps.append(np.select([x<start-c.braking_margin_m,x<start+c.radius_m,
            x<end-c.radius_m,x<end+c.radius_m],[before,c.entry_kph/3.6,bottom,c.climb_kph/3.6],default=np.inf))
    if not caps:return np.full(len(xs),np.inf)
    cap=np.min(caps,axis=0);relevant=xs[:,expected]
    active=(relevant.max(axis=1)>=start-c.braking_margin_m)&(relevant.min(axis=1)<end+c.radius_m)
    return np.where(active,np.minimum(cap,c.pit_kph/3.6),cap)


def assess_event(values,scene,summary,config,diagnostics,*,case_id):
    legacy=assess(values,scene,summary,config.settle_s,config.crossing_kph)
    result=dict(legacy);criteria=dict(legacy['criteria']);criteria.pop('speed_tracking');criteria.pop('no_exit_pause')
    e=config.event;t=np.asarray(values['time_s']);dt=float(np.median(np.diff(t)))
    x=lambda n:np.asarray(values['exp_'+n],float)*si_factor(n)
    xs=np.column_stack([x('X_'+w) for w in WHEELS]);ys=np.column_stack([x('Y_'+w) for w in WHEELS])
    static=(t>1)&(t<config.settle_s)
    expected=required_wheels(case_id)
    observed_coverage=abs(np.median(ys[static],axis=0)-scene['center_y_m'])<scene['width_m']/2-scene['edge_transition_m']
    start=scene['start_station_m'];end=start+scene['length_m']
    crossing=(t>=config.settle_s)&(xs[:,:2].max(axis=1)>start-.5)&(xs[:,2:].min(axis=1)<end+.6)
    cap=envelope_series(xs,expected,scene,e);speed=x('Vx');excess=(speed-cap)*3.6
    times=np.array([r['time_s'] for r in diagnostics],float)
    references=np.array([r.get('speed_reference_m_s',float('nan')) for r in diagnostics],float)
    valid=(len(times)>0 and np.isfinite(times).all() and np.all(np.diff(times)>0) and
        times[0]<=config.settle_s+.03 and times[-1]>=t[-1]-.04 and np.max(np.diff(times),initial=0)<=.031 and
        summary.get('task_version')=='event_crawl_v1' and
        all(len(r.get('event',{}).get('phases',[]))==4 for r in diagnostics) and
        np.isfinite(references).all() and np.all(references>=0))
    reference=np.interp(t,times,references) if valid else np.zeros(len(t))
    overspeed=crossing&(excess>e.speed_tolerance_kph)
    overspeed_s=longest_interval(overspeed,dt)
    excess_peak=float(np.max(excess[crossing])) if crossing.any() else float('inf')
    stall=(t>=config.settle_s+config.body_leveling_s)&(reference>.005)&(abs(speed)<np.minimum(.02,.2*reference))
    stall_s=longest_interval(stall,dt)
    observed=[]
    for i in np.flatnonzero(expected):
        def crossing_at(boundary):
            ids=np.flatnonzero((xs[:-1,i]<boundary)&(xs[1:,i]>=boundary)&(t[1:]>=config.settle_s))+1
            fraction=(boundary-xs[ids-1,i])/(xs[ids,i]-xs[ids-1,i])
            lateral=ys[ids-1,i]+fraction*(ys[ids,i]-ys[ids-1,i])
            return ids[abs(lateral-scene['center_y_m'])<scene['width_m']/2-scene['edge_transition_m']]
        entry=crossing_at(start);exit_=crossing_at(end+e.radius_m)
        entry_time=None;exit_time=None
        if len(entry):
            k=entry[0];entry_time=float(t[k]);later=exit_[t[exit_]>entry_time]
            if len(later):exit_time=float(t[later[0]])
        observed.append(dict(wheel=WHEELS[i],entered_s=entry_time,crossed_s=exit_time))
    raw_complete=bool(observed and all(r['entered_s'] is not None and r['crossed_s'] is not None for r in observed))
    state=summary.get('wheel_state') or {};entered=np.array(state.get('entered',[False]*4),bool);crossed=np.array(state.get('crossed',[False]*4),bool)
    phase_complete=(entered.shape==(4,) and crossed.shape==(4,) and expected.any() and
        bool(np.all(entered[expected]) and np.all(crossed[expected])) and raw_complete)
    moving=crossing&(abs(speed)>.02)
    reference_motion=bool(valid and moving.any() and np.all(reference[moving]>.005))
    reference_excess=(reference-cap)*3.6
    reference_peak=float(np.max(reference_excess[crossing])) if crossing.any() else float('inf')
    reference_overspeed_s=longest_interval(crossing&(reference_excess>e.speed_tolerance_kph),dt)
    tracking_excess=(speed-reference)*3.6
    tracking_peak=float(np.max(tracking_excess[crossing])) if crossing.any() else float('inf')
    tracking_overspeed_s=longest_interval(crossing&(tracking_excess>e.speed_tolerance_kph),dt)
    criteria.update(event_diagnostics_complete=bool(valid),phase_completion=bool(phase_complete),
        event_speed_envelope=bool(crossing.any() and excess_peak<=e.max_speed_overshoot_kph and overspeed_s<=e.max_overspeed_duration_s),
        event_scene_coverage=bool(np.array_equal(observed_coverage,expected)),
        event_motion_reference=reference_motion,
        event_reference_envelope=bool(valid and crossing.any() and reference_peak<=e.max_speed_overshoot_kph and reference_overspeed_s<=e.max_overspeed_duration_s),
        event_reference_overspeed=bool(valid and crossing.any() and tracking_peak<=e.max_speed_overshoot_kph and tracking_overspeed_s<=e.max_overspeed_duration_s),
        finite_progress=bool(crossing.any() and stall_s<=e.max_uncommanded_stall_s))
    metrics=dict(legacy['metrics'],event_speed_overshoot_peak_kph=excess_peak if np.isfinite(excess_peak) else None,
        event_longest_overspeed_s=overspeed_s,event_longest_uncommanded_stall_s=stall_s,
        event_total_duration_s=float(t[-1]),requested_approach_kph=config.cruise_kph,
        event_raw_crossings=observed,event_reference_overshoot_peak_kph=reference_peak if np.isfinite(reference_peak) else None,
        event_reference_overspeed_s=reference_overspeed_s,
        event_speed_above_reference_peak_kph=tracking_peak if np.isfinite(tracking_peak) else None,
        event_speed_above_reference_longest_s=tracking_overspeed_s,
        phase_reference_scope='ACTUAL_RAW_WHEEL_POSITIONS_AND_FROZEN_EVENT_CONFIG')
    passed=all(criteria.values())
    result.update(status='SIMULATION_PASS' if passed else 'FAIL',simulation_pass=passed,acceptance_pass=False,
        task_version='event_crawl_v1',scope='EVENT_CRAWL_PARAMETERIZED_SCREEN',criteria=criteria,metrics=metrics,
        assessment_revision='raw_events_reference_v3',
        legacy_assessment=dict(task_version='constant_speed_v1',status=legacy['status'],criteria=legacy['criteria'],
            requested_crossing_kph=config.crossing_kph,exit_progress=legacy['exit_progress']),
        task_change='Variable spatial speed envelope and finite progress replace constant mean speed and fixed-speed exit-pause gate; physical limits unchanged')
    return result
