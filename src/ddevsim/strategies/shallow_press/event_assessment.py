"""Versioned variable-speed task screen; legacy physical thresholds are retained."""
import numpy as np
from .assessment import assess
from .controller import WHEELS
from ddevsim.units import si_factor


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


def assess_event(values,scene,summary,config,diagnostics):
    legacy=assess(values,scene,summary,config.settle_s,config.crossing_kph)
    result=dict(legacy);criteria=dict(legacy['criteria']);criteria.pop('speed_tracking');criteria.pop('no_exit_pause')
    e=config.event;t=np.asarray(values['time_s']);dt=float(np.median(np.diff(t)))
    x=lambda n:np.asarray(values['exp_'+n],float)*si_factor(n)
    xs=np.column_stack([x('X_'+w) for w in WHEELS]);ys=np.column_stack([x('Y_'+w) for w in WHEELS])
    static=(t>1)&(t<config.settle_s)
    expected=abs(np.median(ys[static],axis=0)-scene['center_y_m'])<scene['width_m']/2-scene['edge_transition_m']
    start=scene['start_station_m'];end=start+scene['length_m']
    crossing=(t>=config.settle_s)&(xs[:,:2].max(axis=1)>start-.5)&(xs[:,2:].min(axis=1)<end+.6)
    cap=envelope_series(xs,expected,scene,e);speed=x('Vx');excess=(speed-cap)*3.6
    times=np.array([r['time_s'] for r in diagnostics],float)
    valid=(len(times)>0 and np.isfinite(times).all() and np.all(np.diff(times)>0) and
        times[0]<=config.settle_s+.03 and times[-1]>=t[-1]-.04 and np.max(np.diff(times),initial=0)<=.031 and
        summary.get('task_version')=='event_crawl_v1' and
        all(len(r.get('event',{}).get('phases',[]))==4 for r in diagnostics))
    reference=np.interp(t,times,[r['speed_reference_m_s'] for r in diagnostics]) if valid else np.zeros(len(t))
    overspeed=crossing&(excess>e.speed_tolerance_kph)
    overspeed_s=longest_interval(overspeed,dt)
    excess_peak=float(np.max(excess[crossing])) if crossing.any() else float('inf')
    stall=crossing&(reference>.03)&(abs(speed)<.02)
    stall_s=longest_interval(stall,dt)
    state=summary.get('wheel_state') or {};entered=np.array(state.get('entered',[False]*4),bool);crossed=np.array(state.get('crossed',[False]*4),bool)
    phase_complete=(entered.shape==(4,) and crossed.shape==(4,) and expected.any() and
        bool(np.all(entered[expected]) and np.all(crossed[expected])))
    criteria.update(event_diagnostics_complete=bool(valid),phase_completion=bool(phase_complete),
        event_speed_envelope=bool(crossing.any() and excess_peak<=e.max_speed_overshoot_kph and overspeed_s<=e.max_overspeed_duration_s),
        finite_progress=bool(crossing.any() and stall_s<=e.max_uncommanded_stall_s))
    metrics=dict(legacy['metrics'],event_speed_overshoot_peak_kph=excess_peak if np.isfinite(excess_peak) else None,
        event_longest_overspeed_s=overspeed_s,event_longest_uncommanded_stall_s=stall_s,
        event_total_duration_s=float(t[-1]),requested_approach_kph=config.cruise_kph,
        phase_reference_scope='ACTUAL_RAW_WHEEL_POSITIONS_AND_FROZEN_EVENT_CONFIG')
    passed=all(criteria.values())
    result.update(status='SIMULATION_PASS' if passed else 'FAIL',simulation_pass=passed,acceptance_pass=False,
        task_version='event_crawl_v1',scope='EVENT_CRAWL_PARAMETERIZED_SCREEN',criteria=criteria,metrics=metrics,
        legacy_assessment=dict(task_version='constant_speed_v1',status=legacy['status'],criteria=legacy['criteria'],
            requested_crossing_kph=config.crossing_kph,exit_progress=legacy['exit_progress']),
        task_change='Variable spatial speed envelope and finite progress replace constant mean speed and fixed-speed exit-pause gate; physical limits unchanged')
    return result
