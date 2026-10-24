"""Independent full-resolution research screen; real geometry stays unassessed."""
import numpy as np
from ddevsim.units import si_factor
from .controller import WHEELS
from ddevsim.chassis_contact import rotation_matrix,plate_clearance


def exit_progress_metrics(values,scene,target_kph):
    """Separate wheel-exit low-speed dwell from merely reaching the final station."""
    t=np.asarray(values['time_s']);dt=float(np.median(np.diff(t)))
    v=np.asarray(values['exp_Vx'])/3.6;end=scene['start_station_m']+scene['length_m'];results=[]
    for wheel,label in zip(WHEELS,('FL','FR','RL','RR')):
        y=np.asarray(values['exp_Y_'+wheel]);x=np.asarray(values['exp_X_'+wheel])
        if abs(float(np.median(y))-scene['center_y_m'])>=scene['width_m']/2-scene['edge_transition_m']:continue
        zone=(x>end-.263)&(x<end+.263)
        slow=zone&(v<.2*target_kph/3.6)
        edges=np.diff(np.r_[False,slow,False].astype(int))
        pause=float(np.max(np.flatnonzero(edges==-1)-np.flatnonzero(edges==1))*dt) if slow.any() else 0.
        results.append(dict(wheel=label,observed=bool(zone.any()),
            zone_duration_s=float(zone.sum()*dt),longest_pause_s=pause,
            min_speed_kph=float(v[zone].min()*3.6) if zone.any() else None))
    return results


def input_screen(values):
    names=['imp_IMP_MYUSM_'+w for w in WHEELS]+['imp_IMP_FS_'+w for w in WHEELS]+['imp_IMP_STEER_SW']
    available=all(n in (values.dtype.names if hasattr(values,'dtype') else values) for n in names)
    if not available:return dict(input_amplitude=False,input_slew=False),dict(input_log_complete=False)
    u=np.column_stack([values[n] for n in names]);t=np.asarray(values['time_s'])
    rate=np.abs(np.diff(u,axis=0)/np.diff(t)[:,None])
    cap=np.array([500.]*4+[18200.]*4+[220.]);slew=np.array([1000.]*4+[45400.]*4+[400.])
    finite=bool(np.isfinite(u).all() and np.isfinite(rate).all() and np.all(np.diff(t)>0))
    criteria=dict(input_amplitude=finite and bool(np.all(abs(u)<=cap+1e-5)),
                  input_slew=finite and bool(np.all(rate<=slew+0.1)))
    return criteria,dict(input_log_complete=True,torque_peak_nm=float(abs(u[:,:4]).max()),
        suspension_force_peak_n=float(abs(u[:,4:8]).max()),steering_wheel_peak_deg=float(abs(u[:,8]).max()),
        torque_rate_peak_nm_s=float(rate[:,:4].max()),force_rate_peak_n_s=float(rate[:,4:8].max()),
        steering_rate_peak_deg_s=float(rate[:,8].max()))


def assumed_clearance(values,scene,settle_s=3.):
    """Full rectangle against physical road, no forces and no real CAD claim."""
    t=np.asarray(values['time_s']);static=(t>1)&(t<settle_s);active=t>=settle_s
    poses=np.column_stack([values['exp_'+n] for n in ('Xo','Yo','Zo','Roll_E','Pitch','Yaw')])
    reference=np.median(poses[static],axis=0);r0=rotation_matrix(reference[3:])
    xy=np.array([[-2.225,-.5],[.3,-.5],[.3,.5],[-2.225,.5]])
    indices=np.flatnonzero(active);sweep=[]
    for clearance in (.10,.15,.20):
        z=(clearance-reference[2]-np.min(xy@r0[2,:2]))/r0[2,2]
        points=np.column_stack((xy,np.full(4,z)));minimum=float('inf');first=None
        for start in range(0,len(indices),512):
            ids=indices[start:start+512];p=poses[ids]
            matrices=np.stack([rotation_matrix(a) for a in p[:,3:]])
            corners=np.einsum('nij,kj->nki',matrices,points)+p[:,:3,None].transpose(0,2,1)
            gap,_=plate_clearance(corners,scene);minimum=min(minimum,float(gap.min()))
            if first is None and np.any(gap<=0):first=float(t[ids[np.flatnonzero(gap<=0)[0]]])
        sweep.append(dict(static_min_clearance_m=clearance,min_gap_m=minimum,first_intersection_s=first))
    return dict(geometry_status='ASSUMED_NOT_MEASURED',bounds_m=[-2.225,.3,-.5,.5],
        physical_contact_force='NOT_MODELED',sampling_s=float(np.median(np.diff(t))),sweep=sweep,
        limitation='Sprung rectangular plane only; excludes axles/exhaust. After first intersection trajectory is counterfactual.')


def assess(values,scene,summary,settle_s=3.,crossing_kph=1.):
    x=lambda n:np.asarray(values['exp_'+n],float)*si_factor(n)
    t=values['time_s']; live=t>=settle_s
    xs=np.column_stack([x('X_'+w) for w in WHEELS])
    start=scene['start_station_m']; end=start+scene['length_m']
    mask=live&(xs[:,:2].max(axis=1)>start-.5)&(xs[:,2:].min(axis=1)<end+.6)
    has_crossing=bool(mask.any())
    if not has_crossing:mask=live
    j=np.column_stack([x('Jnc_'+w) for w in WHEELS])
    load=np.column_stack([x('Fz_'+w) for w in WHEELS])
    cmp=np.column_stack([x('CmpT_'+w+'i') for w in WHEELS])
    speed=x('Vx'); yawrate=x('AVz')
    ys=np.column_stack([x('Y_'+w) for w in WHEELS])-x('Yo')[:,None]
    slip=np.column_stack([x('AVy_'+w)*x('RRE_'+w+'i') for w in WHEELS])-(speed[:,None]-yawrate[:,None]*ys)
    loaded=(load>300)&(cmp>.0001)&live[:,None]
    lost=(load<100)&(cmp<=.0001)&mask[:,None]
    az=x('Az_SM'); base=float(np.median(az[(t>1)&(t<settle_s)]))
    az=az-base
    roll=float(np.max(abs(x('Roll_E')[live]))); pitch=float(np.max(abs(x('Pitch')[live])))
    lateral=float(np.max(abs(x('Yo')[live]-np.median(x('Yo')[(t>1)&(t<settle_s)]))))
    slipmax=float(np.max(abs(slip[loaded]))) if loaded.any() else float('inf')
    dt=float(np.median(np.diff(t))); window=max(1,round(.01/dt))
    az10=np.convolve(az[mask],np.ones(window)/window,mode='valid')
    ncload=float(np.max(load[mask])); peak=float(np.max(abs(az10)))
    longest=[]
    for i in range(4):
        edges=np.diff(np.r_[False,lost[:,i],False].astype(int))
        longest.append(float(np.max(np.flatnonzero(edges==-1)-np.flatnonzero(edges==1))*dt) if lost[:,i].any() else 0.)
    contact_fraction=((load>300)&(cmp>.0001))[mask].mean(axis=0)
    static_load=np.median(load[(t>1)&(t<settle_s)],axis=0)
    ratios=load[mask].max(axis=0)/np.maximum(static_load,1.)
    criteria=dict(crossing_observed=has_crossing,
        rear_and_stop=bool(summary.get('completed')) and bool(min(xs[-1,2:])>end+.263),controller_no_fault=not summary.get('failure_reasons'),
        roll=roll<np.deg2rad(5),pitch=pitch<np.deg2rad(5),path=lateral<.05,
        travel=bool(j[live].min()>-.149 and j[live].max()<.155),
        loaded_wheel_speed_error=slipmax<.35,impact_10ms=peak<15.,
        no_backward_motion=bool(speed[live].min()>-.08),
        contact_retention=bool(max(longest)<=.15 and min(contact_fraction)>.90),
        wheel_load_peak=bool(np.all(static_load>100) and max(ratios)<2.5))
    tail=t>t[-1]-2
    backwards=float(np.sum(np.maximum(-speed[live]-.002,0))*dt)
    criteria.update(stop_hold=bool(np.max(abs(speed[tail]))<.025),backward_distance=backwards<.02)
    input_criteria,input_metrics=input_screen(values);criteria.update(input_criteria)
    progress=exit_progress_metrics(values,scene,crossing_kph)
    criteria['no_exit_pause']=bool(progress) and all(r['observed'] and r['longest_pause_s']<.5 for r in progress)
    metrics=dict(roll_peak_deg=float(np.rad2deg(roll)),pitch_peak_deg=float(np.rad2deg(pitch)),
        path_peak_m=lateral,jounce_min_m=float(j[live].min()),jounce_max_m=float(j[live].max()),
        loaded_slip_proxy_peak_m_s=slipmax if np.isfinite(slipmax) else None,az_rms_m_s2=float(np.sqrt(np.mean(az[mask]**2))),
        az_10ms_peak_m_s2=peak,max_wheel_load_n=ncload,contact_loss_s=(lost.sum(axis=0)*dt).tolist(),
        longest_contact_loss_s=longest,loaded_contact_fraction=contact_fraction.tolist(),
        wheel_load_peak_ratio_to_static=ratios.tolist(),
        crossing_mean_kph=float(speed[mask].mean()*3.6),crossing_max_kph=float(speed[mask].max()*3.6),
        min_speed_m_s=float(speed[live].min()),backward_distance_m=backwards,
        stop_hold_peak_speed_m_s=float(np.max(abs(speed[tail]))),observed_duration_s=float(t[-1]),**input_metrics)
    criteria={k:bool(v) for k,v in criteria.items()}
    passed=all(criteria.values())
    clearance=assumed_clearance(values,scene,settle_s)
    return dict(status='SIMULATION_PASS' if passed else 'FAIL',simulation_pass=passed,acceptance_pass=False,
        scope='PARAMETERIZED_SIMULATION_SCREEN',criteria=criteria,metrics=metrics,
        chassis_diagnostic=clearance,exit_progress=progress,
        unassessed=['REAL_CHASSIS_GEOMETRY','ACTUATOR_PHYSICAL_STROKE_BANDWIDTH_POWER','FULL_TIRE_TANGENTIAL_SLIP'],
        controller=summary)
