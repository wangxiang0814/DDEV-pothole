"""Plant-truth audit for a finite RR moving packet gap; never reclassify task FAIL."""
import csv
import json
import math
from ddevsim.static_wheel_lift.config import FEEDBACK_HEALTH, FEEDBACK_FAULT_TRIAL, SUPPORT_QP


def audit_moving_packet(run,fault_time_s,args):
    result=json.loads((run/'result.json').read_text())
    rows=list(csv.DictReader((run/'rear_control_20ms.csv').open()))
    return assess_moving_response(rows,result,fault_time_s,args)


def assess_moving_response(rows,result,fault_time_s,args):
    post=[r for r in rows if fault_time_s is not None and float(r['time_s'])>=fault_time_s]
    cfg=result['controller_config']['rear']
    stale=[r for r in post if float(r['measurement_age_s'])>FEEDBACK_HEALTH.max_packet_age_s]
    abort=[r for r in post if r['mode']=='RR_ABORT_STOP']
    swing=[r for r in post if r['mode'] in ('RR_CRAWL','RR_STOP','RR_ABORT_STOP','RR_ABORT_EXIT')]
    recovery=[r for r in post if r['mode'] in ('RR_LOWERING','RR_RETURN','RR_COMPLETE')]
    start=result['scenario']['start_station_m'];end=start+result['scenario']['length_m']
    solid=lambda station: station<=start-cfg['crossing_clearance_m'] or station>=end+cfg['crossing_clearance_m']
    keys=('time_s','vx_kph','x_rr_m','yo_m','roll_deg','pitch_deg','min_support_n','zmp_lambda_min',
          'com_lambda_min','rr_clearance_m','fz_rr_filtered_n','measurement_age_s','support_qp_failure_s')
    keys+=tuple(f'{kind}_{c}_{unit}' for kind,unit in (('travel','mm'),('fact','n')) for c in ('fl','fr','rl','rr'))
    finite=bool(post) and all(math.isfinite(float(r.get(k,float('nan')))) for r in post for k in keys)
    rate=max((abs(float(b[f'fact_{c}_n'])-float(a[f'fact_{c}_n']))/(float(b['time_s'])-float(a['time_s']))
        for a,b in zip(post,post[1:]) if float(b['time_s'])>float(a['time_s']) for c in ('fl','fr','rl','rr')),default=0.)
    stopped_since=None
    for r in post:
        if (r['mode']=='RR_ABORT_STOP' and abs(float(r['vx_kph']))<=cfg['stationary_kph'] and
                float(r['measurement_age_s'])<=FEEDBACK_HEALTH.max_packet_age_s and
                float(r['time_s'])>=fault_time_s+args.packet_outage_s):
            if stopped_since is None: stopped_since=float(r['time_s'])
        else: stopped_since=None
    stopped_hold=float(post[-1]['time_s'])-stopped_since if post and stopped_since is not None else 0.
    recovered=bool(recovery and post[-1]['mode']=='RR_COMPLETE' and result['criteria']['four_wheel_recovered'])
    safe_hold=bool(post and post[-1]['mode']=='RR_ABORT_STOP' and stopped_hold>=FEEDBACK_FAULT_TRIAL.moving_stopped_hold_s and
        float(post[-1]['time_s'])-fault_time_s>=FEEDBACK_FAULT_TRIAL.moving_observe_s-cfg['control_period_s'])
    common={
        'fault_injected_in_moving_pit':bool(post) and start<float(post[0]['x_rr_m'])<end and post[0]['mode']=='RR_CRAWL',
        'packet_gap_finished_and_feedback_restored':bool(post) and float(post[-1]['time_s'])>=fault_time_s+args.packet_outage_s and
            float(post[-1]['measurement_age_s'])<=FEEDBACK_HEALTH.max_packet_age_s and
            (not stale or any(float(r['time_s'])>float(stale[-1]['time_s']) and
             float(r['measurement_age_s'])<=FEEDBACK_HEALTH.max_packet_age_s for r in post)),
        'finite_truth':finite,
        'support_preserved':bool(post) and min(float(r['min_support_n']) for r in post)>=cfg['support_floor_n'],
        'attitude_safe':bool(post) and max(abs(float(r[k])) for r in post for k in ('roll_deg','pitch_deg'))<=cfg['attitude_limit_deg'],
        'swing_triangle_safe':bool(swing) and min(float(r[k]) for r in swing for k in ('zmp_lambda_min','com_lambda_min'))>=cfg['lambda_safe'],
        'swing_clear_and_unloaded':bool(swing) and min(float(r['rr_clearance_m']) for r in swing)>=cfg['clearance_m'] and max(float(r['fz_rr_filtered_n']) for r in swing)<=cfg['wheel_unloaded_n'],
        'travel_safe':bool(post) and min(float(r[f'travel_{c}_mm']) for r in post for c in ('fl','fr','rl','rr'))>cfg['travel_rebound_abort_mm'] and max(float(r[f'travel_{c}_mm']) for r in post for c in ('fl','fr','rl','rr'))<cfg['travel_jounce_abort_mm'],
        'force_rate_safe':bool(post) and max(abs(float(r[f'fact_{c}_n'])) for r in post for c in ('fl','fr','rl','rr'))<=cfg['sim_force_limit_n'] and rate<=cfg['sim_force_slew_n_s'],
        'path_safe':bool(post) and max(abs(float(r['yo_m'])-result['path_reference_yo_m']) for r in post)<=cfg['max_lateral_error_m'],
        'no_persistent_qp_failure':bool(post) and max(float(r['support_qp_failure_s']) for r in post)<=SUPPORT_QP.max_continuous_failure_s,
        'native_completed':bool(result['criteria']['native_completed']),
        'stopped_solid_restoration':all(abs(float(r['vx_kph']))<=cfg['stationary_kph'] and solid(float(r['x_rr_m'])) for r in recovery),
    }
    if stale:
        first_stale=float(stale[0]['time_s'])
        common.update(task_stays_fail=result['status']=='FAIL' and result['rr_abort_reason']=='feedback packet stale',
            abort_latched=bool(abort),no_progress_from_stale=all(r['mode']=='RR_ABORT_STOP' for r in stale),
            no_resumed_crawl=not any(r['mode']=='RR_CRAWL' and float(r['time_s'])>=first_stale for r in post),
            native_rate_safe=result['support_allocation']['abort_max_native_output_slew_n_s']<=cfg['sim_force_slew_n_s'],
            recovered_or_stable_hold=recovered or safe_hold)
        response='RECOVERED' if recovered else 'SAFE_HOLD'
    else:
        common.update(normal_cycle_pass=result['status']=='PASS' and all(result['criteria'].values()),
                      no_false_abort=not abort and result['rr_abort_reason'] is None)
        response='PASS'
    return {'response_status':response if all(common.values()) else 'FAIL','task_status':result['status'],
        'criteria':common,'fault_time_s':fault_time_s,'packet_outage_s':args.packet_outage_s,
        'maximum_packet_age_s':max((float(r['measurement_age_s']) for r in post),default=None),
        'stopped_hold_s':stopped_hold,
        'min_support_n':min((float(r['min_support_n']) for r in post),default=None),
        'max_attitude_deg':max((abs(float(r[k])) for r in post for k in ('roll_deg','pitch_deg')),default=None),
        'min_swing_clearance_m':min((float(r['rr_clearance_m']) for r in swing),default=None),
        'max_lateral_m':max((abs(float(r['yo_m'])-result['path_reference_yo_m']) for r in post),default=None),
        'scope':'Finite RR moving packet gap, followed by healthy data; not indefinite blind braking, actuator failure or all fault timings.'}
