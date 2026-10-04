"""Inject a geometry-triggered RR over-pit abort and audit plant-truth response."""
import csv
import argparse
from dataclasses import replace
import json
from pathlib import Path
import sys
import run_right_side_full_cycle as cycle
from ddevsim.static_wheel_lift.config import PitAbortTrialConfig, SUPPORT_QP

CFG = PitAbortTrialConfig()


def inject_over_pit_abort(rear, now_s, station, config=None):
    config = CFG if config is None else config
    start, length = rear.scenario['start_station_m'], rear.scenario['length_m']
    if rear.mode != 'RR_CRAWL' or not start + config.trigger_fraction * length <= station < start + length:
        return False
    rear.abort_reason = 'injected RR over-pit abort'
    rear._enter('RR_ABORT_STOP', now_s)
    return True


class PitAbortController(cycle.FullRightSideController):
    active = None
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.fault_time_s = None
        PitAbortController.active = self

    def __call__(self, now_s, exports):
        command = super().__call__(now_s, exports)
        if self.rear_started and self.fault_time_s is None:
            observed = dict(zip(cycle.front_run.EXPORTS, exports))
            if inject_over_pit_abort(self.rear, now_s, observed['X_R2']):
                self.fault_time_s = now_s
        return command


def audit(run, fault_time_s):
    result = json.loads((run / 'result.json').read_text(encoding='utf-8'))
    with (run / 'rear_control_20ms.csv').open(encoding='utf-8', newline='') as stream:
        rows = list(csv.DictReader(stream))
    abort = [r for r in rows if r['mode'] == 'RR_ABORT_STOP']
    swing = [r for r in rows if r['mode'] in ('RR_ABORT_STOP','RR_ABORT_EXIT')]
    post = [r for r in rows if fault_time_s is not None and float(r['time_s']) >= fault_time_s]
    lowering = [r for r in rows if r['mode'] in ('RR_LOWERING', 'RR_RETURN')]
    cfg = result['controller_config']['rear']
    start = result['scenario']['start_station_m']
    end = start + result['scenario']['length_m']
    solid = lambda x: x <= start - cfg['crossing_clearance_m'] or x >= end + cfg['crossing_clearance_m']
    stopped = [r for r in abort if abs(float(r['vx_kph'])) <= cfg['stationary_kph']]
    stopped_hold = 0.
    segment_start = None
    for row in rows:
        if row['mode'] == 'RR_ABORT_STOP' and abs(float(row['vx_kph'])) <= cfg['stationary_kph']:
            if segment_start is None:
                segment_start = float(row['time_s'])
            stopped_hold = max(stopped_hold, float(row['time_s']) - segment_start)
        else:
            segment_start = None
    native_complete = bool(result['criteria']['native_completed'])
    recovered = bool(native_complete and rows and rows[-1]['mode'] == 'RR_COMPLETE' and result['criteria']['four_wheel_recovered'])
    terminal_hold = float(rows[-1]['time_s']) - segment_start if rows and segment_start is not None else 0.
    bounded_observation = (fault_time_s is not None and bool(rows) and
        float(rows[-1]['time_s']) - fault_time_s >= CFG.max_observe_after_abort_s - cfg['control_period_s'])
    safe_hold = bool(native_complete and bounded_observation and rows and rows[-1]['mode'] == 'RR_ABORT_STOP' and
                     terminal_hold >= CFG.minimum_stopped_hold_s)
    max_slew = max((abs(float(row[f'fact_{c}_n']) - float(previous[f'fact_{c}_n'])) /
                    (float(row['time_s']) - float(previous['time_s']))
                   for previous, row in zip(rows, rows[1:]) if fault_time_s is not None and float(row['time_s']) >= fault_time_s
                   and float(row['time_s']) > float(previous['time_s'])
                   for c in ('fl','fr','rl','rr')), default=0.)
    max_failure = max((float(r.get('support_qp_failure_s', 0.)) for r in swing), default=0.)
    native_slew = result.get('support_allocation', {}).get('abort_max_native_output_slew_n_s')
    origin = result.get('path_reference_yo_m', float(rows[0]['yo_m']) if rows else 0.)
    lateral = max((abs(float(r['yo_m']) - origin) for r in post), default=None)
    criteria = {
        'injected_inside_pit': bool(abort and start < float(abort[0]['x_rr_m']) < end),
        'task_remains_fail': result['status'] == 'FAIL' and result['rr_abort_reason'] == 'injected RR over-pit abort',
        'support_positive': bool(post and min(float(r['min_support_n']) for r in post) >= cfg['support_floor_n']),
        'zmp_safe': bool(swing and min(float(r['zmp_lambda_min']) for r in swing) >= cfg['lambda_safe']),
        'com_safe': bool(swing and min(float(r['com_lambda_min']) for r in swing) >= cfg['lambda_safe']),
        'clearance_positive': bool(swing and min(float(r['rr_clearance_m']) for r in swing) >= cfg['clearance_m']),
        'rr_unloaded': bool(swing and max(float(r['fz_rr_filtered_n']) for r in swing) <= cfg['wheel_unloaded_n']),
        'attitude_safe': bool(post and max(abs(float(r[k])) for r in post for k in ('roll_deg','pitch_deg')) <= cfg['attitude_limit_deg']),
        'no_lowering_over_pit': all(solid(float(r['x_rr_m'])) for r in lowering),
        'stopped_before_lowering': not lowering or abs(float(lowering[0]['vx_kph'])) <= cfg['stationary_kph'],
        'travel_safe': bool(post and min(float(r[f'travel_{c}_mm']) for r in post for c in ('fl','fr','rl','rr')) >= cfg['travel_rebound_abort_mm'] and
                            max(float(r[f'travel_{c}_mm']) for r in post for c in ('fl','fr','rl','rr')) <= cfg['travel_jounce_abort_mm']),
        'force_within_sim_limits': bool(post and max(abs(float(r[f'fact_{c}_n'])) for r in post for c in ('fl','fr','rl','rr')) <= cfg['sim_force_limit_n']),
        'force_rate_20ms_average_within_sim_limits': max_slew <= cfg['sim_force_slew_n_s'],
        'force_rate_native_within_sim_limits': native_slew is not None and native_slew <= cfg['sim_force_slew_n_s'],
        'no_persistent_qp_failure': max_failure <= SUPPORT_QP.max_continuous_failure_s,
        'recovery_straight': lateral is not None and lateral <= cfg['max_lateral_error_m'],
        'stopped': bool(stopped),
        'native_completed': native_complete,
        'recovered_or_stable_hold': recovered or safe_hold,
    }
    i = next((i for i,r in enumerate(rows) if r['mode'] == 'RR_ABORT_STOP'), None)
    jump = max(abs(float(rows[i][f'fact_{c}_n']) - float(rows[i-1][f'fact_{c}_n']))
               for c in ('fl','fr','rl','rr')) if i is not None and i > 0 else None
    return dict(response_status=('RECOVERED' if recovered else 'SAFE_HOLD') if all(criteria.values()) else 'FAIL',
                task_status=result['status'], criteria=criteria, trial_config=vars(CFG),
                fault_time_s=fault_time_s, abort_start_s=float(abort[0]['time_s']) if abort else None,
                abort_handoff_force_step_n=jump, stopped_hold_s=stopped_hold,
                terminal_stopped_hold_s=terminal_hold, bounded_observation_completed=bounded_observation,
                min_abort_support_n=min((float(r['min_support_n']) for r in swing), default=None),
                min_abort_zmp_lambda=min((float(r['zmp_lambda_min']) for r in swing), default=None),
                min_abort_clearance_m=min((float(r['rr_clearance_m']) for r in swing), default=None),
                max_abort_roll_deg=max((abs(float(r['roll_deg'])) for r in swing), default=None),
                max_recovery_roll_deg=max((abs(float(r['roll_deg'])) for r in post), default=None),
                recovery_max_lateral_m=lateral,
                max_abort_force_slew_20ms_n_s=max_slew,
                max_abort_force_slew_native_n_s=native_slew, max_abort_qp_failure_s=max_failure,
                exit_samples=sum(r['mode']=='RR_ABORT_EXIT' for r in rows),
                final_mode=rows[-1]['mode'] if rows else None,
                scope='one geometry-triggered abort; not general over-pit recovery or support-loss certification')


def main():
    global CFG
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--abort-trigger-fraction',type=float,default=CFG.trigger_fraction)
    trial,rest=parser.parse_known_args(sys.argv[1:])
    CFG=replace(CFG,trigger_fraction=trial.abort_trigger_fraction,
        max_observe_after_abort_s=(CFG.max_recovery_observe_s if '--rear-abort-exit' in rest else CFG.max_observe_after_abort_s))
    sys.argv=[sys.argv[0],*rest]
    if '--output' not in sys.argv:
        raise ValueError('supply --output and the normal full-cycle configuration')
    output = Path(sys.argv[sys.argv.index('--output') + 1]).resolve()
    original_controller, original_step = cycle.FullRightSideController, cycle.run_stepwise
    def bounded_run(*args, **kwargs):
        original_stop = kwargs.get('stop_when')
        def stop(now):
            active = PitAbortController.active
            return bool((original_stop and original_stop(now)) or
                        (active and active.fault_time_s is not None and
                         now - active.fault_time_s >= CFG.max_observe_after_abort_s))
        kwargs['stop_when'] = stop
        return original_step(*args, **kwargs)
    cycle.FullRightSideController, cycle.run_stepwise = PitAbortController, bounded_run
    try:
        cycle.main()
    finally:
        cycle.FullRightSideController, cycle.run_stepwise = original_controller, original_step
    report = audit(output, PitAbortController.active.fault_time_s)
    (output / 'pit_abort_report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
