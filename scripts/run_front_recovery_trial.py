"""Inject a pit-front FR HOLD interruption and audit the recovery separately."""
import csv
import json
from pathlib import Path
import sys
import run_right_side_full_cycle as cycle
from summarize_front_recovery import summarize
from ddevsim.static_wheel_lift.config import FEEDBACK_FAULT_TRIAL, SUPPORT_QP


class FrontTrial(cycle.FullRightSideController):
    def __init__(self,**kwargs):
        super().__init__(**kwargs)
        self.injected=False
    def __call__(self,now_s,exports):
        command=super().__call__(now_s,exports)
        if (not self.injected and self.front.mode=='THREE_WHEEL_HOLD' and
                now_s-self.front.mode_start_s>=FEEDBACK_FAULT_TRIAL.after_hold_s):
            self.front.abort_reason='injected FR pit-front hold interruption'
            self.front._enter('ABORT_STOP',now_s)
            self.injected=True
        return command


def main():
    if '--output' not in sys.argv:
        raise ValueError('output and normal full-cycle options required')
    run=Path(sys.argv[sys.argv.index('--output')+1]).resolve()
    original=cycle.FullRightSideController
    cycle.FullRightSideController=FrontTrial
    try:
        cycle.main()
    finally:
        cycle.FullRightSideController=original
    report=audit(run)
    (run/'front_recovery_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


def audit(run):
    report=summarize(run)
    result=json.loads((run/'result.json').read_text())
    cfg=result['controller_config']['front']
    rows=list(csv.DictReader((run/'front_control_20ms.csv').open()))
    post=[x for x in rows if report['abort_start_s'] is not None and float(x['time_s'])>=report['abort_start_s']]
    swing=[x for x in post if x['mode']=='ABORT_STOP']
    recovery=[x for x in post if x['mode'] in ('LOWERING','RETURN_TO_FOUR_WHEEL','COMPLETE')]
    scene=result['scenario']
    solid=lambda station:(station<=scene['start_station_m']-cfg['crossing_clearance_m'] or
                          station>=scene['start_station_m']+scene['length_m']+cfg['crossing_clearance_m'])
    minimum=min((float(x[f'fz_{c}_filtered_n']) for x in post for c in ('fl','rl','rr')),default=None)
    attitude=max((abs(float(x[k])) for x in post for k in ('roll_deg','pitch_deg')),default=None)
    checks={
        'support_preserved':minimum is not None and minimum>=cfg['support_floor_n'],
        'attitude_safe':attitude is not None and attitude<=cfg['attitude_limit_deg'],
        'swing_triangle_safe':bool(swing) and min(float(x[k]) for x in swing for k in ('zmp_lambda_min','com_lambda_min'))>=cfg['lambda_target'],
        'travel_safe':bool(post) and min(float(x[f'travel_{c}_mm']) for x in post for c in ('fl','fr','rl','rr'))>cfg['moving_rebound_abort_mm'] and max(float(x[f'travel_{c}_mm']) for x in post for c in ('fl','fr','rl','rr'))<cfg['jounce_abort_mm'],
        'force_safe':bool(post) and max(abs(float(x[f'fact_{c}_n'])) for x in post for c in ('fl','fr','rl','rr'))<=SUPPORT_QP.force_limit_n,
        'no_crawl':not any(x['mode']=='CRAWL' for x in post),
        'stationary_throughout_response':bool(post) and max(abs(float(x['vx_kph'])) for x in post)<=cfg['stop_speed_kph'],
        'solid_throughout_recovery':bool(recovery) and all(solid(float(x[k])) for x in recovery for k in ('x_fr_m','x_rr_m')),
    }
    report['criteria'].update(checks)
    report['recovery_status']='PASS' if all(report['criteria'].values()) else 'FAIL'
    report.update(min_support_n=minimum,max_attitude_deg=attitude)
    return report


if __name__=='__main__':
    main()
