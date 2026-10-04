"""RR HOLD faults and geometry-triggered moving packet-outage truth audits."""
import argparse
import csv
import json
from pathlib import Path
import sys
import run_right_side_full_cycle as cycle
import ddevsim.static_wheel_lift.observation_noise as observation
import ddevsim.static_wheel_lift.support_allocation_feedback as allocation
from ddevsim.static_wheel_lift.suspension_allocator import SupportIncrement
from ddevsim.static_wheel_lift.config import FEEDBACK_FAULT_TRIAL


class TrialController(cycle.FullRightSideController):
    active = None
    fault = 'packet'
    after_s = FEEDBACK_FAULT_TRIAL.after_hold_s
    duration_s = FEEDBACK_FAULT_TRIAL.packet_outage_s
    phase = 'RR_HOLD'
    pit_fraction = FEEDBACK_FAULT_TRIAL.moving_pit_fraction
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.fault_time_s = None
        self.now_s = 0.
        TrialController.active = self

    def trigger(self, now_s, truth=None):
        self.now_s = now_s
        at_trigger = (now_s-self.rear.mode_start_s >= self.after_s if self.phase=='RR_HOLD' and self.rear.mode_start_s is not None else
            truth is not None and self.rear.scenario['start_station_m']+self.pit_fraction*self.rear.scenario['length_m'] <= truth['X_R2'] < self.rear.scenario['start_station_m']+self.rear.scenario['length_m'])
        if (self.fault_time_s is None and self.rear_started and
                self.rear.mode == self.phase and at_trigger):
            self.fault_time_s = now_s

    def __call__(self, now_s, exports):
        self.trigger(now_s,dict(zip(cycle.front_run.EXPORTS,exports)))
        return super().__call__(now_s,exports)


def main():
    parser=argparse.ArgumentParser(add_help=False)
    parser.add_argument('--fault',choices=('packet','qp'),required=True)
    parser.add_argument('--fault-after-s',type=float,default=FEEDBACK_FAULT_TRIAL.after_hold_s)
    parser.add_argument('--packet-outage-s',type=float,default=FEEDBACK_FAULT_TRIAL.packet_outage_s)
    parser.add_argument('--fault-phase',choices=('RR_HOLD','RR_CRAWL'),default='RR_HOLD')
    parser.add_argument('--fault-pit-fraction',type=float,default=FEEDBACK_FAULT_TRIAL.moving_pit_fraction)
    args,rest=parser.parse_known_args(sys.argv[1:])
    import math
    if (not all(math.isfinite(v) and v>0 for v in (args.fault_after_s,args.packet_outage_s)) or
            not math.isfinite(args.fault_pit_fraction) or not 0.<args.fault_pit_fraction<1. or
            args.fault_phase=='RR_CRAWL' and args.fault!='packet' or
            '--measurement-noise' not in rest or '--output' not in rest or
            '--support-allocation' not in rest or rest[rest.index('--support-allocation')+1]!='active'):
        raise ValueError('positive finite durations, output, noise wrapper and active support allocation required')
    output=Path(rest[rest.index('--output')+1]).resolve()
    TrialController.fault=args.fault;TrialController.after_s=args.fault_after_s
    TrialController.duration_s=args.packet_outage_s
    TrialController.phase=args.fault_phase;TrialController.pit_fraction=args.fault_pit_fraction
    original_controller=cycle.FullRightSideController
    original_step=cycle.run_stepwise
    original_noise=observation.ObservationNoise
    original_allocate=allocation.allocate_support_increment
    class FrozenNoise(original_noise):
        held=None
        held_time=None
        def observe(self,now_s,truth):
            c=TrialController.active
            if c is not None:
                c.trigger(now_s,truth)
            if c is not None and c.fault=='packet' and c.fault_time_s is not None and now_s-c.fault_time_s < c.duration_s:
                if self.held is None:
                    self.held=super().observe(now_s,truth)
                    self.held_time=self.measurement_time_s
                self.measurement_time_s=self.held_time
                self.measurement_age_s=now_s-self.held_time
                return dict(self.held)
            return super().observe(now_s,truth)
    def failed_qp(**kwargs):
        c=TrialController.active
        if c is not None and c.fault=='qp' and c.fault_time_s is not None:
            return SupportIncrement('INFEASIBLE',None,None,None,None)
        return original_allocate(**kwargs)
    def bounded_run(*args,**kwargs):
        original_stop=kwargs.get('stop_when')
        def stop(now):
            c=TrialController.active
            return bool((original_stop and original_stop(now)) or
                (c is not None and c.phase=='RR_CRAWL' and c.fault_time_s is not None and
                 c.rear.abort_reason is not None and now-c.fault_time_s>=FEEDBACK_FAULT_TRIAL.moving_observe_s))
        kwargs['stop_when']=stop
        return original_step(*args,**kwargs)
    sys.argv=[sys.argv[0],*rest]
    cycle.FullRightSideController=TrialController
    observation.ObservationNoise=FrozenNoise
    allocation.allocate_support_increment=failed_qp
    cycle.run_stepwise=bounded_run
    try:
        cycle.main()
    finally:
        cycle.FullRightSideController=original_controller
        observation.ObservationNoise=original_noise
        allocation.allocate_support_increment=original_allocate
        cycle.run_stepwise=original_step
    if args.fault_phase=='RR_CRAWL':
        from moving_packet_audit import audit_moving_packet
        report=audit_moving_packet(output,TrialController.active.fault_time_s,args)
    else:
        report=audit(output,TrialController.active.fault_time_s,args)
    (output/'feedback_fault_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


def audit(output,fault_time_s,args):
    r=json.loads((output/'result.json').read_text())
    rows=list(csv.DictReader((output/'rear_control_20ms.csv').open()))
    post=[x for x in rows if fault_time_s is not None and float(x['time_s'])>=fault_time_s]
    abort=[x for x in post if x['mode']=='RR_ABORT_STOP']
    stale=[x for x in post if float(x['measurement_age_s'])>cycle.FEEDBACK_HEALTH.max_packet_age_s]
    first_abort=float(abort[0]['time_s']) if abort else None
    recovery=[x for x in post if x['mode'] in ('RR_LOWERING','RR_RETURN','RR_COMPLETE')]
    cfg=r['controller_config']['rear']
    swing=[x for x in post if x['mode']=='RR_ABORT_STOP']
    force_rate=max((abs(float(b[f'fact_{k}_n'])-float(a[f'fact_{k}_n'])) /
        (float(b['time_s'])-float(a['time_s'])) for a,b in zip(post,post[1:])
        if float(b['time_s'])>float(a['time_s']) for k in ('fl','fr','rl','rr')),default=0.)
    checks={
        'fault_injected':fault_time_s is not None,
        'task_remains_fail':r['status']=='FAIL' and r['rr_abort_reason'] in ('feedback packet stale','support allocation persistently infeasible'),
        'abort_detected':bool(abort),
        'stationary_response':bool(post) and max(abs(float(x['vx_kph'])) for x in post)<=cfg['stationary_kph'],
        'support_preserved':bool(post) and min(float(x['min_support_n']) for x in post)>=cfg['support_floor_n'],
        'attitude_safe':bool(post) and max(abs(float(x[k])) for x in post for k in ('roll_deg','pitch_deg'))<=cfg['attitude_limit_deg'],
        'swing_triangle_safe':bool(swing) and min(float(x[k]) for x in swing for k in ('zmp_lambda_min','com_lambda_min'))>=cfg['lambda_safe'],
        'swing_clear_and_unloaded':bool(swing) and min(float(x['rr_clearance_m']) for x in swing)>=cfg['clearance_m'] and max(float(x['fz_rr_filtered_n']) for x in swing)<=cfg['wheel_unloaded_n'],
        'travel_safe':bool(post) and min(float(x[f'travel_{k}_mm']) for x in post for k in ('fl','fr','rl','rr'))>=cfg['travel_rebound_abort_mm'] and max(float(x[f'travel_{k}_mm']) for x in post for k in ('fl','fr','rl','rr'))<=cfg['travel_jounce_abort_mm'],
        'force_and_rate_safe':bool(post) and max(abs(float(x[f'fact_{k}_n'])) for x in post for k in ('fl','fr','rl','rr'))<=cfg['sim_force_limit_n'] and force_rate<=cfg['sim_force_slew_n_s'],
        'native_rate_safe':r['support_allocation']['abort_max_native_output_slew_n_s']<=cfg['sim_force_slew_n_s'],
        'straight_recovery':bool(post) and max(abs(float(x['yo_m'])-r['path_reference_yo_m']) for x in post)<=cfg['max_lateral_error_m'],
        'no_crawl_or_exit':not any(x['mode'] in ('RR_CRAWL','RR_ABORT_EXIT') for x in post),
        'no_lowering_from_stale_packet':bool(stale) and all(x['mode']=='RR_ABORT_STOP' for x in stale) if args.fault=='packet' else True,
        'solid_ground_recovery':bool(recovery) and all(float(x['x_rr_m'])<=r['scenario']['start_station_m']-cfg['crossing_clearance_m'] for x in recovery),
        'four_wheel_recovered':r['criteria']['four_wheel_recovered'],
        'native_completed':r['criteria']['native_completed'],
    }
    report={'response_status':'RECOVERED' if all(checks.values()) else 'FAIL','task_status':r['status'],
        'criteria':checks,'fault':vars(args),'fault_time_s':fault_time_s,'abort_start_s':first_abort,
        'max_packet_age_s':max((float(x['measurement_age_s']) for x in post),default=None),
        'min_support_n':min((float(x['min_support_n']) for x in post),default=None),
        'max_attitude_deg':max((abs(float(x[k])) for x in post for k in ('roll_deg','pitch_deg')),default=None),
        'scope':'RR pit-front stationary HOLD; packet outage recovers or persistent QP failure with healthy sensors. Not moving blind braking or actuator/support failure.'}
    return report


if __name__=='__main__':
    main()
