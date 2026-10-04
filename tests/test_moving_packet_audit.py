from types import SimpleNamespace
from dataclasses import asdict
from ddevsim.static_wheel_lift.config import ROBUST_REAR_RUN
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import pytest


@pytest.mark.parametrize('unsafe',[None,'roll','support','stale_lower','moving_lower','late_crawl','short_hold','nan','still_blind','gap_unfinished'])
def test_moving_outage_response_checks_whole_postfault_history(unsafe):
    from moving_packet_audit import assess_moving_response
    row=dict(time_s=10.,mode='RR_CRAWL',vx_kph=3.,x_rr_m=101.3,yo_m=0.,
        roll_deg=-7.,pitch_deg=0.,min_support_n=1000.,zmp_lambda_min=.1,
        com_lambda_min=.1,rr_clearance_m=.04,fz_rr_filtered_n=0.,measurement_age_s=.04,
        support_qp_failure_s=0.)
    for c in ('fl','fr','rl','rr'):
        row[f'travel_{c}_mm']=0.;row[f'fact_{c}_n']=0.
    rows=[row,{**row,'time_s':10.14,'mode':'RR_ABORT_STOP','measurement_age_s':.18},
        {**row,'time_s':11.,'mode':'RR_ABORT_STOP','vx_kph':0.},
        {**row,'time_s':35.,'mode':'RR_ABORT_STOP','vx_kph':0.}]
    changes={'roll':('roll_deg',12.),'support':('min_support_n',0.),
        'stale_lower':('mode','RR_LOWERING'),'moving_lower':('mode','RR_LOWERING'),
        'late_crawl':('mode','RR_CRAWL'),'short_hold':('vx_kph',1.),'nan':('pitch_deg',float('nan'))}
    if unsafe in changes:
        i=1 if unsafe=='stale_lower' else -1
        rows[i][changes[unsafe][0]]=changes[unsafe][1]
    if unsafe=='moving_lower': rows[-1].update(vx_kph=.3,x_rr_m=102.4)
    if unsafe=='still_blind':
        for row in rows[2:]: row['measurement_age_s']=1.
    result=dict(status='FAIL',rr_abort_reason='feedback packet stale',path_reference_yo_m=0.,
        controller_config={'rear':asdict(ROBUST_REAR_RUN)},scenario={'start_station_m':101.,'length_m':.8},
        criteria={'native_completed':True,'four_wheel_recovered':False},
        support_allocation={'abort_max_native_output_slew_n_s':0.})
    report=assess_moving_response(rows,result,10.,SimpleNamespace(packet_outage_s=30. if unsafe=='gap_unfinished' else .2))
    assert report['response_status']==('SAFE_HOLD' if unsafe is None else 'FAIL')
