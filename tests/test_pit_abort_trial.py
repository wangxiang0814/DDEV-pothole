from pathlib import Path
import sys
from types import SimpleNamespace
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from run_pit_abort_trial import inject_over_pit_abort


def test_abort_trigger_requires_crawl_and_midpit_geometry():
    events = []
    rear = SimpleNamespace(mode='RR_HOLD', scenario={'start_station_m':101.,'length_m':.8},
                           abort_reason=None, _enter=lambda m,t: events.append((m,t)))
    assert not inject_over_pit_abort(rear, 10., 101.5)
    rear.mode = 'RR_CRAWL'
    assert not inject_over_pit_abort(rear, 10., 101.2)
    assert not inject_over_pit_abort(rear, 10., 102.)
    assert inject_over_pit_abort(rear, 10., 101.5)
    assert rear.abort_reason == 'injected RR over-pit abort'
    assert events == [('RR_ABORT_STOP',10.)]

import csv
import json
import pytest
from dataclasses import asdict
from ddevsim.static_wheel_lift.config import REAR_CYCLE_RUN
from run_pit_abort_trial import audit


@pytest.mark.parametrize('final_mode,native_complete,final_speed,qp_failure,expected', [
    ('RR_LOWERING', True, 0., 0., 'FAIL'),
    ('RR_ABORT_STOP', False, 0., 0., 'FAIL'),
    ('RR_ABORT_STOP', True, .2, 0., 'FAIL'),
    ('RR_ABORT_STOP', True, 0., .6, 'FAIL'),
    ('RR_ABORT_STOP', True, 0., 0., 'SAFE_HOLD')])
def test_safe_hold_requires_complete_stopped_observation_and_healthy_qp(
        tmp_path, final_mode, native_complete, final_speed, qp_failure, expected):
    base = dict(mode='RR_ABORT_STOP', time_s=10., vx_kph=0., x_rr_m=101.5,
        yo_m=0.,
        min_support_n=1500., zmp_lambda_min=.1, com_lambda_min=.1,
        rr_clearance_m=.05, fz_rr_filtered_n=0., roll_deg=-7., pitch_deg=0.,
        support_qp_failure_s=qp_failure)
    for c in ('fl','fr','rl','rr'):
        base[f'travel_{c}_mm'] = 0.
        base[f'fact_{c}_n'] = 0.
    rows = [base, {**base,'time_s':16.}, {**base, 'time_s':35.,
        'mode':final_mode, 'vx_kph':final_speed,
        'x_rr_m':102.2 if final_mode=='RR_LOWERING' else 101.5}]
    with (tmp_path / 'rear_control_20ms.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=base.keys());writer.writeheader();writer.writerows(rows)
    result = dict(status='FAIL', rr_abort_reason='injected RR over-pit abort',
        support_allocation={'abort_max_native_output_slew_n_s': 0.},
        criteria={'four_wheel_recovered':False,'native_completed':native_complete},
        scenario={'start_station_m':101.,'length_m':.8},
        controller_config={'rear':asdict(REAR_CYCLE_RUN)})
    (tmp_path/'result.json').write_text(json.dumps(result))
    assert audit(tmp_path, 10.)['response_status'] == expected
