import csv
import json
import sys
from pathlib import Path
from dataclasses import asdict
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import run_front_recovery_trial as trial
from ddevsim.static_wheel_lift.config import CLOSED_LOOP_RUN


@pytest.mark.parametrize('unsafe', [None,'speed','pit'])
def test_later_motion_or_pit_entry_is_not_hidden_by_initial_stop(tmp_path,monkeypatch,unsafe):
    monkeypatch.setattr(trial,'summarize',lambda run:{'abort_start_s':0.,'criteria':{'baseline_checks':True}})
    (tmp_path/'result.json').write_text(json.dumps({'controller_config':{'front':asdict(CLOSED_LOOP_RUN)},
        'scenario':{'start_station_m':101.1,'length_m':.8}}))
    row={'time_s':0.,'mode':'ABORT_STOP','vx_kph':0.,'x_fr_m':100.,'x_rr_m':97.,
         'roll_deg':0.,'pitch_deg':0.,'zmp_lambda_min':.1,'com_lambda_min':.1,
         **{f'fz_{c}_filtered_n':1000. for c in ('fl','fr','rl','rr')},
         **{f'travel_{c}_mm':0. for c in ('fl','fr','rl','rr')},
         **{f'fact_{c}_n':0. for c in ('fl','fr','rl','rr')}}
    rows=[row,{**row,'time_s':1.,'mode':'LOWERING'},
          {**row,'time_s':2.,'mode':'RETURN_TO_FOUR_WHEEL'}]
    if unsafe=='speed': rows[-1]['vx_kph']=.3
    if unsafe=='pit': rows[-1]['x_fr_m']=101.5
    with (tmp_path/'front_control_20ms.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=row);writer.writeheader();writer.writerows(rows)
    report=trial.audit(tmp_path)
    assert report['recovery_status']==('PASS' if unsafe is None else 'FAIL')
