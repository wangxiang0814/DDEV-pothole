"""Read-only native evidence verification and versioned event reassessment.

Run from repository root. Historical result/manifest/source/native files are
never edited. --partial skips cohorts with any unfinished member.
"""
import argparse
import csv
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import zipfile

ROOT=Path.cwd().resolve()
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts')]
from analyze_shallow_preview import verified_run,sha
from ddevsim.strategies.shallow_press.config import ShallowPressConfig
from ddevsim.strategies.shallow_press.event_assessment import event_checks
import numpy as np


def read_event_columns(path):
    # Physical criteria are reused only behind exact source-hash checks below.
    # Reassessment needs raw time, speed and actual wheel positions, not the
    # other ~120 native channels. Round-trip conversion preserves CSV doubles.
    import pandas as pd
    columns=['time_s','exp_Vx']+['exp_'+a+'_'+w for a in ('X','Y') for w in ('L1','R1','L2','R2')]
    return pd.read_csv(path,usecols=columns,float_precision='round_trip').to_records(index=False)


def cohort_ready(pp):
    plan=json.loads(pp.read_text(encoding='utf-8'))
    for row in plan['cases']:
        run=ROOT/'runs/shallow_press'/row['case']/row['run_id']
        if not all((run/name).is_file() for name in ('result.json','matrix_identity.json','run_manifest.json')):return False
    return True


def main():
    p=argparse.ArgumentParser();p.add_argument('--partial',action='store_true');p.add_argument('--wait',action='store_true');a=p.parse_args()
    base=ROOT/'evidence/shallow_event_crawl_20261026';out=base/'source_archives';out.mkdir(exist_ok=True)
    h=lambda b:hashlib.sha256(b).hexdigest()
    files=[Path(__file__).resolve(),ROOT/'scripts/analyze_shallow_preview.py']
    files += [ROOT/'src/ddevsim'/n for n in ('units.py','chassis_contact.py','strategies/shallow_press/event_assessment.py','strategies/shallow_press/event_phases.py',
        'strategies/shallow_press/event_config.py','strategies/shallow_press/assessment.py','strategies/shallow_press/config.py',
        'strategies/shallow_press/controller.py','strategies/shallow_press/preview.py','strategies/shallow_press/traction.py','strategies/shallow_press/body_control.py')]
    analysis_bytes={path:path.read_bytes() for path in files}
    archives={};bindings=[];rows=[];pending=[];cohort_hashes={}
    plans=sorted(base.glob('*/frozen_plan.json'),key=lambda pp:(not cohort_ready(pp),pp.parent.name))
    for pp in plans:
        plan=json.loads(pp.read_text(encoding='utf-8'))
        if not cohort_ready(pp) and a.wait:
            print('Waiting for complete immutable cohort: '+pp.parent.name,flush=True)
            while not cohort_ready(pp):time.sleep(5.)
        if not cohort_ready(pp):
            pending.append(pp.parent.name)
            if a.partial:continue
            raise ValueError('unfinished cohort '+str(pp))
        checked=subprocess.run([sys.executable,str(ROOT/'scripts/analyze_shallow_preview.py'),'--evidence',str(pp.parent)],capture_output=True)
        if checked.returncode:raise RuntimeError(checked.stderr.decode('utf-8',errors='replace'))
        cohort_hashes[str(pp.relative_to(ROOT))]=sha(pp)
        for case in plan['cases']:
            run=ROOT/'runs/shallow_press'/case['case']/case['run_id'];result,manifest=verified_run(run)
            sources={n.replace('\\','/'):v for n,v in manifest['source_sha256'].items()}
            identity=h(json.dumps(sources,sort_keys=True).encode('utf-8'));zp=out/(identity[:16]+'.zip')
            if identity not in archives:
                content={n:(run/'source'/n).read_bytes() for n in sources}
                if any(h(content[n])!=v for n,v in sources.items()):raise ValueError('source drift')
                if not zp.exists():
                    with zipfile.ZipFile(zp,'w',compression=zipfile.ZIP_DEFLATED) as z:
                        for n,b in sorted(content.items()):
                            info=zipfile.ZipInfo(n,date_time=(2026,10,27,0,0,0));info.compress_type=zipfile.ZIP_DEFLATED;z.writestr(info,b)
                with zipfile.ZipFile(zp) as z:
                    if any(h(z.read(n))!=v for n,v in sources.items()):raise ValueError('archive drift')
                archives[identity]=dict(path=str(zp.relative_to(ROOT)),sha256=sha(zp),source_sha256=sources)
            bindings.append(dict(cohort=str(pp.relative_to(ROOT)),run_id=case['run_id'],case=case['case'],
                source_identity=identity,manifest_sha256=sha(run/'run_manifest.json'),result_sha256=sha(run/'result.json')))
            row=dict(cohort=pp.parent.name,case=case['case'],run_id=case['run_id'],depth_m=result['scene']['depth_m'],
                requested_approach_kph=result['config']['cruise_kph'],original_status=result['status'],
                original_revision=result.get('assessment_revision','historical'),original_result_sha256=sha(run/'result.json'),
                native_csv_sha256=result['native_csv_sha256'],plant=result['plant'],capability=result['capability'],
                native_duration_s=result['metrics']['observed_duration_s'],metrics=result['metrics'],
                model_diagnostic=result['status']=='MODEL_DIAGNOSTIC_ONLY',
                legacy_regression=result.get('task_version')!='event_crawl_v1')
            if not row['legacy_regression']:
                data=read_event_columns(run/'native_1ms.csv');diagnostic_path=run/'control_diagnostics.json'
                diagnostics=json.loads(diagnostic_path.read_text(encoding='utf-8'))
                effective=dict(result['config']);event=dict(effective.get('event',{}));retired={}
                if 'traction_utilization' in effective:retired['controller.traction_utilization']=effective.pop('traction_utilization')
                for key in ('rotation_assist_gain','rotation_assist_limit_nm','rotation_assist_lag_m','grounded_braking','hub_motion_feedback'):
                    if key in event:retired[key]=event.pop(key)
                effective['event']=event;config=ShallowPressConfig(**effective)
                # Reuse physical checks only when their exact implementation
                # matches the immutable original run, avoiding repeated costly
                # chassis-clearance sweeps. New event checks read raw data.
                for key in ('src/ddevsim/strategies/shallow_press/assessment.py','src/ddevsim/units.py','src/ddevsim/chassis_contact.py'):
                    if sources[key]!=sha(ROOT/key):raise ValueError('physical assessment changed; full recomputation required')
                physical={k:v for k,v in result['legacy_assessment']['criteria'].items() if k not in ('speed_tracking','no_exit_pause')}
                ec,em=event_checks(data,result['scene'],result['controller'],config,diagnostics,case_id=case['case'])
                physical.update(ec)
                if 'contact_callback' in result:physical['contact_callback_interface']=result['contact_callback']['interface_valid']
                row.update(reassessment_revision='raw_events_reference_v3',reassessment_criteria=physical,
                    reassessment_failed=[k for k,v in physical.items() if not v],
                    reassessment_metrics=dict(result['metrics'],**em),diagnostics_sha256=sha(diagnostic_path),
                    diagnostics_scope='ORIGINAL_BOUND' if result.get('control_diagnostics_sha256') else 'HASHED_AT_REASSESSMENT_ONLY_NOT_ORIGINAL_BINDING',
                    reassessment_config=asdict(config),
                    retired_control_options_not_used_by_event_checks=retired,
                    simulation_pass=bool(all(physical.values()) and not row['model_diagnostic'] and not result['capability']['diagnostic']))
                before=(np.maximum(data['exp_X_L1'],data['exp_X_R1'])>=result['scene']['start_station_m']-.5)
                first=np.flatnonzero(before)
                prefix=np.arange(len(data))<(first[0] if len(first) else len(data))
                prefix&=data['time_s']>=config.settle_s+config.body_leveling_s
                speed=data['exp_Vx'];peak=float(np.max(speed[prefix])) if prefix.any() else 0.
                row['actual_approach_peak_kph']=peak
                row['approach_peak_within_point15_kph']=abs(peak-config.cruise_kph)<=.15
            else:row['simulation_pass']=False
            rows.append(row)
    if any(path.read_bytes()!=data for path,data in analysis_bytes.items()):raise ValueError('analysis sources changed during verification')
    record=dict(scope='EVENT_DEVELOPMENT_NOT_CONTINUOUS_ROBUSTNESS_OR_REAL_SAFETY',pending=pending,
        full_acceptance=False,cohort_plan_sha256=cohort_hashes,rows=rows)
    (base/'study_summary.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    (base/'source_archive_index.json').write_text(json.dumps(dict(scope='EXACT_LISTED_RUNTIME_SOURCES_NOT_NATIVE_DATA_OR_FULL_REPOSITORY',archives=archives,runs=bindings),indent=2)+'\n',encoding='utf-8')
    headers=['cohort','case','depth_cm','requested_approach_kph','actual_approach_peak_kph','actual_pit_mean_kph','actual_pit_peak_kph',
        'az_rms_m_s2','impact_10ms_m_s2','roll_deg','pitch_deg','loaded_slip_proxy_m_s','peak_load_ratio','max_contact_loss_s','backward_m','simulation_pass','failed','run_id']
    with (base/'metrics.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.writer(f);w.writerow(headers)
        for r in rows:
            m=r['metrics'];w.writerow([r['cohort'],r['case'],r['depth_m']*100,r['requested_approach_kph'],r.get('actual_approach_peak_kph'),
                m['crossing_mean_kph'],m['crossing_max_kph'],m['az_rms_m_s2'],m['az_10ms_peak_m_s2'],m['roll_peak_deg'],m['pitch_peak_deg'],m['loaded_slip_proxy_peak_m_s'],
                max(m['wheel_load_peak_ratio_to_static']),max(m['longest_contact_loss_s']),m['backward_distance_m'],r['simulation_pass'],
                ';'.join(r.get('reassessment_failed',[])),r['run_id']])
    source=base/'analysis_sources';source.mkdir(exist_ok=True)
    source_sha={}
    for path in files:
        name=Path('finalize_event_evidence_20261027.py') if path==Path(__file__).resolve() else path.relative_to(ROOT)
        dest=source/name;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(analysis_bytes[path]);source_sha[str(name)]=sha(dest)
    (source/'sha256.json').write_text(json.dumps(source_sha,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(verified_runs=len(rows),event_pass=sum(r['simulation_pass'] for r in rows),pending=pending)))


if __name__=='__main__':main()
