"""Independent native data collection for a classical shallow-pit MPC model gate.

Every invocation owns one run and native DLL process. No expert PASS is emitted.
"""
import argparse
import csv
from dataclasses import asdict
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from ddevsim.cosim import run_stepwise
from ddevsim.native_video import DEFAULT_TRUCKSIM_DATA
from ddevsim.runtime.run_workspace import claim_run,sha256_file,write_run_manifest
from ddevsim.scenarios.pit import load_case
from ddevsim.scenarios.builder import prepare_case
from ddevsim.strategies.shallow_press.validation import ValidationExcitation


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case',choices=['right_track','wide'],required=True)
    p.add_argument('--depth-m',type=float,required=True)
    p.add_argument('--speed-kph',type=float,required=True)
    p.add_argument('--run-id',required=True)
    p.add_argument('--seed',type=int,default=1)
    p.add_argument('--excite',action='store_true')
    p.add_argument('--amplitude-n',type=float,default=350)
    p.add_argument('--duration-s',type=float,default=45)
    p.add_argument('--step-s',type=float,choices=[.0005,.00025,.000125],default=.0005)
    p.add_argument('--linearize-initial',action='store_true')
    a=p.parse_args()
    if not 0<=a.speed_kph<=3 or not 0<a.depth_m<=.06 or not 0<=a.amplitude_n<=1000 or not 10<=a.duration_s<=80:
        p.error('outside declared small-pit research envelope')
    case=load_case(ROOT/('scenarios/shallow_press/'+a.case+'.json'),depth_m=a.depth_m,
                   length_m=.8,width_m=.9 if a.case=='right_track' else 1.8,
                   approach_distance_m=2.,friction=.7,stop_s=a.duration_s)
    run=claim_run(ROOT/'runs','shallow_press',a.case,a.run_id)
    manifest=prepare_case(case,run)
    manifest['diagnostic']='shallow_mpc_model_validation'
    manifest['diagnostic_options']=vars(a)
    manifest['code_sha256']={str(f.relative_to(ROOT)):sha256_file(f) for f in
        (Path(__file__).resolve(),ROOT/'src/ddevsim/strategies/shallow_press/validation.py',
         ROOT/'src/ddevsim/strategies/shallow_press/prediction.py')}
    model=run/'model/run_all.par'
    txt=model.read_text(encoding='utf-8')
    if a.step_s!=.0005:
        txt,n=re.subn(r'(?m)^tstep\s+[-+0-9.eE]+$', 'tstep '+str(a.step_s),txt)
        if n!=1:
            raise ValueError('expected one native tstep')
    if a.linearize_initial:
        # Commands after the final END are ignored by the native parsfile reader.
        body,ending=txt.rsplit('\nEND',1)
        commands='\nOPT_LINEARIZATION 0\n'
        commands+='\n'.join('LINEAR_CONTROL '+n for n in manifest['import_names'])+'\n'
        commands+='\n'.join('LINEAR_OUTPUT EXP_'+n for n in manifest['export_names'])+'\n'
        # 2019 event syntax treats ':' as a delimiter even inside quotes.
        # Its relative files resolve against the installed database directory.
        relative_stage=Path('tmp_mpc_validation')/a.run_id
        native_stage=DEFAULT_TRUCKSIM_DATA/relative_stage
        native_stage.mkdir(parents=True,exist_ok=False)
        native_matrix=run/'model/output/initial_linearization.m'
        native_event=native_stage/'event.par'
        commands+='DEFINE_EVENT T > 5 ; '+str(relative_stage/'event.par').replace('/','\\')+'\n'
        txt=body+commands+'END'+ending
        # Matrix writing uses the process workdir (the isolated model), unlike
        # event loading which uses the database root.
        event_text='PARSFILE\nLINEARIZE output\\initial_linearization.m\nEND\n'
        native_event.write_text(event_text,encoding='ascii')
        (run/'model/linearization_event.par').write_text(event_text,encoding='ascii')
    model.write_text(txt,encoding='utf-8')
    manifest['final_model_sha256']=sha256_file(model)
    manifest['actual_step_s']=a.step_s
    manifest['run_status']='DIAGNOSTIC_RUNNING'
    write_run_manifest(run,manifest)
    controller=ValidationExcitation(manifest['export_names'],speed_kph=a.speed_kph,
        seed=a.seed,excite=a.excite,end_station_m=case.geometry.trailing_edge_m+.263+.8,
        step_s=a.step_s,amplitude_n=a.amplitude_n)
    try:
        native=run_stepwise(run/'model/simfile.sim',controller,run/'native_1ms.csv',
                           manifest['import_names'],manifest['export_names'],
                           log_decimation=int(round(.001/a.step_s)))
        if native['status']!='COMPLETED' or native['error_message'] or not native['reached_configured_stop']:
            raise RuntimeError('native run did not complete: '+str(native))
    except Exception as exc:
        manifest.update(run_status='DIAGNOSTIC_FAILED',error=str(exc),acceptance_pass=False)
        write_run_manifest(run,manifest)
        raise
    trace=run/'control_intervals.csv'
    with trace.open('w',newline='',encoding='utf-8') as f:
        writer=csv.DictWriter(f,fieldnames=list(controller.records[0]))
        writer.writeheader()
        writer.writerows(controller.records)
    result=dict(scope='PREDICTOR_VALIDATION_DATA_ONLY',task_status='UNASSESSED',
                acceptance_pass=False,native=native,rear_cleared=controller.completed,
                attitude_20deg_recorded=controller.attitude_exceeded,
                control_trace=str(trace),trace_sha256=sha256_file(trace),
                final_model_sha256=sha256_file(model),scene=asdict(case.geometry))
    (run/'prediction_diagnostic.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    manifest.update(run_status='DIAGNOSTIC_COMPLETED',task_status='UNASSESSED',acceptance_pass=False)
    write_run_manifest(run,manifest)
    print(json.dumps({'run':str(run),'duration_s':native['final_time_s'],'rear_cleared':controller.completed,
                      'linearization_exists':(run/'model/output/initial_linearization.m').exists()}))


if __name__=='__main__':
    main()
