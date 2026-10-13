"""Collect a full shallow-only native physical prediction holdout in one process."""
import argparse
import csv
from dataclasses import asdict
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from ddevsim.cosim import run_stepwise
from ddevsim.native_video import DEFAULT_TRUCKSIM_DATA
from ddevsim.solver_api import parse_simfile
from ddevsim.runtime.run_workspace import claim_run,sha256_file,write_run_manifest
from ddevsim.scenarios.pit import load_case
from ddevsim.scenarios.builder import prepare_case
from ddevsim.strategies.shallow_press.identification import parse_native_linearization
from ddevsim.strategies.shallow_press.native_physical_probe import instrument_model,NativePhysicalProbe


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case',choices=['right_track','wide'],required=True)
    p.add_argument('--depth-m',type=float,required=True)
    p.add_argument('--speed-kph',type=float,required=True)
    p.add_argument('--run-id',required=True)
    p.add_argument('--flat-reference',action='store_true')
    p.add_argument('--linearize-at-s',type=float,default=5.)
    a=p.parse_args()
    if not 0<a.depth_m<=.06 or not 0<=a.speed_kph<=3: p.error('small-pit envelope required')
    if a.flat_reference and (a.depth_m>1e-6 or not 5<=a.linearize_at_s<=15):
        p.error('flat references require depth <= 1 micron and extraction 5..15s')
    duration=a.linearize_at_s+5 if a.flat_reference else 45
    case=load_case(ROOT/('scenarios/shallow_press/'+a.case+'.json'),depth_m=a.depth_m,
        length_m=.8,width_m=.9 if a.case=='right_track' else 1.8,
        approach_distance_m=2,friction=.7,stop_s=duration)
    run=claim_run(ROOT/'runs','shallow_press',a.case,a.run_id)
    manifest=prepare_case(case,run)
    model=run/'model/run_all.par'; sim=run/'model/simfile.sim'
    txt,simtxt,road_names=instrument_model(model.read_text(encoding='utf-8'),
                                         sim.read_text(encoding='utf-8'),manifest['export_names'])
    if a.flat_reference:
        relative=Path('tmp_mpc_physical')/a.run_id
        stage=DEFAULT_TRUCKSIM_DATA/relative
        stage.mkdir(parents=True,exist_ok=False)
        event='PARSFILE\nLINEARIZE output\\physical_linearization.m\nEND\n'
        (stage/'event.par').write_text(event,encoding='ascii')
        (run/'model/linearization_event.par').write_text(event,encoding='ascii')
        txt=txt.rsplit('\nEND',1)[0]+'\nDEFINE_EVENT T > '+str(a.linearize_at_s)+' ; '+str(relative/'event.par').replace('/','\\')+'\nEND\n'
    model.write_text(txt,encoding='utf-8'); sim.write_text(simtxt,encoding='utf-8')
    original=ROOT/'evidence/shallow_mpc_validation_20261013/verified_analysis/native_linearization.m'
    states=parse_native_linearization(original.read_text(encoding='utf-8'))['state_names']
    controller=NativePhysicalProbe(manifest['export_names'],speed_kph=a.speed_kph,
        end_station_m=case.geometry.trailing_edge_m+.263+.8,state_names=states,
        dll_path=parse_simfile(sim)['DLLFILE'],model_dir=run/'model',extract=a.flat_reference,linearize_at_s=a.linearize_at_s)
    code_files=[Path(__file__).resolve()]+list((ROOT/'src/ddevsim/strategies/shallow_press').glob('*prediction.py'))
    code_files += [ROOT/'src/ddevsim/strategies/shallow_press'/n for n in ('native_physical_probe.py','validation.py','identification.py')]
    manifest.update(diagnostic='physical_mpc_probe_v1',diagnostic_options=vars(a),
        instrumentation=dict(version=1,public_control_ports=9,diagnostic_road_ports=list(road_names),
                             all_road_commands_zero=True),
        final_model_sha256=sha256_file(model),final_simfile_sha256=sha256_file(sim),
        code_sha256={str(f.relative_to(ROOT)):sha256_file(f) for f in code_files},run_status='DIAGNOSTIC_RUNNING')
    write_run_manifest(run,manifest)
    try:
        native=run_stepwise(sim,controller,run/'native_1ms.csv',manifest['import_names']+list(road_names),
                            manifest['export_names'],log_decimation=2)
        if native['error_message'] or not native['reached_configured_stop']:
            raise RuntimeError(str(native))
        if a.flat_reference and controller.reference is None:
            raise RuntimeError('flat model extraction missing')
        trace=run/'physical_intervals.csv'
        with trace.open('x',newline='',encoding='utf-8') as f:
            writer=csv.DictWriter(f,fieldnames=list(controller.rows[0]))
            writer.writeheader(); writer.writerows(controller.rows)
        result=dict(task_status='UNASSESSED',acceptance_pass=False,native=native,
            rear_cleared=controller.base.completed,scene=asdict(case.geometry),
            trace_sha256=sha256_file(trace),native_csv_sha256=sha256_file(run/'native_1ms.csv'),
            reference=controller.reference)
        (run/'physical_diagnostic.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        manifest.update(run_status='DIAGNOSTIC_COMPLETED',task_status='UNASSESSED',acceptance_pass=False)
    except Exception as exc:
        manifest.update(run_status='DIAGNOSTIC_FAILED',error=str(exc),acceptance_pass=False)
        raise
    finally: write_run_manifest(run,manifest)
    print(json.dumps(dict(run=str(run),duration_s=native['final_time_s'],rear_cleared=controller.base.completed)))


if __name__=='__main__': main()
