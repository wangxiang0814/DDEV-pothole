"""Current-MF52 bounded stationary extension diagnostic, never expert acceptance."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from ddevsim.cosim import run_stepwise
from ddevsim.runtime.run_workspace import claim_run,sha256_file,write_run_manifest
from ddevsim.scenarios.pit import load_case
from ddevsim.scenarios.builder import prepare_case
from ddevsim.strategies.shallow_press.identification import RateBoundedStrokeProbe,extension_headroom
from ddevsim.strategies.shallow_press.tire_comparison import make_mf52_tir,install_mf52_comparison,research_baseline_metadata
from ddevsim.strategies.shallow_press.assessment import read_native_log,input_screen
from ddevsim.solver_api import parse_simfile
from ddevsim.vehicle_profiles.tuned_ii import vehicle_identity
from run_shallow_preview import freeze_run_inputs
import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id',required=True)
    a=parser.parse_args()
    case=load_case(ROOT/'scenarios/shallow_press/right_track.json',depth_m=.10,length_m=.8,width_m=.9,stop_s=45)
    run=claim_run(ROOT/'runs','shallow_press','right_track',a.run_id)
    manifest=prepare_case(case,run);model=run/'model/run_all.par'
    source=model.read_text(encoding='utf-8');tir,record=make_mf52_tir(source)
    (run/'model/tire_comparison.tir').write_text(tir,encoding='ascii')
    stage=Path(tempfile.gettempdir())/'DDEV_tire_comparison'/hashlib.sha256(str(run).encode('utf-8')).hexdigest()[:16]
    stage.mkdir(parents=True,exist_ok=False);tire=stage/'tire_comparison.tir';tire.write_text(tir,encoding='ascii')
    program=parse_simfile(run/'model/simfile.sim')['PROGDIR'].rstrip('\\/')
    model.write_text(install_mf52_comparison(source,tire,program),encoding='utf-8')
    manifest['comparison_vehicle']={**research_baseline_metadata(record),'parent':case.vehicle_profile_id,
        'vehicle_sha256':vehicle_identity(model),'tire_sha256':sha256_file(tire),'ascii_stage':str(tire)}
    sources=[Path(__file__).resolve(),ROOT/'scripts/run_shallow_preview.py']
    sources.extend(sorted((ROOT/'src/ddevsim/strategies/shallow_press').glob('*.py')))
    sources.extend(ROOT/'src/ddevsim'/n for n in ('units.py','cosim.py','solver_api.py','pothole_case.py',
        'visual_mesh.py','chassis_contact.py','scenarios/pit.py','scenarios/builder.py','scenarios/parameters.py',
        'vehicle_profiles/tuned_ii.py','vehicle_profiles/transformations.py','runtime/contracts.py','runtime/run_workspace.py'))
    hashes={str(p.relative_to(ROOT)):sha256_file(p) for p in sources}
    for p in sources:
        target=run/'source'/p.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(p.read_bytes())
    manifest.update(run_status='RUNNING',route='STATIONARY_EXTENSION_DIAGNOSTIC',effective_config=dict(
        force_rate_n_s=1000.,peak_common_force_n=4000.,extension_guard_m=-.125,
        static_hold_brake_mpa=1.,native_stop_s=45.),diagnostic_options=vars(a),source_sha256=hashes,
        final_model_sha256=sha256_file(model))
    plant=freeze_run_inputs(run,manifest);write_run_manifest(run,manifest)
    probe=RateBoundedStrokeProbe();index={n:i for i,n in enumerate(manifest['export_names'])}
    command=np.zeros(9);wheels=('L1','R1','L2','R2')
    def callback(t,exports):
        # Continuous rate-bounded native commands, rather than 20ms stairs.
        j=np.array([exports[index['Jnc_'+w]] for w in wheels])*.001
        load=np.array([exports[index['Fz_'+w]] for w in wheels])
        command[4:8]=probe.step(t,j,load,np.deg2rad(exports[index['Roll_E']]),np.deg2rad(exports[index['Pitch']]))
        return tuple(command)
    try:
        native=run_stepwise(run/'model/simfile.sim',callback,run/'native_1ms.csv',
            manifest['import_names'],manifest['export_names'],log_decimation=2)
        if native['error_message'] or not native['reached_configured_stop']:raise RuntimeError(str(native))
        values=read_native_log(run/'native_1ms.csv');t=values['time_s'];static=(t>=2.5)&(t<3)
        j=np.column_stack([values['exp_Jnc_'+w] for w in wheels])*.001
        u=np.column_stack([values['imp_IMP_FS_'+w] for w in wheels]);criteria,metrics=input_screen(values)
        result=dict(status='STATIONARY_EXTENSION_DIAGNOSTIC_ONLY',simulation_pass=False,acceptance_pass=False,
            strategy_id='shallow_press',native=native,plant=plant,scene=asdict(case.geometry),
            guard_reason=probe.guard_reason,headroom=extension_headroom(np.median(j[static],axis=0)),
            jounce_min_m=j.min(axis=0).tolist(),peak_command_n=np.max(abs(u),axis=0).tolist(),
            input_criteria=criteria,input_metrics=metrics,physical_hardware_validated=False,
            input_provenance_sha256=manifest['input_provenance_sha256'],native_csv_sha256=sha256_file(run/'native_1ms.csv'),
            model_sha256=sha256_file(model))
        (run/'result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
        manifest.update(run_status='COMPLETED',task_status=result['status'],result_sha256=sha256_file(run/'result.json'))
        print(json.dumps(result))
    except Exception as exc:
        manifest.update(run_status='ERROR',task_status='ERROR',error=repr(exc));raise
    finally:write_run_manifest(run,manifest)


if __name__=='__main__':main()
