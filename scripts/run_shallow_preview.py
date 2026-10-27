"""Native expert-two preview/feedforward/feedback worker; one process per run."""
import argparse
import csv
from dataclasses import asdict,replace
import json
from pathlib import Path
import re
import sys
import tempfile
import hashlib
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from ddevsim.cosim import run_stepwise
from ddevsim.runtime.contracts import StrategyContext
from ddevsim.runtime.run_workspace import claim_run,sha256_file,write_run_manifest
from ddevsim.scenarios.pit import load_case,validate_case
from ddevsim.scenarios.builder import prepare_case
from ddevsim.strategies.shallow_press.config import ShallowPressConfig
from ddevsim.strategies.shallow_press.controller import ShallowPressStrategy
from ddevsim.strategies.shallow_press.native import NativePreviewBridge,release_constant_brake,contact_patch_comparison,low_speed_comparison
from ddevsim.strategies.shallow_press.assessment import assess,read_native_log,capability_diagnostic,model_diagnostic_result


def freeze_run_inputs(run,manifest):
    """Refresh generated hashes and make an immutable, non-circular identity."""
    manifest['prepared_assets']=dict(manifest['generated_assets'])
    names=list(manifest['generated_assets'])
    if (run/'model/tire_comparison.tir').exists():names.append('model/tire_comparison.tir')
    manifest['generated_assets']={name:sha256_file(run/name) for name in names}
    parent=manifest['vehicle']
    comparison=manifest.get('comparison_vehicle')
    recipe=dict(parent_vehicle_sha256=parent['vehicle_sha256'],
        comparison=None if comparison is None else {k:v for k,v in comparison.items() if k not in ('ascii_stage','vehicle_sha256')})
    plant=dict(parent_profile_id=parent['profile_id'],parent_vehicle_sha256=parent['vehicle_sha256'],
        same_vehicle_as_parent=comparison is None,comparison=comparison,
        recipe_sha256=hashlib.sha256(json.dumps(recipe,sort_keys=True).encode('utf-8')).hexdigest())
    manifest['plant']=plant
    provenance={k:manifest[k] for k in ('plant','generated_assets','source_sha256','effective_config','diagnostic_options')}
    if 'capability' in manifest:provenance['capability']=manifest['capability']
    with (run/'input_provenance.json').open('x',encoding='utf-8') as stream:
        json.dump(provenance,stream,ensure_ascii=False,indent=2)
    manifest['input_provenance_sha256']=sha256_file(run/'input_provenance.json')
    return plant


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case',required=True,choices=['right_track','wide'])
    p.add_argument('--depth-m',required=True,type=float)
    p.add_argument('--run-id',required=True)
    p.add_argument('--speed-kph',type=float)
    p.add_argument('--crossing-kph',type=float)
    p.add_argument('--length-m',type=float,default=.8)
    p.add_argument('--width-m',type=float)
    p.add_argument('--approach-distance-m',type=float,default=2)
    p.add_argument('--friction',type=float,default=.7)
    p.add_argument('--duration-s',type=float,default=60)
    p.add_argument('--mode',choices=['full','passive','traction','suspension'])
    p.add_argument('--step-s',type=float,choices=[.0005,.00025,.000125,.0001,.00005,.00002,.00001],default=.0005)
    p.add_argument('--delay-s',type=float,default=0)
    p.add_argument('--motor-delay-s',type=float,default=.002)
    p.add_argument('--contact-length-m',type=float,default=0,
                   help='independent comparison plant only; 0 preserves shared point contact')
    p.add_argument('--tyre-low-speed-m-s',type=float,default=None,
                   help='independent comparison only; SI converted to native km/h')
    p.add_argument('--tire-model',choices=['internal','mf52_compare','mf52_research'],default='mf52_research',
                   help='user-adopted expert-two research baseline; internal retains original plant')
    p.add_argument('--config',type=Path)
    p.add_argument('--mf52-operating-mode',type=int,choices=[3104,3114,3124,3534,3544],default=3114,
                   help='non3114 modes are unadopted model diagnostics, never baseline acceptance')
    p.add_argument('--diagnostic-torque-rate-nm-s',type=float)
    p.add_argument('--model-diagnostic',action='store_true',help='explicit unadopted model comparison; never a baseline pass')
    p.add_argument('--road-callback',choices=['none','passthrough','circle'],default='none',
                   help='unadopted official road-query interface comparison; never baseline acceptance')
    p.add_argument('--probe',action='store_true')
    p.add_argument('--coast-contact-probe',action='store_true',
                   help='diagnostic: remove all commands before entry, never an expert acceptance run')
    p.add_argument('--claimed-output',action='store_true',help=argparse.SUPPRESS)
    p.add_argument('--demonstration-config',type=Path)
    p.add_argument('--dry-run',action='store_true')
    a=p.parse_args(argv)
    if not 0<a.depth_m<=.20 or not 20<=a.duration_s<=180:p.error('invalid research scene envelope')
    if not 0<=a.delay_s<=.08 or not 0<=a.motor_delay_s<=.08 or not 0<=a.contact_length_m<=.3:p.error('invalid delay or comparison geometry')
    if a.tyre_low_speed_m_s is not None and not 0<a.tyre_low_speed_m_s<=2:p.error('invalid tyre low speed')
    if a.tire_model!='internal' and (a.tyre_low_speed_m_s is not None or a.contact_length_m):
        p.error('external comparison defines its own contact/low-speed model; cannot combine internal patches')
    if a.tire_model=='internal' and a.mf52_operating_mode!=3114:p.error('external operating mode needs external tire')
    if a.road_callback!='none' and (a.tire_model!='mf52_research' or a.mf52_operating_mode not in (3104,3114) or a.contact_length_m or a.tyre_low_speed_m_s is not None or a.probe or a.coast_contact_probe):
        p.error('road callback needs isolated MF52-3104/3114 with no other contact modifications or actuator probes')
    capture=None
    if a.demonstration_config:
        from ddevsim.demonstration.cameras import load_config
        capture=load_config(a.demonstration_config)
    config=json.loads(a.config.read_text(encoding='utf-8')) if a.config else {}
    config.update({k:v for k,v in dict(cruise_kph=a.speed_kph,crossing_kph=a.crossing_kph,mode=a.mode,diagnostic_torque_rate_nm_s=a.diagnostic_torque_rate_nm_s).items() if v is not None})
    config=ShallowPressConfig(**config)
    if a.road_callback!='none' and config.diagnostic_torque_rate_nm_s is not None:
        p.error('road callback interface comparisons require the aligned torque capability')
    case=load_case(ROOT/('scenarios/shallow_press/'+a.case+'.json'),depth_m=a.depth_m,
                  length_m=a.length_m,width_m=a.width_m if a.width_m is not None else (.9 if a.case=='right_track' else 1.8),
                  approach_distance_m=2,friction=a.friction,stop_s=a.duration_s)
    geometry=replace(case.geometry,approach_distance_m=a.approach_distance_m,
                     road_lead_in_m=max(15,a.approach_distance_m+3))
    case=replace(case,geometry=geometry,initial_state=dict(speed_mps=0.,
        vehicle_start_station_m=geometry.leading_edge_m-a.approach_distance_m))
    validate_case(case)
    if a.dry_run:
        print(json.dumps(dict(strategy='shallow_press',route='PREVIEW_FEEDFORWARD_FEEDBACK',config=asdict(config),
            scene=asdict(case.geometry),tire_model=a.tire_model,road_callback=a.road_callback,
            body_feedback_delay_s=a.delay_s,motor_feedback_delay_s=a.motor_delay_s)))
        return 0
    if a.claimed_output:
        run=ROOT/'runs/shallow_press'/a.case/a.run_id
        claim=json.loads((run/'run_claim.json').read_text(encoding='utf-8'))
        if (claim['output']!=str(run.resolve()) or claim['strategy_id']!='shallow_press' or
                claim['case_id']!=a.case or claim['run_id']!=a.run_id):raise ValueError('worker output claim mismatch')
        if set(f.name for f in run.iterdir())-{'run_claim.json','run_manifest.json','run.log'}:
            raise FileExistsError('claimed shallow output already used')
    else:run=claim_run(ROOT/'runs','shallow_press',a.case,a.run_id)
    try:return execute_run(a,config,case,run,capture)
    except Exception as exc:
        record=json.loads((run/'run_manifest.json').read_text(encoding='utf-8'))
        record.update(run_status='ERROR',task_status='ERROR',error=repr(exc),acceptance_pass=False)
        write_run_manifest(run,record)
        raise


def execute_run(a,config,case,run,capture):
    manifest=prepare_case(case,run)
    if not a.probe:
        model=run/'model/run_all.par'
        model.write_text(release_constant_brake(model.read_text(encoding='utf-8')),encoding='utf-8')
        from ddevsim.vehicle_profiles.tuned_ii import vehicle_identity
        if vehicle_identity(model)!=manifest['vehicle']['vehicle_sha256']:
            raise ValueError('brake release unexpectedly changed vehicle hardware identity')
        manifest['procedure_brake_release_mpa']=0.
    if a.tire_model in ('mf52_compare','mf52_research'):
        from ddevsim.strategies.shallow_press.tire_comparison import make_mf52_tir,install_mf52_comparison,research_baseline_metadata,operating_mode_metadata
        model=run/'model/run_all.par';source=model.read_text(encoding='utf-8')
        tir,record=make_mf52_tir(source);(run/'model/tire_comparison.tir').write_text(tir,encoding='ascii')
        if a.tire_model=='mf52_research':record=research_baseline_metadata(record)
        record=operating_mode_metadata(record,a.mf52_operating_mode)
        stage=Path(tempfile.gettempdir())/'DDEV_tire_comparison'/hashlib.sha256(str(run).encode('utf-8')).hexdigest()[:16]
        stage.mkdir(parents=True,exist_ok=False);tire=stage/'tire_comparison.tir';tire.write_text(tir,encoding='ascii')
        from ddevsim.solver_api import parse_simfile
        program=parse_simfile(run/'model/simfile.sim')['PROGDIR'].rstrip('\\/')
        model.write_text(install_mf52_comparison(source,tire,program,a.mf52_operating_mode),encoding='utf-8')
        from ddevsim.vehicle_profiles.tuned_ii import vehicle_identity
        manifest['comparison_vehicle']={**record,'parent':case.vehicle_profile_id,
            'vehicle_sha256':vehicle_identity(model),'tire_sha256':sha256_file(tire),'ascii_stage':str(tire)}
    if a.contact_length_m or a.tyre_low_speed_m_s is not None:
        model=run/'model/run_all.par'
        text=contact_patch_comparison(model.read_text(encoding='utf-8'),a.contact_length_m)
        if a.tyre_low_speed_m_s is not None:text=low_speed_comparison(text,a.tyre_low_speed_m_s)
        model.write_text(text,encoding='utf-8')
        from ddevsim.vehicle_profiles.tuned_ii import vehicle_identity
        inherited={k:v for k,v in manifest.get('comparison_vehicle',{}).items() if k not in ('ascii_stage','vehicle_sha256')}
        manifest['comparison_vehicle']=dict(parent=case.vehicle_profile_id,underlying_tire=inherited,
            modification='EXPLICIT_TIRE_COMPARISON',length_m=a.contact_length_m,
            low_speed_m_s=a.tyre_low_speed_m_s,
            vehicle_sha256=vehicle_identity(model),adopted=False,diagnostic_only=True,
            adoption_scope='NOT_ADOPTED_CONTACT_CONFIGURATION_DIAGNOSTIC')
    if a.road_callback!='none':
        record=dict(manifest['comparison_vehicle'])
        for key in ('baseline_id','adoption_date','adoption_basis'):record.pop(key,None)
        record.update(adopted=False,diagnostic_only=True,adoption_scope='UNADOPTED_ROAD_CALLBACK_DIAGNOSTIC',
            baseline_parent_id='expert2_mf52_research_20261024',road_callback=a.road_callback,
            modification='EXTERNAL_ROAD_CALLBACK_'+a.road_callback.upper(),
            callback_contract='VS_STI_GET_ROAD_CONTACT_EXT_FUNC_Y_X_ID_Z_DY_DX_MUY_MUX',
            isotropic_friction_only=True,physical_road_geometry_changed=False)
        manifest['comparison_vehicle']=record
    # Additional telemetry changes no physical vehicle parameter or input port.
    model=run/'model/run_all.par'; text=model.read_text(encoding='utf-8')
    extra=[root+w for root in ('My_Bk_','My_WC_') for w in ('L1','R1','L2','R2')]
    text=text.replace('\nEND\n','\n'+''.join('EXPORT '+n+'\n' for n in extra)+'END\n')
    if any(('EXPORT '+n+'\n') not in text for n in extra):raise ValueError('telemetry insertion failed')
    model.write_text(text,encoding='utf-8'); manifest['export_names']+=extra
    sim=run/'model/simfile.sim'; text=sim.read_text(encoding='utf-8')
    text=re.sub(r'(?m)^PORTS_EXP \d+$','PORTS_EXP '+str(len(manifest['export_names'])),text)
    text=text.replace('contact_mpc_stationary_probe','shallow_preview')
    sim.write_text(text,encoding='utf-8')
    if a.step_s!=.0005:
        path=run/'model/run_all.par'; txt=path.read_text(encoding='utf-8')
        txt,n=re.subn(r'(?m)^tstep\s+[-+0-9.eE]+$','tstep '+str(a.step_s),txt)
        if n!=1:raise ValueError('native step declaration missing')
        path.write_text(txt,encoding='utf-8')
    if config.policy=='event_crawl':
        from ddevsim.strategies.shallow_press.event_controller import EventCrawlStrategy
        strategy=EventCrawlStrategy()
    else:strategy=ShallowPressStrategy()
    strategy.reset(StrategyContext('shallow_press',case.vehicle_profile_id,a.case,.02,
        dict(scenario=asdict(case.geometry),controller=asdict(config)),{}))
    bridge=NativePreviewBridge(strategy,manifest['export_names'],step_s=a.step_s,delay_s=a.delay_s,motor_delay_s=a.motor_delay_s)
    (run/'control_config.json').write_text(json.dumps(asdict(config),indent=2)+'\n',encoding='utf-8')
    sources=[Path(__file__).resolve()]+[ROOT/'src/ddevsim/strategies/shallow_press'/n for n in
        ('preview.py','config.py','controller.py','traction.py','native.py','actuation.py','assessment.py','tire_comparison.py','body_control.py','contact_kinematics.py','suspension_geometry.py','fast_load.py',
         'event_config.py','event_phases.py','event_speed.py','event_controller.py','event_assessment.py','road_callback.py')]
    sources += [ROOT/'src/ddevsim'/n for n in ('units.py','cosim.py','chassis_contact.py')]
    sources += [ROOT/'src/ddevsim'/n for n in ('solver_api.py','pothole_case.py','vehicle_case.py','visual_mesh.py',
        'scenarios/pit.py','scenarios/builder.py','scenarios/parameters.py','vehicle_profiles/tuned_ii.py',
        'vehicle_profiles/transformations.py','runtime/contracts.py','runtime/run_workspace.py')]
    code={str(f.relative_to(ROOT)):sha256_file(f) for f in sources}
    for f in sources:
        dest=run/'source'/f.relative_to(ROOT); dest.parent.mkdir(parents=True,exist_ok=True); dest.write_bytes(f.read_bytes())
    manifest.update(run_status='RUNNING',task_status='UNASSESSED',route='PREVIEW_FEEDFORWARD_FEEDBACK',
        effective_config=asdict(config),capability=config.capability_metadata(),diagnostic_options={**vars(a),'config':str(a.config) if a.config else None,
            'demonstration_config':str(a.demonstration_config) if a.demonstration_config else None},
        source_sha256=code,final_model_sha256=sha256_file(run/'model/run_all.par'))
    plant=freeze_run_inputs(run,manifest)
    write_run_manifest(run,manifest)
    def probe(t,exports):
        u=[0.]*9
        for i in range(4):
            begin=4+5*i
            if begin<=t<begin+1:u[4+i]=400
            if begin+2<=t<begin+3:u[4+i]=-400
        return u
    coast_started=[None]
    callback_road=None
    if a.road_callback!='none':
        from ddevsim.strategies.shallow_press.road_callback import CallbackRoad
        from ddevsim.solver_api import parse_simfile
        libraries=set(re.findall(r'(?m)^EXTERNAL_TIRE_MODULE_DLL (.+)$',(run/'model/run_all.par').read_text(encoding='utf-8')))
        if len(libraries)!=1:raise ValueError('one external tire wrapper required')
        callback_road=CallbackRoad(manifest['export_names'],asdict(case.geometry),
            Path(parse_simfile(run/'model/simfile.sim')['DLLFILE']),Path(next(iter(libraries)).strip()),a.road_callback)
    def command(t,exports):
        if callback_road is not None:
            callback_road.update(t,exports);callback_road.install()
        if a.coast_contact_probe:
            front=max(exports[manifest['export_names'].index('X_'+w)] for w in ('L1','R1'))
            if coast_started[0] is None and front>case.geometry.start_station_m-.5:
                coast_started[0]=t
            if coast_started[0] is not None:return (0.,)*9
        return bridge(t,exports)
    try:
        native=run_stepwise(run/'model/simfile.sim',probe if a.probe else command,run/'native_1ms.csv',
            manifest['import_names'],manifest['export_names'],log_decimation=round(.001/a.step_s),
            demonstration_dir=(run/'demonstration') if capture and capture['enabled'] else None)
        if native['error_message'] or not native['reached_configured_stop']:raise RuntimeError(str(native))
        if a.probe: result=dict(status='PROBE_ONLY',simulation_pass=False,acceptance_pass=False)
        else:
            values=read_native_log(run/'native_1ms.csv')
            if config.policy=='event_crawl':
                from ddevsim.strategies.shallow_press.event_assessment import assess_event
                result=assess_event(values,asdict(case.geometry),strategy.summary(),config,strategy.records,case_id=a.case)
            else:result=assess(values,asdict(case.geometry),strategy.summary(),config.settle_s,config.crossing_kph)
            if config.diagnostic_torque_rate_nm_s is not None:
                result=capability_diagnostic(result,values,config.actuation_torque_rate_nm_s)
            with (run/'control_intervals.csv').open('x',newline='',encoding='utf-8') as f:
                w=csv.DictWriter(f,fieldnames=list(bridge.rows[0])); w.writeheader(); w.writerows(bridge.rows)
            (run/'control_diagnostics.json').write_text(json.dumps(strategy.records,indent=2,allow_nan=False)+'\n',encoding='utf-8')
            if config.policy=='event_crawl':
                digest=sha256_file(run/'control_diagnostics.json')
                result['control_diagnostics_sha256']=digest;manifest['control_diagnostics_sha256']=digest
            if a.coast_contact_probe:
                result.update(status='CONTACT_COAST_PROBE_ONLY',simulation_pass=False,
                              coast_started_s=coast_started[0],acceptance_pass=False)
        if callback_road is not None:
            callback_record=callback_road.write(run);result['contact_callback']=callback_record
            result['criteria']['contact_callback_interface']=callback_record['interface_valid']
            result['simulation_pass']=result['simulation_pass'] and callback_record['interface_valid']
            manifest['contact_callback']=callback_record
        if a.model_diagnostic or a.mf52_operating_mode!=3114 or a.contact_length_m or a.tyre_low_speed_m_s is not None or a.road_callback!='none':
            result=model_diagnostic_result(result)
        result.update(strategy_id='shallow_press',native=native,scene=asdict(case.geometry),config=asdict(config),
            plant=plant,capability=config.capability_metadata(),input_provenance_sha256=manifest['input_provenance_sha256'],
            native_csv_sha256=sha256_file(run/'native_1ms.csv'),model_sha256=manifest['final_model_sha256'])
        (run/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        manifest.update(run_status='COMPLETED',task_status=result['status'],acceptance_pass=False,
            simulation_pass=result['simulation_pass'],result_sha256=sha256_file(run/'result.json'))
    except Exception as exc:
        manifest.update(run_status='ERROR',task_status='ERROR',error=str(exc),acceptance_pass=False)
        raise
    finally:write_run_manifest(run,manifest)
    if capture and capture['enabled']:
        from ddevsim.demonstration.capture import capture_episode
        try:
            collection=capture_episode(run,capture)
            manifest['demonstration']=dict(status=collection['collection_status'],trainable=False,
                episode='demonstration/episode/episode.json')
        except Exception as exc:
            manifest['demonstration']=dict(status='ERROR',trainable=False,error=repr(exc))
        write_run_manifest(run,manifest)
    print(json.dumps(dict(run=str(run),status=result['status'],diagnostic_motion_pass=result.get('diagnostic_motion_pass'),metrics=result.get('metrics'),criteria=result.get('criteria'))))
    return 0


if __name__=='__main__':raise SystemExit(main())
