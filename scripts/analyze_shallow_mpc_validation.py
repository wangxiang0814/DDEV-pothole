"""Freeze a classical predictor on training runs, then evaluate independent runs."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from ddevsim.runtime.run_workspace import sha256_file
from ddevsim.strategies.shallow_press.identification import (
    trace_features,parse_native_linearization,qualify_pit_prediction,OUTPUT_NAMES)
from ddevsim.strategies.shallow_press.prediction import DynamicModel,fit_dynamic_model,rollout,prediction_metrics
from ddevsim.strategies.shallow_press.validation import preview_road

OUT=ROOT/'evidence/shallow_mpc_validation_20261013'


def validate_collection(collection):
    if collection['status']!='COMPLETED' or len(collection['runs'])!=collection['total_planned']:
        raise ValueError('complete matrix required before holdout evaluation')
    if any(r['exit_code']!=0 for r in collection['runs']):
        raise ValueError('native worker failure in matrix')


def preflight_outputs(paths):
    existing=[str(p) for p in paths if p.exists()]
    if existing:
        raise FileExistsError('analysis output already exists: '+existing[0])


def write(name,value,output_dir=OUT):
    path=output_dir/name
    if path.exists():
        raise FileExistsError(str(path))
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')


def read_run(item):
    run=Path(item['run_path'])
    diagnostic=json.loads((run/'prediction_diagnostic.json').read_text(encoding='utf-8'))
    if not diagnostic['native']['reached_configured_stop'] or diagnostic['native']['error_message']:
        raise ValueError('native run did not complete cleanly')
    if sha256_file(Path(diagnostic['control_trace']))!=diagnostic['trace_sha256']:
        raise ValueError('control trace hash mismatch')
    if sha256_file(run/'model/run_all.par')!=diagnostic['final_model_sha256']:
        raise ValueError('native model hash mismatch')
    values=np.genfromtxt(diagnostic['control_trace'],delimiter=',',names=True,encoding='utf-8')
    episode=trace_features(values,diagnostic['scene'])
    episode['scene']=diagnostic['scene']
    episode['result']=diagnostic
    return episode


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--fit-only',action='store_true')
    p.add_argument('--output-dir',type=Path,default=OUT,
                   help='New analysis output folder; source collection/model remain frozen')
    args=p.parse_args()
    output_dir=args.output_dir.resolve()
    output_dir.mkdir(parents=True,exist_ok=True)
    collection=json.loads((OUT/'collection.json').read_text(encoding='utf-8'))
    config=json.loads(Path(collection['matrix']).read_text(encoding='utf-8'))
    if collection['matrix_sha256']!=sha256_file(Path(collection['matrix'])):
        raise ValueError('frozen matrix hash mismatch')
    train=[r for r in collection['runs'] if r['role']=='train']
    if args.fit_only:
        if len(train)!=len(config['training']) or any(r['exit_code']!=0 for r in train):
            raise ValueError('complete training matrix required')
        episodes=[]
        for item in train:
            e=read_run(item)
            keep=e['t']>=5
            episodes.append({key:e[key][keep] for key in ('y','u','road')})
        model=fit_dynamic_model(episodes,ridge=config['predictor']['ridge'])
        write('dynamic_model.json',dict(type='CLASSICAL_ARX2',outputs=list(OUTPUT_NAMES),
              training_runs=[r['run_path'] for r in train],parameters=model.as_dict(),
              matrix_sha256=collection['matrix_sha256'],holdout_used=False,
              native_trace_convention='output at observed_time; input held over following interval'))
        print(json.dumps({'rank':model.feature_rank,'features':model.feature_count,
                          'condition':model.condition_number,'spectral_radius':model.spectral_radius}))
        return
    validate_collection(collection)
    targets=[output_dir/name for name in ('native_linear_model.json','native_linearization.m','prediction_summary.json')]
    targets += [output_dir/('pred_'+r['options']['case']+'_'+r['run_id']+'.npz')
                for r in collection['runs'] if r['role']=='holdout']
    preflight_outputs(targets)
    saved=json.loads((OUT/'dynamic_model.json').read_text(encoding='utf-8'))
    pars=saved['parameters']
    for key in ('coefficients','y_center','y_scale','u_scale','road_scale'):
        pars[key]=np.asarray(pars[key])
    model=DynamicModel(**pars)
    reports=[]
    for item in collection['runs']:
        if item['role']=='train':
            continue
        e=read_run(item)
        scene=e['scene']
        xs=e['xy'][:,:,0]
        front=xs[:,:2].max(axis=1) if item['options']['case']=='wide' else xs[:,1]
        rear=xs[:,2:].min(axis=1) if item['options']['case']=='wide' else xs[:,3]
        mask=(e['t']>=5)&(front>=scene['start_station_m']-.263-.4)&(rear<=scene['start_station_m']+scene['length_m']+.263+.5)
        horizons={}
        for horizon in config['predictor']['horizon_s']:
            count=round(horizon/.02)
            starts=np.where(mask)[0]
            starts=starts[(starts>=1)&(starts+count<len(e['t']))]
            predictions=[]
            actuals=[]
            constant=[]
            lost=[]
            phase=[]
            for k in starts:
                preview=preview_road(e['xy'][k],speed_mps=e['y'][k,16],dt=.02,steps=count,scene=scene)
                predicted=rollout(model,e['y'][k-1:k+1],e['u'][k:k+count],preview)[-1]
                predictions.append(predicted)
                actuals.append(e['y'][k+count])
                constant.append(e['y'][k])
                lost.append(e['contact'][k+count])
                near=np.min(abs(xs[k+count,:,None]-np.array([scene['start_station_m'],scene['start_station_m']+scene['length_m']])[None,:]))
                phase.append(near<=.3)
            if not starts.size:
                raise ValueError('no complete pit prediction windows: '+item['run_path'])
            truth,pred=np.array(actuals),np.array(predictions)
            metrics=prediction_metrics(truth,pred)
            contact=prediction_metrics(truth[:,8:12],pred[:,8:12],contact_truth=np.array(lost))
            edges=np.asarray(phase,dtype=bool)
            edge_metrics=prediction_metrics(truth[edges],pred[edges]) if edges.any() else None
            gate=qualify_pit_prediction(metrics,edge_metrics,
                false_support_samples=contact['false_support_samples'],rear_cleared=e['result']['rear_cleared'])
            horizons[str(horizon)]=dict(metrics=metrics,persistence=prediction_metrics(truth,np.array(constant)),
                contact=contact,gate=gate,edge_metrics=edge_metrics,
                origin_count=len(starts),first_origin_s=float(e['t'][starts[0]]),last_origin_s=float(e['t'][starts[-1]]),
                negative_load_prediction_samples=int(np.sum(pred[:,8:12]<0)))
            if item['role']=='holdout' and horizon==.2:
                with (output_dir/('pred_'+item['options']['case']+'_'+item['run_id']+'.npz')).open('xb') as target:
                    np.savez_compressed(target,time=e['t'][starts+count],truth=truth,prediction=pred,
                                        persistence=np.array(constant),edge_mask=edges)
        reports.append(dict(run_id=item['run_id'],run_path=item['run_path'],role=item['role'],options=item['options'],
            rear_cleared=e['result']['rear_cleared'],horizons=horizons,trace_sha256=e['result']['trace_sha256']))
        print(item['options']['case'],item['run_id'],'200ms',horizons['0.2']['gate'],flush=True)
    # Preserve and inspect the independently extracted native physical matrices.
    native_run=ROOT/'runs/shallow_press/right_track/mpc_native_extract_20261013_007'
    native_path=native_run/'model/output/initial_linearization.m'
    native=parse_native_linearization(native_path.read_text(encoding='utf-8'))
    eig=np.linalg.eigvals(native['A'])
    native_info=dict(run_path=str(native_run),sha256=sha256_file(native_path),
        shapes={key:list(native[key].shape) for key in 'ABCD'},
        finite=all(np.isfinite(native[key]).all() for key in 'ABCD'),
        max_real_eigenvalue=float(np.max(eig.real)),
        note='Native model at settled flat 5.001s; 9 actuator inputs, no road disturbance inputs. Not a global pit predictor.')
    write('native_linear_model.json',{key:(value.tolist() if isinstance(value,np.ndarray) else value) for key,value in native.items()},output_dir)
    with (output_dir/'native_linearization.m').open('xb') as target:
        target.write(native_path.read_bytes())
    summary=dict(scope='PREDICTOR_ROUTE_GATE_NOT_EXPERT_ACCEPTANCE',source_matrix_sha256=collection['matrix_sha256'],
        model_sha256=sha256_file(OUT/'dynamic_model.json'),native_extraction=native_info,
        outputs=list(OUTPUT_NAMES),predictor_training_only=True,cases=reports,
        future_control_is_oracle=True,preview_uses_future_native_pose=False,
        expert_implemented=False,case_gate_at_200ms=[dict(case=r['options']['case'],depth_m=r['options']['depth_m'],
            speed_kph=r['options']['speed_kph'],**r['horizons']['0.2']['gate']) for r in reports if r['role']=='holdout'])
    write('prediction_summary.json',summary,output_dir)


if __name__=='__main__':
    main()
