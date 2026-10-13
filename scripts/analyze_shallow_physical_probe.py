"""Frozen native physical models, independent pit holdouts; no online MPC claim."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import scipy
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from ddevsim.runtime.run_workspace import sha256_file
from ddevsim.strategies.shallow_press.identification import OUTPUT_NAMES,OUTPUT_SI_FACTORS,trace_features,qualify_pit_prediction
from ddevsim.strategies.shallow_press.prediction import prediction_metrics
from ddevsim.strategies.shallow_press.physical_prediction import physical_rollout_batch,preview_physical_road
from ddevsim.strategies.shallow_press.native_physical_probe import validate_reference_matrices


def read(run):
    manifest=json.loads((run/'run_manifest.json').read_text(encoding='utf-8'))
    result=json.loads((run/'physical_diagnostic.json').read_text(encoding='utf-8'))
    if manifest['run_status']!='DIAGNOSTIC_COMPLETED' or not result['native']['reached_configured_stop'] or result['native']['error_message']:
        raise ValueError('incomplete native source '+str(run))
    for file,key,source in [('model/run_all.par','final_model_sha256',manifest),
        ('model/simfile.sim','final_simfile_sha256',manifest),
        ('physical_intervals.csv','trace_sha256',result),('native_1ms.csv','native_csv_sha256',result)]:
        if sha256_file(run/file)!=source[key]: raise ValueError('source hash mismatch '+str(run/file))
    values=np.genfromtxt(run/'physical_intervals.csv',delimiter=',',names=True,encoding='utf-8')
    return manifest,result,values


def freeze_reference(run):
    manifest,result,values=read(run)
    ref=result['reference']; parsed=ref['model']
    validate_reference_matrices(parsed,(run/'model/output/physical_linearization.m').read_text(encoding='utf-8'))
    chosen=[parsed['output_names'].index(n) for n in OUTPUT_NAMES]
    k=np.argmin(abs(values['observed_time_s']-ref['observed_time_s']))
    preserved_state=np.array([values['sv_'+n][k] for n in parsed['state_names']])
    if abs(values['observed_time_s'][k]-ref['observed_time_s'])>1e-9 or not np.allclose(preserved_state,ref['state'],rtol=0,atol=1e-12):
        raise ValueError('reference state diverges from preserved native trace')
    y0=np.array([values['exp_'+n][k] for n in OUTPUT_NAMES])*OUTPUT_SI_FACTORS
    # The state/outputs precede the new command. Bias input is the prior hold.
    u0=np.r_[[values['imp_'+n][max(0,k-1)] for n in parsed['input_names'][:9]],np.zeros(12)]
    u0[8]*=np.pi/180
    model={key:np.asarray(parsed[key],float) for key in 'ABCD'}
    model['C']=model['C'][chosen]; model['D']=model['D'][chosen]
    model.update(f0=np.asarray(ref['f0']),y0=y0,load_rows=list(range(8,12)),height_cols=list(range(9,13)))
    if parsed['input_names'][9:13]!=['IMP_ZGND_'+w+'I' for w in ('L1','R1','L2','R2')]:
        raise ValueError('unexpected native road order')
    return model,np.asarray(ref['state']),u0,parsed['state_names'],dict(
        run_path=str(run),model_sha256=manifest['final_model_sha256'],
        matrix_sha256=sha256_file(run/'model/output/physical_linearization.m'),
        consumed_reference_json_sha256=sha256_file(run/'physical_diagnostic.json'),
        reference_trace_sha256=sha256_file(run/'physical_intervals.csv'),
        drift_source='Causal backward0.5ms native state derivative stored in reference JSON; initial flat reference traces do not separately log derivative columns',
        reference_observed_time_s=ref['observed_time_s'],
        note='Matrix event and reference observation differ by about 19ms on nearly flat road; this local approximation is screened, not assumed exact.')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,default=ROOT/'evidence/shallow_physical_mpc_20261013')
    p.add_argument('--anchored',action='store_true',help='Current-state affine drift, output and contact gap screen, using new derivative traces')
    p.add_argument('--road-step-s',type=float,choices=[.02,.001,.0005],default=.02)
    p.add_argument('--vertical',action='store_true',help='14 native vertical states; planar/tyre-slip states held fixed')
    a=p.parse_args(); out=a.output.resolve(); out.mkdir(parents=True,exist_ok=False)
    models={}
    for label,name in [('stationary','physical_flat_20261013_004'),('moving2','physical_flat_v2_20261013_001')]:
        models[label]=freeze_reference(ROOT/'runs/shallow_press/right_track'/name)
    # Save both physical candidates before loading holdouts. No fit or tuning.
    frozen={label:dict(metadata=item[4],state0=item[1].tolist(),input0=item[2].tolist(),
        state_names=item[3],model={k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in item[0].items()})
        for label,item in models.items()}
    (out/'frozen_models.json').write_text(json.dumps(frozen,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    reports=[]; figs,axes=plt.subplots(3,2,figsize=(12,10),sharex=False)
    for case in ('right_track','wide'):
        for depth in (2,4,6):
            name='physical_'+case+'_d'+str(depth)+'_v2_20261013_'+('002' if a.anchored else '001')
            run=ROOT/'runs/shallow_press'/case/name
            manifest,result,values=read(run)
            e=trace_features(values,result['scene']); scene=result['scene']; xs=e['xy'][:,:,0]
            front=xs[:,:2].max(axis=1) if case=='wide' else xs[:,1]
            rear=xs[:,2:].min(axis=1) if case=='wide' else xs[:,3]
            mask=(e['t']>=5)&(front>=scene['start_station_m']-.663)&(rear<=scene['start_station_m']+scene['length_m']+.763)
            starts=np.where(mask)[0]; starts=starts[starts+20<len(values)]
            edge=np.min(np.abs(xs[starts,:,None]-np.array([scene['start_station_m'],scene['start_station_m']+scene['length_m']])),axis=(1,2))<.3
            substeps=round(.02/a.road_step_s)
            horizon=20*substeps
            xy=np.stack([np.column_stack([values['exp_Xctc_'+w+'i'],values['exp_Yctc_'+w+'i']]) for w in ('L1','R1','L2','R2')],axis=1)
            roads=np.array([preview_physical_road(xy[k],speed_mps=e['y'][k,16],yaw_rad=np.deg2rad(values['exp_Yaw'][k]),dt=a.road_step_s,steps=horizon,scene=scene) for k in starts])
            commands=np.repeat(np.array([e['u'][k:k+20] for k in starts]),substeps,axis=1)
            persistence={}
            for ticks in (1,5,10,20):
                actual=e['y'][starts+ticks]; unchanged=e['y'][starts]
                contact=prediction_metrics(actual[:,8:12],unchanged[:,8:12],contact_truth=e['contact'][starts+ticks])
                persistence[str(ticks*.02)]=dict(metrics=prediction_metrics(actual,unchanged),
                    edge_metrics=prediction_metrics(actual[edge],unchanged[edge]) if edge.any() else None,
                    false_support_samples=contact['false_support_samples'])
            variant={}
            for label,(model,state0,u0,state_names,meta) in models.items():
                states=np.column_stack([values['sv_'+n][starts] for n in state_names])-state0
                inputs=np.concatenate((commands,roads),axis=2)-u0
                anchored_model=model
                if a.anchored:
                    # Reset once at the current observation. No future output
                    # or derivative enters propagation. These are oracle native
                    # states and causal 0.5ms backward derivatives, not sensors.
                    anchored_model=dict(model)
                    states=np.zeros_like(states)
                    previous=np.array([e['u'][max(0,k-1)] for k in starts])
                    inputs=np.concatenate((commands-previous[:,None,:],roads-roads[:,:1,:]),axis=2)
                    anchored_model['f0']=np.column_stack([values['fdot_'+n][starts] for n in state_names])
                    anchored_model['y0']=e['y'][starts].copy()
                    compression=np.column_stack([values['exp_CmpT_'+w+'i'][starts] for w in ('L1','R1','L2','R2')])*.001
                    stiffness=np.diag(model['D'][np.ix_(range(8,12),range(9,13))])
                    negative_gap=np.minimum(compression,0)*stiffness
                    anchored_model['y0'][:,8:12]=np.where(compression<0,negative_gap,anchored_model['y0'][:,8:12])
                    anchored_model['contact_correction_origin']=np.maximum(-negative_gap,0)
                if a.vertical:
                    names=['SV_ZO','SV_VZS','SV_ROLL','SV_PITCH','SV_AVX','SV_AVY']
                    names += [p+'_'+w for w in ('L1','R1','L2','R2') for p in ('SV_JNC','SV_JNCR')]
                    select=[state_names.index(n) for n in names]
                    anchored_model=dict(anchored_model)
                    anchored_model['A']=model['A'][np.ix_(select,select)]
                    anchored_model['B']=model['B'][select]
                    anchored_model['C']=model['C'][:,select]
                    anchored_model['f0']=anchored_model['f0'][...,select]
                    states=states[:,select]
                for unilateral in (False,True):
                    variant_name=label+('_unilateral' if unilateral else '_linear')
                    pred=physical_rollout_batch(anchored_model,states,inputs,a.road_step_s,unilateral=unilateral)
                    checks={}
                    for ticks in (1,5,10,20):
                        actual=e['y'][starts+ticks]; forecast=pred[:,ticks*substeps-1]
                        metric=prediction_metrics(actual,forecast)
                        contacts=prediction_metrics(actual[:,8:12],forecast[:,8:12],contact_truth=e['contact'][starts+ticks])
                        edge_metric=prediction_metrics(actual[edge],forecast[edge]) if edge.any() else None
                        checks[str(ticks*.02)]=dict(metrics=metric,edge_metrics=edge_metric,
                            false_support_samples=contacts['false_support_samples'],
                            gate=qualify_pit_prediction(metric,edge_metric,false_support_samples=contacts['false_support_samples'],rear_cleared=result['rear_cleared']))
                    variant[variant_name]=checks
                    if label=='moving2':
                        wheel=1 if case=='right_track' else 0
                        ax=axes[(depth//2)-1,0 if case=='right_track' else 1]
                        ax.plot(e['t'][starts+10],pred[:,10*substeps-1,wheel+8],label=variant_name,linewidth=.9)
                        if unilateral:
                            ax.plot(e['t'][starts+10],e['y'][starts+10,wheel+8],label='TruckSim',color='black',linewidth=1)
                            ax.set_title(case+' '+str(depth)+'cm / 2km/h'); ax.set_ylabel('Wheel load N'); ax.legend(fontsize=7)
                            np.savez_compressed(out/(name+'.npz'),t=e['t'][starts],truth=e['y'][starts+10],prediction=pred[:,10*substeps-1],edge=edge,contact=e['contact'][starts+10])
            report=dict(case=case,depth_cm=depth,run_path=str(run),rear_cleared=result['rear_cleared'],
                        trace_sha256=result['trace_sha256'],native_csv_sha256=result['native_csv_sha256'],
                        vehicle_identity=manifest['vehicle']['vehicle_sha256'],variants=variant,persistence=persistence)
            reports.append(report)
            print(json.dumps(dict(case=case,depth_cm=depth,metrics={n:dict(load_rmse_n=max(v['0.2']['metrics']['rmse'][8:12]),load_max_n=max(v['0.2']['metrics']['max_abs'][8:12]),false_support=v['0.2']['false_support_samples'],gate=v['0.2']['gate']) for n,v in variant.items()})),flush=True)
    figs.tight_layout(); figs.savefig(out/'load_predictions.png',dpi=150); plt.close(figs)
    summary=dict(scope='PHYSICAL_MODEL_PREDICTION_SCREEN_ONLY',mpc_closed_loop_run=False,
        state_initialization='ORACLE_NATIVE_SNAPSHOT_67',propagated_state_count=14 if a.vertical else 67,
        future_commands='ORACLE_APPLIED_SEQUENCE',
        preview='CURRENT_CONTACT_PATCH_POSE_CURRENT_SPEED_CONFIGURED_MAP',
        contact_approximation='Unilateral normal-load correction through native height dynamics; not a full hybrid tyre or friction law',
        current_affine_drift_and_gap=a.anchored,
        road_step_s=a.road_step_s,vertical_reduction=a.vertical,
        models=frozen,reports=reports,runtime_versions=dict(python=sys.version,numpy=np.__version__,scipy=scipy.__version__),
        code_sha256={str(f.relative_to(ROOT)):sha256_file(f) for f in
            [Path(__file__).resolve(),ROOT/'src/ddevsim/chassis_contact.py']+
            [ROOT/'src/ddevsim/strategies/shallow_press'/n for n in
             ('physical_prediction.py','native_physical_probe.py','identification.py','validation.py','prediction.py')]})
    (out/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n',encoding='utf-8')


if __name__=='__main__': main()
