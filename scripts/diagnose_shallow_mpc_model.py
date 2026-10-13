"""Independent physical-model, oracle-preview and numerical-sensitivity checks."""
import json
import argparse
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from analyze_shallow_mpc_validation import OUT,read_run,write
from ddevsim.runtime.run_workspace import sha256_file
from ddevsim.strategies.shallow_press.identification import OUTPUT_NAMES,prediction_gate
from ddevsim.strategies.shallow_press.prediction import DynamicModel,rollout,prediction_metrics,propagate_state_space


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-name',choices=['model_diagnostics.json','model_diagnostics_verified.json'],default='model_diagnostics.json')
    args=parser.parse_args()
    collection=json.loads((OUT/'collection.json').read_text(encoding='utf-8'))
    native=json.loads((OUT/'native_linear_model.json').read_text(encoding='utf-8'))
    native={k:(np.asarray(v) if k in 'ABCD' else v) for k,v in native.items()}
    select=[native['output_names'].index(n) for n in OUTPUT_NAMES]
    local=[]
    for item in collection['runs'][:2]:
        e=read_run(item)
        k=int(np.where(e['t']>=5)[0][0])
        stop=int(np.where(e['t']>=15)[0][0])
        predicted=propagate_state_space(native['A'],native['B'],native['C'][select],native['D'][select],
             np.zeros(67),e['u'][k:stop],dt=.02)+e['y'][k]
        metrics=prediction_metrics(e['y'][k+1:stop+1],predicted)
        contact=prediction_metrics(e['y'][k+1:stop+1,8:12],predicted[:,8:12],
                                   contact_truth=e['contact'][k+1:stop+1])
        local.append(dict(run_path=item['run_path'],note='Native physical model, zero perturbation state at settled t=5s, all measured applied inputs; no fitting to this run.',
                          metrics=metrics,contact=contact,gate=prediction_gate(metrics,false_support_samples=contact['false_support_samples'])))
    saved=json.loads((OUT/'dynamic_model.json').read_text(encoding='utf-8'))['parameters']
    for key in ('coefficients','y_center','y_scale','u_scale','road_scale'):
        saved[key]=np.asarray(saved[key])
    model=DynamicModel(**saved)
    summary=json.loads((OUT/'prediction_summary.json').read_text(encoding='utf-8'))
    oracles=[]
    for item in collection['runs']:
        if item['role']!='holdout':
            continue
        e=read_run(item)
        scene=e['scene']
        x=e['xy'][:,:,0]
        front=x[:,:2].max(axis=1) if item['options']['case']=='wide' else x[:,1]
        rear=x[:,2:].min(axis=1) if item['options']['case']=='wide' else x[:,3]
        mask=(e['t']>=5)&(front>=scene['start_station_m']-.663)&(rear<=scene['start_station_m']+scene['length_m']+.763)
        starts=np.where(mask)[0]
        starts=starts[(starts>=1)&(starts+10<len(e['t']))]
        pred=np.array([rollout(model,e['y'][k-1:k+1],e['u'][k:k+10],e['road'][k:k+11])[-1] for k in starts])
        truth=e['y'][starts+10]
        metric=prediction_metrics(truth,pred)
        contacts=prediction_metrics(truth[:,8:12],pred[:,8:12],contact_truth=e['contact'][starts+10])
        oracles.append(dict(run_path=item['run_path'],metrics=metric,contact=contacts,
                           gate=prediction_gate(metric,false_support_samples=contacts['false_support_samples']),
                           note='DIAGNOSTIC_ORACLE_FUTURE_POSE_FOR_ROAD_ONLY; not deployable, not used for training, not release gate.'))
    numerical=[]
    for item in collection['runs']:
        if item['role']!='step':
            continue
        base,=[r for r in collection['runs'] if r['role']=='holdout' and r['options']['case']==item['options']['case']
               and r['options']['depth_m']==item['options']['depth_m'] and r['options']['speed_kph']==item['options']['speed_kph']]
        signals=[]
        names=['Fz_L1','Fz_R1','Fz_L2','Fz_R2','Jnc_L1','Jnc_R1','Jnc_L2','Jnc_R2',
               'Roll_E','Pitch','Az_SM','Vx']
        for r in (base,item):
            diag=json.loads((Path(r['run_path'])/'prediction_diagnostic.json').read_text(encoding='utf-8'))
            p=Path(diag['native']['csv'])
            columns=p.open(encoding='utf-8').readline().strip().split(',')
            chosen=['time_s']+['exp_'+n for n in names]
            vals=np.genfromtxt(p,delimiter=',',skip_header=1,usecols=[columns.index(n) for n in chosen])
            keep=vals[:,0]>=5
            maxloads=np.max(vals[keep,1:5],axis=0)
            jncmax=np.max(vals[keep,5:9],axis=0)
            jncmin=np.min(vals[keep,5:9],axis=0)
            az=np.convolve(vals[keep,11]*9.80665,np.ones(10)/10,mode='valid')
            signals.append(dict(run_path=r['run_path'],native_csv_sha256=sha256_file(p),
                max_load_n=maxloads.tolist(),max_jounce_mm=jncmax.tolist(),min_jounce_mm=jncmin.tolist(),
                roll_peak_deg=float(max(abs(vals[keep,9]))),pitch_peak_deg=float(max(abs(vals[keep,10]))),
                az_10ms_peak_m_s2=float(max(abs(az))),rear_cleared=diag['rear_cleared']))
        a,b=signals
        compare=dict(load_peak_change_fraction=float(np.max(abs(np.array(b['max_load_n'])-a['max_load_n'])/np.array(a['max_load_n']))),
                     max_jounce_change_mm=float(max(abs(np.array(b['max_jounce_mm'])-a['max_jounce_mm']))),
                     roll_peak_change_deg=abs(b['roll_peak_deg']-a['roll_peak_deg']),
                     pitch_peak_change_deg=abs(b['pitch_peak_deg']-a['pitch_peak_deg']),
                     az_10ms_change_fraction=abs(b['az_10ms_peak_m_s2']-a['az_10ms_peak_m_s2'])/max(a['az_10ms_peak_m_s2'],1e-12))
        compare['research_step_screen_pass']=(compare['load_peak_change_fraction']<=.05 and compare['max_jounce_change_mm']<=2
              and compare['roll_peak_change_deg']<=.5 and compare['pitch_peak_change_deg']<=.5 and compare['az_10ms_change_fraction']<=.05)
        numerical.append(dict(case=item['options']['case'],signals=signals,comparison=compare,
                 note='0.5 vs 0.25ms with 1ms logs. Native and controller first-tick phase differs by .25ms; not a pure integration-error attribution.'))
    write(args.output_name,dict(native_flat_validation=local,oracle_preview_200ms=oracles,
                                      step_sensitivity=numerical))
    print(json.dumps({'native_flat_gates':[r['gate'] for r in local],
                      'oracle_joint_pass_count':sum(r['gate']['joint_mpc_model'] for r in oracles),
                      'step':[{r['case']:r['comparison']} for r in numerical]},indent=2))


if __name__=='__main__':
    main()
