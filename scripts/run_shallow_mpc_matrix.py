"""Run the frozen predictor experiment matrix in separate sequential processes."""
import itertools
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from ddevsim.runtime.run_workspace import sha256_file


def main():
    matrix=ROOT/'configs/shallow_press/mpc_validation_20261013.json'
    config=json.loads(matrix.read_text(encoding='utf-8'))
    out=ROOT/'evidence/shallow_mpc_validation_20261013'
    out.mkdir(exist_ok=False)
    jobs=[]
    for i,opts in enumerate(config['training']):
        jobs.append(('train',dict(opts,excite=True), 'mpc_model_train_20261013_%02d'%i))
    hold=config['holdout']
    for case,depth,speed in itertools.product(hold['cases'],hold['depths_m'],hold['speeds_kph']):
        jobs.append(('holdout',dict(case=case,depth_m=depth,speed_kph=speed,seed=hold['seed'],
            excite=False,duration_s=hold['duration_s']),
            'mpc_model_holdout_20261013_v%02d_d%03d'%(speed,round(depth*1000))))
    for i,opts in enumerate(config['step_sensitivity']):
        jobs.append(('step',dict(opts,excite=False),'mpc_model_step_20261013_%02d'%i))
    results=[]
    manifest={'matrix':str(matrix),'matrix_sha256':sha256_file(matrix),
              'runs':results,'total_planned':len(jobs),'status':'RUNNING'}
    dest=out/'collection.json'
    for role,opts,run_id in jobs:
        argv=[sys.executable,str(ROOT/'scripts/run_shallow_mpc_validation.py'),'--run-id',run_id]
        for key,val in opts.items():
            if key=='excite':
                if val:
                    argv.append('--excite')
            else:
                argv.extend(['--'+key.replace('_','-'),str(val)])
        log=out/(opts['case']+'_'+run_id+'.log')
        started=time.monotonic()
        with log.open('w',encoding='utf-8') as f:
            result=subprocess.run(argv,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
        item=dict(role=role,options=opts,run_id=run_id,run_path=str(ROOT/'runs/shallow_press'/opts['case']/run_id),
                  exit_code=result.returncode,wall_s=time.monotonic()-started,log=str(log),argv=argv)
        results.append(item)
        dest.write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
        print('%d/%d %s %s rc=%d wall=%.1fs'%(len(results),len(jobs),role,opts['case']+'/'+run_id,result.returncode,item['wall_s']),flush=True)
    manifest['status']='COMPLETED' if all(r['exit_code']==0 for r in results) else 'HAS_FAILURES'
    dest.write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    return 0 if manifest['status']=='COMPLETED' else 1


if __name__=='__main__':
    raise SystemExit(main())
