"""Run an explicit frozen plan with at most two isolated native worker processes."""
import argparse
from concurrent.futures import ThreadPoolExecutor,as_completed
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]


def sha(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()


def validate_plan(plan,root):
    seen=set()
    for row in plan['cases']:
        key=(row['case'],row['run_id'])
        if key in seen:raise ValueError('duplicate matrix output '+str(key))
        seen.add(key)
        args=row['args']
        for option,value in (('--case',key[0]),('--run-id',key[1])):
            if args.count(option)!=1 or args[args.index(option)+1]!=value:
                raise ValueError('matrix identity does not match arguments')
        if Path(key[1]).name!=key[1] or key[0] not in ('right_track','wide'):
            raise ValueError('invalid matrix output')
    for name,expected in {**plan['source_sha256'],plan['config_path']:plan['config_sha256']}.items():
        if sha(root/name)!=expected:raise ValueError('frozen input drift: '+name)


def can_resume(run,plan_sha,args):
    try:
        marker=json.loads((run/'matrix_identity.json').read_text(encoding='utf-8'))
        manifest=json.loads((run/'run_manifest.json').read_text(encoding='utf-8'))
        result=json.loads((run/'result.json').read_text(encoding='utf-8'))
        inputs=(('native_1ms.csv','native_csv_sha256'),('model/run_all.par','model_sha256'),
                ('input_provenance.json','input_provenance_sha256'))
        return (marker==dict(plan_sha256=plan_sha,args=args) and
                manifest['run_status']=='COMPLETED' and
                manifest['result_sha256']==sha(run/'result.json') and
                all(sha(run/name)==result[key] for name,key in inputs))
    except (OSError,ValueError,KeyError):return False


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan',required=True,type=Path)
    parser.add_argument('--workers',type=int,choices=(1,2),default=2)
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--dry-run',action='store_true')
    args=parser.parse_args();plan=json.loads(args.plan.read_text(encoding='utf-8'))
    validate_plan(plan,ROOT);digest=sha(args.plan)
    if args.dry_run:
        print(json.dumps(dict(cases=len(plan['cases']),workers=args.workers,plan_sha256=digest)))
        return 0
    # Preflight every case before dispatch, so malformed options claim no runs.
    worker=ROOT/'scripts/run_shallow_preview.py';commands=[]
    for row in plan['cases']:
        run=ROOT/'runs/shallow_press'/row['case']/row['run_id']
        resume=args.resume and can_resume(run,digest,row['args'])
        if run.exists() and not resume:raise FileExistsError('matrix refuses existing output '+str(run))
        command=[sys.executable,str(worker)]+row['args']
        check=subprocess.run(command+['--dry-run'],cwd=str(ROOT),stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        if check.returncode:raise ValueError(check.stderr.decode('utf-8',errors='replace'))
        commands.append((row,run,command,resume))
    logs=args.plan.parent/'logs';logs.mkdir(exist_ok=True)
    def execute(item):
        row,run,command,resume=item
        validate_plan(plan,ROOT)
        code=0
        if not resume:
            with (logs/(row['run_id']+'.log')).open('xb') as log:
                code=subprocess.run(command,cwd=str(ROOT),stdout=log,stderr=subprocess.STDOUT).returncode
            if code==0:
                with (run/'matrix_identity.json').open('x',encoding='utf-8') as marker:
                    json.dump(dict(plan_sha256=digest,args=row['args']),marker,indent=2)
        record=json.loads((run/'result.json').read_text(encoding='utf-8')) if (run/'result.json').exists() else {}
        return dict(case=row['case'],run_id=row['run_id'],kind=row['kind'],depth_m=row['depth_m'],
            crossing_kph=row['crossing_kph'],returncode=code,resumed=resume,
            status=record.get('status','ERROR'),simulation_pass=record.get('simulation_pass',False),
            failed_criteria=[k for k,v in record.get('criteria',{}).items() if not v],
            metrics=record.get('metrics'),exit_progress=record.get('exit_progress'),
            result_sha256=sha(run/'result.json') if record else None)
    results=[]
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures={pool.submit(execute,item):item[0] for item in commands}
        for future in as_completed(futures):
            row=futures[future]
            try:result=future.result()
            except Exception as exc:result=dict(run_id=row['run_id'],status='ERROR',returncode=-1,error=repr(exc))
            results.append(result)
            print(json.dumps({k:result[k] for k in ('run_id','status','returncode')},ensure_ascii=False),flush=True)
            output=args.plan.parent/'matrix_summary.json';temp=output.with_suffix('.tmp')
            temp.write_text(json.dumps(dict(plan_sha256=digest,results=results),indent=2)+'\n',encoding='utf-8')
            temp.replace(output)
    return int(any(r['returncode'] for r in results))


if __name__=='__main__':raise SystemExit(main())
