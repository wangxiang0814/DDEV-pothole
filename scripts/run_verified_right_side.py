"""Named, reviewable native validation cases; no videos, no model rebuild."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
MANIFEST=ROOT/'configs/verified_right_side_20261005.json'


def build_command(manifest,case,output,reference_model=None):
    if case not in manifest['cases']:
        raise ValueError('unknown validation case')
    selected=manifest['cases'][case]
    options={**manifest['common'],**selected['options']}
    if 'output' in options or 'support-allocation-reference-model' in options:
        raise ValueError('output/reference must be explicit launcher arguments')
    if selected.get('requires_reference_model'):
        if reference_model is None: raise ValueError('boundary needs an explicit original reference model')
        options['support-allocation-reference-model']=str(reference_model.resolve())
    command=[sys.executable,str(ROOT/'scripts/run_right_side_full_cycle.py'),'--output',str(output.resolve())]
    for key,value in options.items():
        if value is None or value is False: continue
        command.append('--'+key)
        if value is not True: command.append(str(value))
    return command


def main():
    manifest=json.loads(MANIFEST.read_text(encoding='utf-8'))
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',choices=tuple(manifest['cases']),required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--reference-model',type=Path)
    parser.add_argument('--dry-run',action='store_true')
    args=parser.parse_args()
    output=args.output.resolve()
    command=build_command(manifest,args.case,output,args.reference_model)
    record={'case':args.case,'scope':manifest['scope'],'argv':command,
            'manifest_sha256':hashlib.sha256(MANIFEST.read_bytes()).hexdigest()}
    print(json.dumps(record,indent=2),flush=True)
    if args.dry_run: return
    if output.exists(): raise FileExistsError(output)
    process=subprocess.run(command,cwd=ROOT)
    record['process_exit_code']=process.returncode
    result_path=output/'result.json'
    if result_path.exists():
        result=json.loads(result_path.read_text())
        record['task_status']=result['status']
        record['acceptance_pass']=result['status']=='PASS' and all(result['criteria'].values())
    else:
        record['acceptance_pass']=False
    if output.exists():
        (output/'launch_manifest.json').write_text(json.dumps(record,indent=2)+'\n')
    if process.returncode or not record['acceptance_pass']:
        raise SystemExit(process.returncode or 1)


if __name__=='__main__': main()
