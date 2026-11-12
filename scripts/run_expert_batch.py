"""Run a JSON plan of isolated wheel_lift and shallow_press simulations."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from ddevsim.runtime.batch import build_jobs, describe_job
from ddevsim.runtime.run_workspace import validate_identifier


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--batch-id', required=True)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)
    validate_identifier(args.batch_id)
    runs = (ROOT / 'runs').resolve()
    output = runs / '_batches' / args.batch_id
    if output.resolve() != output or output.exists():
        raise FileExistsError(output)
    plan_bytes = args.plan.read_bytes()
    jobs = build_jobs(json.loads(plan_bytes), ROOT)
    for job in jobs:
        check = subprocess.run([*job.argv, '--dry-run'], cwd=ROOT, capture_output=True)
        if check.returncode:
            raise ValueError(f'{job.output}: {check.stderr.decode(errors="replace")}')
    if args.dry_run:
        print(json.dumps({'validated': [str(job.output) for job in jobs]}, ensure_ascii=False))
        return 0

    output.mkdir(parents=True, exist_ok=False)
    (output / 'plan.json').write_bytes(plan_bytes)
    summary = {'version': 1, 'plan_sha256': hashlib.sha256(plan_bytes).hexdigest(),
               'planned': len(jobs), 'runs': []}
    for job in jobs:
        try:
            record = describe_job(job, subprocess.run(job.argv, cwd=ROOT).returncode)
        except OSError as exc:
            record = describe_job(job, 1)
            record['launch_error'] = str(exc)
        summary['runs'].append(record)
        temporary = output / 'summary.tmp'
        temporary.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n',
                             encoding='utf-8')
        temporary.replace(output / 'summary.json')
    print('Batch summary:', output / 'summary.json')
    return int(any(row['returncode'] for row in summary['runs']))


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as exc:
        print(exc, file=sys.stderr)
        raise SystemExit(2)
