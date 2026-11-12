"""Build batch jobs for the same launcher used by individual expert runs."""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import sys

from ddevsim.native_video import find_history
from ddevsim.runtime.run_workspace import run_output_path


@dataclass(frozen=True)
class BatchJob:
    output: Path
    argv: tuple[str, ...]


def build_jobs(plan: dict, project: Path) -> list[BatchJob]:
    if (not isinstance(plan, dict) or plan.get('version') != 1
            or not isinstance(plan.get('runs'), list) or not plan['runs']):
        raise ValueError('expected a version 1 batch plan with a non-empty runs list')
    owned = ('--strategy', '--case', '--run-id', '--dry-run', '--claimed-output', '--help')
    jobs, outputs = [], set()
    for row in plan['runs']:
        if not isinstance(row, dict) or set(row) - {'strategy', 'case', 'run_id', 'args'}:
            raise ValueError('invalid batch run entry')
        strategy, case, run_id = (row.get(key) for key in ('strategy', 'case', 'run_id'))
        output = run_output_path(project / 'runs', strategy, case, run_id)
        if output in outputs:
            raise ValueError('duplicate batch output: ' + str(output))
        if output.exists():
            raise FileExistsError(output)
        args = row.get('args', [])
        if not isinstance(args, list) or not all(isinstance(a, str) for a in args):
            raise ValueError('batch args must be a list of CLI strings')
        for arg in args:
            option = arg.partition('=')[0]
            if option in ('-h', '--') or (
                option.startswith('--') and any(name.startswith(option) for name in owned)
            ):
                raise ValueError('the batch owns the option ' + option)
        argv = (sys.executable, str(project / 'scripts/run_strategy.py'),
                '--strategy', strategy, '--case', case, '--run-id', run_id, *args)
        jobs.append(BatchJob(output, argv))
        outputs.add(output)
    return jobs


def describe_job(job: BatchJob, returncode: int) -> dict:
    """Expose produced artifacts and preserve the strategy's acceptance result."""
    folder = job.output
    record = {'output': str(folder), 'argv': list(job.argv), 'returncode': returncode}
    for filename, keys in (
        ('run_manifest.json', ('run_status', 'task_status', 'simulation_pass', 'acceptance_pass',
                               'demonstration')),
        ('result.json', ('status', 'simulation_pass', 'acceptance_pass')),
    ):
        path = folder / filename
        if path.is_file():
            try:
                data = json.loads(path.read_text(encoding='utf-8'))
                if not isinstance(data, dict):
                    raise ValueError(f'{filename} must contain a JSON object')
                record[path.stem] = {key: data[key] for key in keys if key in data}
            except (OSError, ValueError, TypeError) as exc:
                record[path.stem + '_error'] = str(exc)
    record['logs'] = [path.name for path in sorted(folder.glob('*.csv'))]
    if (folder / 'run.log').is_file():
        record['logs'].insert(0, 'run.log')
    try:
        record['native_history'] = find_history(folder).as_dict()
    except FileNotFoundError:
        record['native_history'] = None
    except (OSError, ValueError) as exc:
        record['native_history_error'] = str(exc)
    return record
