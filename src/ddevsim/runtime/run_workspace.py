"""Exclusive run directories and subprocess execution, independent of algorithms."""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[3]
_RESERVED = {'CON', 'PRN', 'AUX', 'NUL', *(f'COM{i}' for i in range(1, 10)),
             *(f'LPT{i}' for i in range(1, 10))}


def sha256_file(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _inside(path, parent):
    try:
        path.relative_to(parent)
        return True
    except ValueError:
        return False


def claim_run(root: Path, strategy_id: str, case_id: str, run_id: str) -> Path:
    root = Path(root).resolve()
    for value in (strategy_id, case_id, run_id):
        if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,95}', value) or value.upper() in _RESERVED:
            raise ValueError('run identifiers must be safe single path segments')
    if strategy_id not in ('wheel_lift', 'shallow_press'):
        raise ValueError('unknown strategy')
    protected = ('models', 'evidence', 'src', 'scripts', 'interfaces', 'configs', 'scenarios', '.git')
    if root.name != 'runs' or any(_inside(root, ROOT / p) for p in protected):
        raise ValueError('outputs must be under a runs directory, outside source assets')
    target = (root / strategy_id / case_id / run_id).resolve()
    if not _inside(target, root):
        raise ValueError('run output escapes root')
    target.parent.mkdir(parents=True, exist_ok=True)
    target.mkdir(exist_ok=False)
    identity = {'strategy_id': strategy_id, 'case_id': case_id, 'run_id': run_id,
                'output': str(target), 'created_utc': datetime.now(timezone.utc).isoformat()}
    with (target / 'run_claim.json').open('x', encoding='utf-8') as dest:
        json.dump(identity, dest, indent=2)
    write_run_manifest(target, {**identity, 'run_status': 'CLAIMED', 'acceptance_pass': False})
    return target


def verify_claimed_output(path: Path) -> None:
    path = Path(path).resolve()
    claim = json.loads((path / 'run_claim.json').read_text(encoding='utf-8'))
    if claim['output'] != str(path) or claim['strategy_id'] != 'wheel_lift':
        raise ValueError('output claim does not match wheel_lift worker')
    if set(p.name for p in path.iterdir()) - {'run_claim.json', 'run_manifest.json', 'run.log', 'control_config.json'}:
        raise FileExistsError('claimed run already contains generated artifacts')


def code_identity() -> dict:
    def git(*args):
        result = subprocess.run(['git', *args], cwd=ROOT, capture_output=True, text=True)
        return result.stdout.strip() if result.returncode == 0 else None
    state = git('status', '--porcelain')
    return {'commit': git('rev-parse', 'HEAD'), 'dirty': None if state is None else bool(state)}


def write_run_manifest(run_dir: Path, data) -> Path:
    path = Path(run_dir) / 'run_manifest.json'
    temporary = path.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    temporary.replace(path)
    return path


def run_worker(argv: list[str], *, run_dir: Path, cwd: Path) -> int:
    run_dir = Path(run_dir).resolve()
    manifest_path = run_dir / 'run_manifest.json'
    record = json.loads(manifest_path.read_text(encoding='utf-8'))
    record.update(argv=list(argv), code=code_identity(), run_status='RUNNING')
    write_run_manifest(run_dir, record)
    try:
        with (run_dir / 'run.log').open('w', encoding='utf-8') as log:
            process = subprocess.run(argv, cwd=cwd, stdout=log, stderr=subprocess.STDOUT)
        record['process_exit_code'] = process.returncode
        result_path = run_dir / 'result.json'
        result = json.loads(result_path.read_text(encoding='utf-8')) if result_path.exists() else {}
        criteria = result.get('criteria', {})
        record['task_status'] = result.get('status', 'UNASSESSED')
        record['acceptance_pass'] = (process.returncode == 0 and result.get('status') == 'PASS'
                                     and bool(criteria) and all(v is True for v in criteria.values()))
        record['run_status'] = 'COMPLETED' if record['acceptance_pass'] else 'FAILED'
        for name in ('model/run_all.par', 'model/simfile.sim', 'model/scenario.json'):
            path = run_dir / name
            if path.is_file():
                record.setdefault('generated_assets', {})[name] = sha256_file(path)
        code = process.returncode or (0 if record['acceptance_pass'] else 1)
    except (OSError, ValueError, TypeError) as exc:
        record.update(run_status='ERROR', error=str(exc), acceptance_pass=False)
        code = 1
    write_run_manifest(run_dir, record)
    return code
