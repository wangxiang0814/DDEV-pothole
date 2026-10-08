import json
from pathlib import Path
import sys

import pytest


def test_run_claim_is_exclusive_and_strategy_outputs_are_distinct(tmp_path):
    from ddevsim.runtime.run_workspace import claim_run
    root = tmp_path / 'runs'
    first = claim_run(root, 'wheel_lift', 'case', 'same')
    second = claim_run(root, 'shallow_press', 'case', 'same')
    assert first != second
    assert first.is_dir() and second.is_dir()
    with pytest.raises(FileExistsError):
        claim_run(root, 'wheel_lift', 'case', 'same')


@pytest.mark.parametrize('value', ['../escape', 'a/b', 'a\\b', '..', 'CON', 'C:', 'a b'])
def test_run_identifiers_reject_traversal(tmp_path, value):
    from ddevsim.runtime.run_workspace import claim_run
    with pytest.raises(ValueError):
        claim_run(tmp_path / 'runs', 'wheel_lift', value, 'run')
    assert not (tmp_path / 'escape').exists()


def test_source_asset_cannot_be_output():
    from ddevsim.runtime.run_workspace import claim_run, ROOT
    with pytest.raises(ValueError):
        claim_run(ROOT / 'models/runs', 'wheel_lift', 'case', 'run')


def test_unicode_worker_argv_is_preserved_and_failure_is_recorded(tmp_path):
    from ddevsim.runtime.run_workspace import claim_run, run_worker
    run_dir = claim_run(tmp_path / 'runs', 'wheel_lift', 'case', 'trial')
    script = tmp_path / '含 空格.py'
    script.write_text("import sys; print(sys.argv[1]); raise SystemExit(7)", encoding='utf-8')
    code = run_worker([sys.executable, str(script), '参数 with space'], run_dir=run_dir, cwd=tmp_path)
    assert code == 7
    manifest = json.loads((run_dir / 'run_manifest.json').read_text(encoding='utf-8'))
    assert manifest['process_exit_code'] == 7 and manifest['run_status'] == 'FAILED'
    assert manifest['argv'][2] == '参数 with space'
    assert not manifest['acceptance_pass']


def test_worker_return_zero_without_task_result_is_not_pass(tmp_path):
    from ddevsim.runtime.run_workspace import claim_run, run_worker
    run_dir = claim_run(tmp_path / 'runs', 'wheel_lift', 'case', 'trial')
    assert run_worker([sys.executable, '-c', 'pass'], run_dir=run_dir, cwd=tmp_path) == 1
    assert not json.loads((run_dir / 'run_manifest.json').read_text(encoding='utf-8'))['acceptance_pass']


def test_worker_start_failure_preserves_manifest(tmp_path):
    from ddevsim.runtime.run_workspace import claim_run, run_worker
    run_dir = claim_run(tmp_path / 'runs', 'wheel_lift', 'case', 'trial')
    assert run_worker([str(tmp_path / 'absent.exe')], run_dir=run_dir, cwd=tmp_path) != 0
    assert json.loads((run_dir / 'run_manifest.json').read_text(encoding='utf-8'))['run_status'] == 'ERROR'


def test_claimed_run_can_prepare_once_but_existing_output_cannot_be_reused(tmp_path):
    from ddevsim.runtime.run_workspace import claim_run
    from ddevsim.strategies.wheel_lift.model_builder import prepare_model
    run_dir = claim_run(tmp_path / 'runs', 'wheel_lift', 'case', 'trial')
    with pytest.raises(FileExistsError):
        prepare_model(run_dir, crawl=True)
    model, _ = prepare_model(run_dir, crawl=True, claimed_output=True)
    assert model == run_dir / 'model'
    with pytest.raises(FileExistsError):
        prepare_model(run_dir, crawl=True, claimed_output=True)
