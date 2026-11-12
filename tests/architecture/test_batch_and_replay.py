import pytest

from ddevsim.native_video import resolve_run_source
from ddevsim.runtime.batch import build_jobs


def test_playback_run_id_resolves_only_when_unique(tmp_path):
    first = tmp_path / 'runs/wheel_lift/seed_repeat/example'
    first.mkdir(parents=True)
    assert resolve_run_source('example', tmp_path) == first
    second = tmp_path / 'runs/shallow_press/wide/example'
    second.mkdir(parents=True)
    with pytest.raises(ValueError, match='ambiguous'):
        resolve_run_source('example', tmp_path)
    assert resolve_run_source('runs/shallow_press/wide/example', tmp_path) == second


def test_batch_preflight_refuses_duplicate_and_existing_runs(tmp_path):
    row = dict(strategy='shallow_press', case='wide', run_id='test_001',
               args=['--depth-m', '.04', '--speed-kph', '3'])
    plan = dict(version=1, runs=[row])
    jobs = build_jobs(plan, tmp_path)
    assert jobs[0].output == tmp_path / 'runs/shallow_press/wide/test_001'
    assert jobs[0].argv[-4:] == tuple(row['args'])
    assert not (tmp_path / 'runs').exists()
    with pytest.raises(ValueError, match='duplicate'):
        build_jobs(dict(version=1, runs=[row, row]), tmp_path)
    with pytest.raises(ValueError, match='batch owns'):
        build_jobs(dict(version=1, runs=[dict(row, args=['--run-i', 'elsewhere'])]), tmp_path)
    jobs[0].output.mkdir(parents=True)
    with pytest.raises(FileExistsError):
        build_jobs(plan, tmp_path)
