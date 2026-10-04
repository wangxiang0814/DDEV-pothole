import sys
from pathlib import Path
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import run_right_side_full_cycle as cycle


def test_preload_rate_override_applied_before_building_the_model(monkeypatch, tmp_path):
    class ProbeDone(Exception):
        pass
    seen = []
    def probe(*args, **kwargs):
        seen.append(cycle.front_run.CFG.preload_max_reference_rate)
        raise ProbeDone()
    monkeypatch.setattr(cycle.front_run, 'CFG', cycle.CLOSED_LOOP_RUN)
    monkeypatch.setattr(cycle.front_run, '_prepare_model', probe)
    monkeypatch.setattr(sys, 'argv', ['run', '--output', str(tmp_path / 'run'),
        '--efficiency-profile', 'compact', '--front-preload-reference-rate', '4'])
    with pytest.raises(ProbeDone):
        cycle.main()
    assert seen == [4.]


@pytest.mark.parametrize('rate', ['nan', 'inf', '0.5', '-1'])
def test_invalid_rate_does_not_create_a_model(monkeypatch, tmp_path, capsys, rate):
    target = tmp_path / 'run'
    monkeypatch.setattr(sys, 'argv', ['run', '--output', str(target),
        '--front-preload-reference-rate', rate])
    with pytest.raises(SystemExit):
        cycle.main()
    assert 'preload rate must be finite and at least 1' in capsys.readouterr().err
    assert not target.exists()
