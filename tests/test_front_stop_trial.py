import sys
from pathlib import Path
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import run_right_side_full_cycle as cycle


def test_stop_ramp_applied_before_model_copy(monkeypatch, tmp_path):
    class ProbeDone(Exception):
        pass
    seen = []
    def probe(*args, **kwargs):
        seen.append(cycle.front_run.CFG.stop_ramp_s)
        raise ProbeDone()
    monkeypatch.setattr(cycle.front_run, 'CFG', cycle.CLOSED_LOOP_RUN)
    monkeypatch.setattr(cycle.front_run, '_prepare_model', probe)
    monkeypatch.setattr(sys, 'argv', ['run', '--output', str(tmp_path / 'run'),
        '--front-stop-ramp-s', '2.6'])
    with pytest.raises(ProbeDone):
        cycle.main()
    assert seen == [2.6]


@pytest.mark.parametrize('value', ['nan', 'inf', '0', '-1', '5'])
def test_invalid_stop_ramp_rejected_before_model_copy(monkeypatch, tmp_path, value):
    def probe(*args, **kwargs):
        pytest.fail('invalid stopping ramp reached model copy')
    monkeypatch.setattr(cycle.front_run, '_prepare_model', probe)
    monkeypatch.setattr(sys, 'argv', ['run', '--output', str(tmp_path / 'run'),
        '--front-stop-ramp-s', value])
    with pytest.raises(SystemExit):
        cycle.main()


@pytest.mark.parametrize('value', ['nan', 'inf', '0', '-1', '1201'])
def test_invalid_stop_support_slew_rejected_before_model_copy(monkeypatch, tmp_path, value):
    def probe(*args, **kwargs):
        pytest.fail('invalid support slew reached model copy')
    monkeypatch.setattr(cycle.front_run, '_prepare_model', probe)
    monkeypatch.setattr(sys, 'argv', ['run', '--output', str(tmp_path / 'run'),
        '--front-stop-support-slew-n-s', value])
    with pytest.raises(SystemExit):
        cycle.main()


def test_stop_support_slew_applied_before_model_copy(monkeypatch, tmp_path):
    class ProbeDone(Exception):
        pass
    seen = []
    def probe(*args, **kwargs):
        seen.append(cycle.front_run.CFG.stop_support_slew_n_s)
        raise ProbeDone()
    monkeypatch.setattr(cycle.front_run, 'CFG', cycle.CLOSED_LOOP_RUN)
    monkeypatch.setattr(cycle.front_run, '_prepare_model', probe)
    monkeypatch.setattr(sys, 'argv', ['run', '--output', str(tmp_path / 'run'),
        '--front-stop-support-slew-n-s', '1200'])
    with pytest.raises(ProbeDone):
        cycle.main()
    assert seen == [1200.]
