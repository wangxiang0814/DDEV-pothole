from dataclasses import replace
import sys
from pathlib import Path

import pytest


def test_two_front_instances_do_not_share_config_or_state():
    from ddevsim.strategies.wheel_lift.front import LiftCrawlController
    from ddevsim.static_wheel_lift.config import CLOSED_LOOP_RUN
    first = LiftCrawlController(crawl=True, scenario={'friction': .7},
                               config=replace(CLOSED_LOOP_RUN, stop_ramp_s=2.6))
    second = LiftCrawlController(crawl=True, scenario={'friction': .7})
    assert first.config.stop_ramp_s == 2.6
    assert second.config.stop_ramp_s == CLOSED_LOOP_RUN.stop_ramp_s
    first.support.correction[0] = 100.
    first.preload_clock.progress_s = 12.
    assert second.support.correction[0] == 0.
    assert second.preload_clock.progress_s != 12.


def test_cli_override_passes_config_without_mutating_defaults(monkeypatch, tmp_path):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
    import run_right_side_full_cycle as cycle
    class ProbeDone(Exception):
        pass
    seen = []
    before = cycle.front_run.CFG
    def probe(*args, **kwargs):
        seen.append(kwargs['config'].stop_ramp_s)
        raise ProbeDone()
    monkeypatch.setattr(cycle.front_run, '_prepare_model', probe)
    monkeypatch.setattr(sys, 'argv', ['run', '--output', str(tmp_path / 'run'),
                                    '--front-stop-ramp-s', '2.6'])
    with pytest.raises(ProbeDone):
        cycle.main()
    assert seen == [2.6]
    assert cycle.front_run.CFG is before


def test_model_preparation_uses_explicit_configuration(tmp_path):
    from ddevsim.strategies.wheel_lift.model_builder import prepare_model
    from ddevsim.static_wheel_lift.config import CLOSED_LOOP_RUN
    config = replace(CLOSED_LOOP_RUN, crawl_speed_kph=2.6, control_period_s=.03)
    _, scene = prepare_model(tmp_path / 'run', crawl=True, config=config)
    assert scene['target_speed_kph'] == pytest.approx(2.6)
    assert scene['controller_period_s'] == pytest.approx(.03)
