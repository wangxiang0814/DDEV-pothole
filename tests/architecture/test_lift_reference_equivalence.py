import json
from pathlib import Path

import pytest


@pytest.mark.parametrize('profile', ['baseline', 'efficient'])
def test_front_commands_match_pre_refactor_observations(profile):
    from ddevsim.strategies.wheel_lift.front import LiftCrawlController, EXPORTS
    from ddevsim.static_wheel_lift.config import CLOSED_LOOP_RUN, EFFICIENT_CYCLE_FRONT
    data = json.loads((Path(__file__).parent / 'fixtures/lift_baseline.json').read_text())
    config = {'baseline': CLOSED_LOOP_RUN, 'efficient': EFFICIENT_CYCLE_FRONT}[profile]
    controller = LiftCrawlController(crawl=True, scenario=data['scenario'], config=config)
    assert list(EXPORTS) == data['exports']
    case = next(c for c in data['cases'] if c['profile'] == profile)
    for event in case['events']:
        if event.get('enter'):
            controller._enter(event['enter'], event['time_s'])
        result = controller(event['time_s'], event['exports'])
        assert result == pytest.approx(event['command'], abs=1e-10, rel=1e-12)
        assert controller.mode == event['mode']
