"""Prepare explicit shallow-pit geometry and display resources; no control simulation."""
from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from ddevsim.scenarios.pit import load_case
from ddevsim.scenarios.approach import DEFAULT_INITIAL_DISTANCE_M, DRY_ROAD_FRICTION, approach_distance
from ddevsim.scenarios.builder import prepare_case
from ddevsim.runtime.run_workspace import claim_run


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', required=True, choices=('right_track', 'wide'))
    parser.add_argument('--run-id')
    parser.add_argument('--template', type=Path)
    parser.add_argument('--approach-distance-m', type=float, default=DEFAULT_INITIAL_DISTANCE_M)
    parser.add_argument('--friction', type=float, choices=[DRY_ROAD_FRICTION])
    for name in ('depth-m', 'length-m', 'width-m', 'stop-s'):
        parser.add_argument('--' + name, type=float)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)
    template = args.template or ROOT / 'scenarios/shallow_press' / (args.case + '.json')
    try:
        case = load_case(template, depth_m=args.depth_m, length_m=args.length_m, width_m=args.width_m,
                         approach_distance_m=approach_distance(args.approach_distance_m),
                         stop_s=args.stop_s, fixed_origin=True)
        if case.case_id != args.case or case.strategy_id != 'shallow_press':
            raise ValueError('template and requested shallow case differ')
    except (ValueError, KeyError) as exc:
        parser.error(str(exc))
    if args.dry_run:
        print(json.dumps({'status': 'VALIDATED', 'case_id': case.case_id,
                          'coverage': case.coverage, 'wheel_sequence': case.wheel_sequence,
                          'initial_state': dict(case.initial_state)}, indent=2))
        return 0
    run_id = args.run_id or datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    run_dir = claim_run(ROOT / 'runs', case.strategy_id, case.case_id, run_id)
    result = prepare_case(case, run_dir)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print('Prepared scene:', run_dir)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
