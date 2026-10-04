"""Audit a native FR pit-front abort without labeling recovery as task success."""
import argparse
import csv
import hashlib
import json
from pathlib import Path


def summarize(run):
    result = json.loads((run / 'result.json').read_text(encoding='utf-8'))
    with (run / 'front_control_20ms.csv').open(encoding='utf-8', newline='') as stream:
        rows = list(csv.DictReader(stream))
    index = next((i for i, row in enumerate(rows) if row['mode'] == 'ABORT_STOP'), None)
    lower = next((row for row in rows if row['mode'] == 'LOWERING'), None)
    after = rows[max(0, index - 1):] if index is not None else []
    corners = ('fl', 'fr', 'rl', 'rr')
    scenario = result['scenario']
    cfg = result['controller_config']['front']
    start = scenario['start_station_m']
    end = start + scenario['length_m']
    def solid(station):
        return (station <= start - cfg['crossing_clearance_m'] or
                station >= end + cfg['crossing_clearance_m'])
    criteria = {
        'abort_recorded': index is not None and bool(result.get('fr_abort_reason')),
        'task_remains_fail': result['status'] == 'FAIL',
        'stopped_on_solid_ground_before_lowering': bool(lower and
            abs(float(lower['vx_kph'])) <= cfg['stop_speed_kph'] and
            solid(float(lower['x_fr_m'])) and solid(float(lower['x_rr_m']))),
        'four_wheel_recovered': bool(rows and rows[-1]['mode'] == 'COMPLETE' and
            all(float(rows[-1][f'fz_{c}_n']) >= cfg['support_floor_n'] for c in corners)),
    }
    jump = (max(abs(float(rows[index][f'fact_{c}_n']) -
                    float(rows[index - 1][f'fact_{c}_n'])) for c in corners)
            if index is not None and index > 0 else None)
    criteria['continuous_abort_handoff'] = jump is not None and jump <= 1e-6
    criteria['native_completed'] = result['front']['criteria']['native_completed']
    return {
        'run_name': run.name, 'recovery_status': 'PASS' if all(criteria.values()) else 'FAIL',
        'task_status': result['status'], 'criteria': criteria,
        'abort_reason': result.get('fr_abort_reason'),
        'abort_start_s': float(rows[index]['time_s']) if index is not None else None,
        'lower_start_s': float(lower['time_s']) if lower else None,
        'final_time_s': float(rows[-1]['time_s']) if rows else None,
        'abort_handoff_force_jump_n': jump,
        'maximum_following_20ms_force_step_n': max((
            abs(float(b[f'fact_{c}_n']) - float(a[f'fact_{c}_n']))
            for a, b in zip(after, after[1:]) for c in corners), default=None),
        'model_run_all_sha256': hashlib.sha256((run / 'model/run_all.par').read_bytes()).hexdigest(),
        'scope': 'FR stopped on solid ground; no over-pit or support-loss recovery claim',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = summarize(args.run)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
