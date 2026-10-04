"""Export matched native force-bias responses and full-cycle truth acceptance."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from summarize_right_side_matrix import summarize


def read_rows(run, stage):
    with (run / f'{stage}_control_20ms.csv').open(encoding='utf-8', newline='') as stream:
        return list(csv.DictReader(stream))


def compare(baseline, run):
    base = summarize(baseline)
    record = summarize(run)
    a, b = base['result'], record['result']
    for field in ('controller_config', 'measurement_noise', 'scenario'):
        if field == 'measurement_noise':
            ca, cb = dict(a[field]['config']), dict(b[field]['config'])
            # Historical no-delay runs predate this field; absence means zero.
            ca.setdefault('feedback_delay_s', 0.)
            cb.setdefault('feedback_delay_s', 0.)
            if ca != cb:
                raise ValueError('noise and delay must match the baseline')
        elif a[field] != b[field]:
            raise ValueError(f'mismatched baseline {field}')
    if (base['model_run_all_sha256'] != record['model_run_all_sha256'] or
            a['support_allocation']['config'] != b['support_allocation']['config'] or
            a['support_allocation']['provenance'] != b['support_allocation']['provenance']):
        raise ValueError('compare the same exact model and allocation settings')
    pulse = b['force_disturbance']
    cfg = pulse['config']
    stage = 'front' if cfg['phase'] == 'THREE_WHEEL_HOLD' else 'rear'
    sample = [r for r in read_rows(run, stage) if r['mode'] == cfg['phase']]
    reference = [r for r in read_rows(baseline, stage) if r['mode'] == cfg['phase']]
    if not sample or not reference or pulse['pulse_start_s'] is None:
        record['response'] = {'available': False}
        return record
    t0 = float(sample[0]['time_s'])
    time = np.array([float(r['time_s']) - t0 for r in sample])
    base_time = np.array([float(r['time_s']) - float(reference[0]['time_s']) for r in reference])
    start = pulse['pulse_start_s'] - t0
    end = start + 2 * cfg['ramp_s'] + cfg['hold_s']
    during = (time >= start) & (time <= end)
    after = (time >= end + .5) & (time <= end + 1.5)
    def delta(field, command=False):
        observed = np.array([float(r[('command_' if command else '') + field]) for r in sample])
        original = np.array([float(r[field]) for r in reference])
        return observed - np.interp(time, base_time, original)
    load = delta(f'fz_{cfg["corner"].lower()}_filtered_n')
    record['response'] = {
        'available': True,
        'scope': 'descriptive matched HOLD response, no new acceptance thresholds',
        'weak_corner': cfg['corner'],
        'peak_weak_load_departure_during_pulse_n': float(np.max(np.abs(load[during]))),
        'mean_abs_weak_load_departure_after_pulse_n': float(np.mean(np.abs(load[after]))) if any(after) else None,
        'after_window_seconds_from_pulse_end': [.5, 1.5],
        'peak_roll_departure_deg': float(np.max(np.abs(delta('roll_deg')[during]))),
        'max_control_force_departure_n': float(max(np.max(np.abs(delta(f'fact_{c}_n', command=True)[during]))
                                                   for c in ('fl', 'fr', 'rl', 'rr'))),
        'min_true_zmp_lambda_during_pulse': float(min(float(sample[i]['zmp_lambda_min'])
                                                    for i in np.flatnonzero(during))),
        'max_recorded_actual_bias_n': max(abs(float(r['disturbance_applied_n'])) for r in sample),
    }
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--runs', type=Path, nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    records = [compare(args.baseline, run) for run in args.runs]
    report = {'baseline': summarize(args.baseline), 'runs': records,
              'scope': 'matched tuned I_I force-bias tests; not broad disturbance certification'}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    for r in records:
        print(r['run_name'], r['result']['status'], r['response'])


if __name__ == '__main__':
    main()
