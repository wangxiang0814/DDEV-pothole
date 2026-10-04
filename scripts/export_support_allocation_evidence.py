"""Export native allocation comparisons and audit the injected recovery handoff."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

from summarize_right_side_matrix import summarize


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs', type=Path, nargs='+', required=True)
    parser.add_argument('--recovery', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    records = [summarize(p) for p in args.runs]
    for p, record in zip(args.runs, records):
        peaks = {}
        for stage in ('front', 'rear'):
            with (p / f'{stage}_control_20ms.csv').open(encoding='utf-8', newline='') as stream:
                rows = list(csv.DictReader(stream))
            peaks[stage] = max((abs(float(r.get(f'allocation_corr_{c}_n', 0.)))
                                for r in rows for c in ('fl', 'fr', 'rl', 'rr')), default=0.)
        record['max_allocation_correction_n'] = peaks
    with (args.recovery / 'rear_control_20ms.csv').open(encoding='utf-8', newline='') as stream:
        rows = list(csv.DictReader(stream))
    i = next(i for i, r in enumerate(rows) if r['mode'] == 'RR_ABORT_STOP')
    before, after = rows[i-1], rows[i]
    recovery = json.loads((args.recovery / 'recovery_report.json').read_text(encoding='utf-8'))
    recovery.update({
        'run_name': args.recovery.name,
        'allocation_correction_before_abort_n': {c: float(before[f'allocation_corr_{c}_n'])
                                               for c in ('fl', 'fr', 'rl', 'rr')},
        'snapshot_handoff_max_step_n': max(abs(float(after[f'fact_{c}_n']) - float(before[f'fact_{c}_n']))
                                           for c in ('fl', 'fr', 'rl', 'rr')),
        'outer_correction_after_abort_n': {c: float(after[f'allocation_corr_{c}_n'])
                                           for c in ('fl', 'fr', 'rl', 'rr')},
        'model_run_all_sha256': hashlib.sha256((args.recovery / 'model/run_all.par').read_bytes()).hexdigest(),
    })
    report = {'scope': 'each tuned I_I run preserves its own exact model and config; truth acceptance',
              'historical_monitor_scope': 'Only historical support_qp_monitor_nominal predates the wheel-height constraint; later transition monitors preserve their own validated height model.',
              'runs': records, 'recovery': recovery}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    for r in records:
        a = r['result'].get('support_allocation', {})
        print(r['run_name'], r['result']['status'], r['cycle_complete_s'],
              r['shared_path_audit']['maximum_lateral_m'], a.get('solve_p95_ms'))
    print(json.dumps(recovery, indent=2))


if __name__ == '__main__':
    main()
