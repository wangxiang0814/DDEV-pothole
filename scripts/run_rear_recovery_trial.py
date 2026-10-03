"""Inject one pit-front RR preload abort; keep normal cycle acceptance FAIL."""
from __future__ import annotations

import csv
import json
from pathlib import Path
import sys

import run_right_side_full_cycle as cycle
from ddevsim.static_wheel_lift.config import RECOVERY_PRELOAD_ABORT_AFTER_S


def inject_preload_abort(rear, now_s, *, after_s):
    if (rear.mode != 'RR_PRELOAD' or rear.mode_start_s is None or
            now_s - rear.mode_start_s < after_s):
        return False
    rear.abort_reason = 'injected pit-front preload abort'
    rear._enter('RR_ABORT_STOP', now_s)
    return True


class RecoveryTrialController(cycle.FullRightSideController):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.fault_injected = False

    def __call__(self, now_s, exports):
        commands = super().__call__(now_s, exports)
        if self.rear_started and not self.fault_injected:
            self.fault_injected = inject_preload_abort(
                self.rear, now_s, after_s=RECOVERY_PRELOAD_ABORT_AFTER_S)
        return commands


def main():
    if '--output' not in sys.argv:
        raise ValueError('supply --output and the normal full-cycle trial options')
    output = Path(sys.argv[sys.argv.index('--output') + 1]).resolve()
    original = cycle.FullRightSideController
    cycle.FullRightSideController = RecoveryTrialController
    try:
        cycle.main()
    finally:
        cycle.FullRightSideController = original
    result = json.loads((output / 'result.json').read_text(encoding='utf-8'))
    with (output / 'rear_control_20ms.csv').open(encoding='utf-8', newline='') as stream:
        rows = list(csv.DictReader(stream))
    aborts = [r for r in rows if r['mode'] == 'RR_ABORT_STOP']
    recovered = rows and rows[-1]['mode'] == 'RR_COMPLETE'
    abort_index = next((i for i, r in enumerate(rows) if r['mode'] == 'RR_ABORT_STOP'), None)
    after = rows[max(0, abort_index - 1):] if abort_index is not None else []
    report = {
        'recovery_status': 'PASS' if (aborts and recovered and result['status'] == 'FAIL' and
            result['rr_abort_reason'] == 'injected pit-front preload abort' and
            result['criteria']['four_wheel_recovered']) else 'FAIL',
        'task_status': result['status'], 'abort_reason': result['rr_abort_reason'],
        'injection': {'phase': 'RR_PRELOAD', 'after_s': RECOVERY_PRELOAD_ABORT_AFTER_S},
        'abort_start_s': float(aborts[0]['time_s']) if aborts else None,
        'final_mode': rows[-1]['mode'] if rows else None,
        'abort_max_abs_vx_kph': max((abs(float(r['vx_kph'])) for r in after), default=None),
        'abort_max_actuator_step_n': max((
            abs(float(b[f'fact_{c}_n']) - float(a[f'fact_{c}_n']))
            for a, b in zip(after, after[1:]) for c in ('fl', 'fr', 'rl', 'rr')), default=None),
        'four_wheel_recovered': result['criteria']['four_wheel_recovered'],
        'scope': 'injected phase abort before pit; not support-loss or over-pit recovery validation',
    }
    (output / 'recovery_report.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
