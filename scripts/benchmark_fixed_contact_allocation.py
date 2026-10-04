"""Compare equivalent fixed-contact balance with a versioned generic QP."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from ddevsim.static_wheel_lift.load_allocator import allocate_loads


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-ref', default='cb4077b')
    parser.add_argument('--samples', type=int, default=100)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.samples < 10:
        raise ValueError('at least ten timing samples required')
    source = subprocess.check_output(['git', 'show',
        f'{args.baseline_ref}:src/ddevsim/static_wheel_lift/load_allocator.py'], cwd=ROOT)
    scratch = ROOT / 'tmp/fixed_balance_baseline.py'
    scratch.parent.mkdir(exist_ok=True)
    scratch.write_bytes(source)
    spec = importlib.util.spec_from_file_location('fixed_balance_baseline', scratch)
    baseline = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = baseline
    spec.loader.exec_module(baseline)
    results = {}
    for corner in ('FR', 'RR'):
        idx = ['FL', 'FR', 'RL', 'RR'].index(corner)
        xy = np.array([[101., .8], [101., -.8], [98., .8], [98., -.8]])
        desired = np.array([6000., 2500., 1400., 5500.])
        desired[idx] = 0.
        total = desired.sum()
        low, high = np.full(4, 500.), np.full(4, 12000.)
        low[idx] = high[idx] = 0.
        kwargs = dict(lifted_corner=corner, lift_target_n=0., lower_n=low, upper_n=high)
        timings = {'generic': [], 'direct': []}
        error = 0.
        for i in range(args.samples + 5):
            outputs = {}
            order = ('generic', 'direct') if i % 2 else ('direct', 'generic')
            for name in order:
                fn = baseline.allocate_loads if name == 'generic' else allocate_loads
                start = time.perf_counter()
                out = fn(xy, total, desired @ xy / total, desired, **kwargs)
                elapsed = (time.perf_counter() - start) * 1000.
                if out.status != 'OPTIMAL':
                    raise ValueError('benchmark case failed: ' + name)
                outputs[name] = out.fz_ref_n
                if i >= 5:
                    timings[name].append(elapsed)
            error = max(error, float(np.max(abs(outputs['generic'] - outputs['direct']))))
        if error > 1e-5:
            raise ValueError('fixed balance outputs differ')
        results[corner] = {'maximum_force_difference_n': error, **{
            name: {'median_ms': float(np.median(values)),
                   'p95_ms': float(np.percentile(values, 95)), 'max_ms': float(max(values))}
            for name, values in timings.items()}}
    report = {'baseline_ref': args.baseline_ref,
              'baseline_source_sha256': hashlib.sha256(source).hexdigest(),
              'direct_source_sha256': hashlib.sha256((ROOT / 'src/ddevsim/static_wheel_lift/load_allocator.py').read_bytes()).hexdigest(),
              'samples_per_corner_per_solver': args.samples,
              'scope': 'interleaved same-process upper allocator timing; no hard realtime guarantee',
              'results': results}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
