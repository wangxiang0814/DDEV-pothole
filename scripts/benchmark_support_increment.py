"""Interleaved equivalent-solver benchmark; frozen synthetic contract cases."""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from ddevsim.static_wheel_lift.config import SUPPORT_QP
from ddevsim.static_wheel_lift.suspension_allocator import allocate_support_increment


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', type=int, default=100)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.samples < 10:
        raise ValueError('at least 10 samples required')
    outcomes = {}
    for corner, permutation in [('FR', [0, 1, 2, 3]), ('RR', [0, 3, 2, 1])]:
        p = np.asarray(permutation)
        loads = np.array([6000., 0., 1000., 7000.])[p]
        xy = np.array([[1., 1.], [1., -1.], [-1., 1.], [-1., -1.]])[p]
        gf = np.array([[-1., .2, 1., 0.], [0., 0., 0., 0.],
                       [1., -.2, -1., -1.], [0., 0., 0., 1.]])[p][:, p]
        for name, multiplier in [('interior', .01), ('force_box_active', 100.)]:
            target = loads + multiplier * (np.array([-100., 0., 200., -100.])[p])
            zmp = loads @ xy / loads.sum()
            kwargs = dict(lifted_corner=corner, contacts_xy=xy, fz_n=loads,
                fz_target_n=target, com_xy=zmp, zmp_xy=zmp, attitude_rad=np.zeros(2),
                attitude_target_rad=np.zeros(2), travel_m=np.zeros(4),
                wheel_height_m=np.array([0., .04, 0., 0.])[p],
                gains={'Fz_n': gf, 'CoM_xy_m': np.zeros((2, 4)),
                       'ZMP_xy_m': np.zeros((2, 4)), 'attitude_rad': np.zeros((2, 4)),
                       'travel_m': np.zeros((4, 4)), 'wheel_height_m': np.zeros((4, 4))},
                base_force_n=np.zeros(4), previous_applied_n=np.zeros(4),
                correction_n=np.zeros(3), dt_s=.02)
            timings = {'iterative': [], 'direct_with_fallback': []}
            error = 0.
            modes = {}
            for i in range(args.samples + 5):
                outputs = {}
                order = list(timings) if i % 2 else list(timings)[::-1]
                for method in order:
                    cfg = replace(SUPPORT_QP, direct_feasible_solve=method != 'iterative')
                    begin = time.perf_counter()
                    result = allocate_support_increment(**kwargs, config=cfg)
                    duration = (time.perf_counter() - begin) * 1000.
                    if result.status != 'OPTIMAL':
                        raise ValueError(f'benchmark not optimal: {corner}/{name}/{method}')
                    outputs[method] = result
                    if i >= 5:
                        timings[method].append(duration)
                        if method != 'iterative':
                            modes[result.solver] = modes.get(result.solver, 0) + 1
                error = max(error, float(np.max(abs(outputs['iterative'].force_n -
                                                    outputs['direct_with_fallback'].force_n))))
            if error > .01:
                raise ValueError('equivalent solvers differ by more than 0.01 N')
            outcomes[f'{corner}_{name}'] = {'maximum_force_difference_n': error,
                'selected_solvers': modes, **{method: {'median_ms': float(np.median(values)),
                    'p95_ms': float(np.percentile(values, 95)), 'max_ms': float(max(values))}
                    for method, values in timings.items()}}
    report = {'scope': 'Interleaved same-process synthetic three-input contract cases, '
               'not plant performance or hard realtime certification.',
              'samples_per_case_per_solver': args.samples,
              'allocator_source_sha256': hashlib.sha256((ROOT / 'src/ddevsim/static_wheel_lift/suspension_allocator.py').read_bytes()).hexdigest(),
              'config': vars(SUPPORT_QP), 'results': outcomes}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
