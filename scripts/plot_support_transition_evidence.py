"""Compare RR truth trajectories of two specified native full-cycle runs."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def read_run(path):
    result = json.loads((path / 'result.json').read_text(encoding='utf-8'))
    with (path / 'rear_control_20ms.csv').open(encoding='utf-8', newline='') as stream:
        rows = list(csv.DictReader(stream))
    selected = [r for r in rows if r['mode'] in ('RR_PRELOAD', 'RR_LIFTING', 'RR_POSTURE',
                                               'RR_HOLD', 'RR_CRAWL', 'RR_STOP')]
    begin = float(selected[0]['time_s'])
    return result, selected, np.array([float(r['time_s']) - begin for r in selected])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--staged', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    datasets = [(path, label, style, read_run(path)) for path, label, style in (
        (args.baseline, 'hold-only', '--'), (args.staged, 'staged', '-'))]
    first, second = datasets[0][3][0], datasets[1][3][0]
    if (first['scenario'] != second['scenario'] or
            first['measurement_noise']['config']['seed'] != second['measurement_noise']['config']['seed'] or
            hashlib.sha256((args.baseline / 'model/run_all.par').read_bytes()).digest() !=
            hashlib.sha256((args.staged / 'model/run_all.par').read_bytes()).digest()):
        raise ValueError('comparison requires identical scenario, source model and noise seed')
    fig, axes = plt.subplots(3, 2, figsize=(11, 9), sharex=True)
    a = axes.ravel()
    labels = ('Abs. roll (deg)', 'Minimum support load (N)', 'Min. ZMP / CoM barycentric coordinate',
              'RR clearance above pit top (mm)', 'Lateral error from shared path (cm)', 'Support force correction (N)')
    for path, label, style, data in datasets:
        result, rows, t = data
        a[0].plot(t, [abs(float(r['roll_deg'])) for r in rows], style, label=label)
        a[1].plot(t, [float(r['min_support_n']) for r in rows], style, label=label)
        a[2].plot(t, [min(float(r['zmp_lambda_min']), float(r['com_lambda_min'])) for r in rows], style, label=label)
        a[3].plot(t, [1000 * float(r['rr_clearance_m']) for r in rows], style, label=label)
        a[4].plot(t, [100 * (float(r['yo_m']) - result['path_reference_yo_m']) for r in rows], style, label=label)
        for c, colour in zip(('fl', 'fr', 'rl'), ('tab:blue', 'tab:orange', 'tab:green')):
            a[5].plot(t, [float(r.get(f'allocation_corr_{c}_n', 0.)) for r in rows], style,
                      color=colour, linewidth=1., label=f'{label} {c.upper()}')
        if label == 'staged':
            for mode in ('RR_LIFTING', 'RR_POSTURE', 'RR_HOLD', 'RR_CRAWL', 'RR_STOP'):
                i = next((i for i, r in enumerate(rows) if r['mode'] == mode), None)
                if i is not None:
                    for ax in a:
                        ax.axvline(t[i], color='0.8', linewidth=.7)
                    a[0].text(t[i], .95, mode[3:], transform=a[0].get_xaxis_transform(),
                              rotation=90, va='top', fontsize=7)
    for ax, title in zip(a, labels):
        ax.set_title(title, fontsize=10)
        ax.grid(alpha=.25)
        ax.legend(fontsize=8)
    a[0].axhline(10., color='red', alpha=.5, linewidth=.8)
    a[1].axhline(500., color='red', alpha=.5, linewidth=.8)
    a[2].axhline(.05, color='red', alpha=.5, linewidth=.8)
    a[3].axhline(10., color='red', alpha=.5, linewidth=.8)
    for ax in a[-2:]:
        ax.set_xlabel('Time from RR preload start (s)')
    fig.suptitle('RR feedback comparison: plant truth, same low-friction scenario and seed\n'
                 'PRELOAD starts in four-contact mode; 10 mm clearance is required for hold / crossing.', fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, .94))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=150)
    plt.close(fig)
    print(args.output.resolve())


if __name__ == '__main__':
    main()
