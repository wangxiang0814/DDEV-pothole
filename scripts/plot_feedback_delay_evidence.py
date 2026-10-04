"""Plot the matched nominal 40 ms feedback-delay launch trials."""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs', type=Path, nargs=3, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    results = [json.loads((p / 'result.json').read_text(encoding='utf-8')) for p in args.runs]
    key = lambda r: (r['scenario'], r['measurement_noise']['config'],
                     r['support_allocation']['provenance']['model_sha256'])
    if any(key(r) != key(results[0]) for r in results[1:]):
        raise ValueError('compare only the same plant, scenario, noise and delay')
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), sharex=True)
    for path, result in zip(args.runs, results):
        with (path / 'front_control_20ms.csv').open(encoding='utf-8', newline='') as stream:
            rows = list(csv.DictReader(stream))
        start = next(float(r['time_s']) for r in rows if r['mode'] == 'CRAWL')
        rows = [r for r in rows if start <= float(r['time_s']) <= start + 5.]
        time = [float(r['time_s']) - start for r in rows]
        ramp = result['controller_config']['front']['crawl_accel_ramp_s']
        label = f'{ramp:g} s launch: task {result["status"]}'
        values = ([float(r['fr_top_clearance_m']) * 1000 for r in rows],
                  [float(r['vx_kph']) for r in rows],
                  [float(r['min_support_n']) for r in rows],
                  [float(r['force_corr_fl_n']) for r in rows])
        for axis, data in zip(axes.flat, values):
            axis.plot(time, data, label=label)
    for axis, title, unit in zip(axes.flat,
        ('FR clearance: launch transient', 'Vehicle speed', 'Weakest supporting load',
         'Legacy FL load feedback correction'), ('mm', 'km/h', 'N', 'N')):
        axis.set_title(title)
        axis.set_ylabel(unit)
        axis.set_xlabel('Seconds after FR CRAWL entry')
        axis.grid(alpha=.25)
        axis.legend(fontsize=8)
    axes[0, 0].axhline(5, color='red', linestyle='--', linewidth=1, label='pre-pit/lip guard')
    axes[1, 0].axhline(500, color='red', linestyle='--', linewidth=1)
    axes[1, 1].axhline(-500, color='red', linestyle='--', linewidth=1)
    fig.suptitle('Native plant truth; matched nominal model / seed; 40 ms observation delay')
    fig.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    main()
