"""Recompute diagnostic metrics and plot local depth-sweep evidence."""
from dataclasses import fields
import csv
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from run_shallow_depth_baseline import ROOT, Baseline, summarize
from ddevsim.pothole_case import PotholeScenario


def main():
    output = ROOT / 'evidence/shallow_depth_baseline_20261008'
    output.mkdir(exist_ok=True)
    results = []
    for path in sorted((ROOT / 'runs/shallow_press').glob('*/depth_baseline_20261008_*/baseline_diagnostic.json')):
        run = path.parent
        original = json.loads(path.read_text(encoding='utf-8'))
        manifest = json.loads((run / 'run_manifest.json').read_text(encoding='utf-8'))
        scene = json.loads((run / 'model/scenario.json').read_text(encoding='utf-8'))
        options = manifest['diagnostic_options']
        geometry = PotholeScenario(**{f.name: scene[f.name] for f in fields(PotholeScenario)})
        case = SimpleNamespace(case_id=scene['case_id'], geometry=geometry)
        control = Baseline(manifest['export_names'], options['speed_kph'], options['torque_limit_nm'])
        control.abort_reason = original['abort_reason']
        result = summarize(run / 'native_5ms.csv', case, control, original['native'])
        result.update(run_id=run.name, run_path=str(run.relative_to(ROOT)),
                      options=options, vehicle_sha256=scene['vehicle']['vehicle_sha256'],
                      native_csv_sha256=hashlib.sha256((run / 'native_5ms.csv').read_bytes()).hexdigest())
        result['wheel_contact_loss_sum_s'] = sum(w['contact_loss_s'] for w in result['wheels'].values())
        result['jounce_peak_mm'] = max(w['jounce_max_mm'] for w in result['wheels'].values())
        results.append(result)
    statics = []
    for path in sorted((ROOT / 'runs/shallow_press/right_track').glob('reach_probe_20261008_*/baseline_diagnostic.json')):
        d = json.loads(path.read_text(encoding='utf-8'))
        statics.append({'run_id': path.parent.name, 'settled_jounce_mm': d['settled_jounce_mm'],
                        'wheels': d['wheels'], 'roll_peak_deg': d['roll_peak_deg'],
                        'pitch_peak_deg': d['pitch_peak_deg'], 'abort_reason': d['abort_reason']})
    report = {'scope': 'finite native diagnostics; no expert task PASS', 'results': results,
              'static_force_probes': statics, 'chassis_contact': 'NOT_OBSERVABLE',
              'metric_code_sha256': hashlib.sha256((ROOT / 'scripts/run_shallow_depth_baseline.py').read_bytes()).hexdigest()}
    (output / 'summary.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    keys = ['case', 'run_id', 'depth_m', 'length_m', 'width_m', 'edge_transition_m',
            'target_speed_kph', 'rear_cleared', 'speed_min_kph', 'speed_max_kph',
            'speed_rmse_kph', 'roll_peak_deg', 'pitch_peak_deg', 'az_peak_m_s2',
            'az_rms_m_s2', 'jounce_peak_mm', 'wheel_contact_loss_sum_s',
            'lateral_peak_m', 'body_vertical_drop_m', 'torque_peak_nm', 'torque_saturation_s']
    with (output / 'summary.csv').open('w', newline='', encoding='utf-8-sig') as f:
        writer = csv.DictWriter(f, keys)
        writer.writeheader()
        writer.writerows({k:r[k] for k in keys} for r in results)
    primary = [r for r in results if r['run_id'].startswith('depth_baseline_20261008_d')]
    fig, axes = plt.subplots(2, 3, figsize=(13, 7.5), constrained_layout=True)
    metrics = [('speed_min_kph', 'Minimum speed (km/h)'), ('az_peak_m_s2', 'Peak sprung acceleration (m/s²)'),
               ('jounce_peak_mm', 'Maximum jounce (mm)'), ('wheel_contact_loss_sum_s', 'Summed wheel contact loss (wheel·s)'),
               ('roll_peak_deg', 'Peak absolute roll (deg)'), ('body_vertical_drop_m', 'Body origin drop (m)')]
    for ax, (key, label) in zip(axes.flat, metrics):
        for case, color in [('right_track', '#2563eb'), ('wide', '#dc2626')]:
            data = sorted([r for r in primary if r['case'] == case], key=lambda r:r['depth_m'])
            ax.plot([r['depth_m']*1000 for r in data], [r[key] for r in data], 'o-', color=color, label=case)
        ax.axvline(100, linestyle='--', color='#64748b', linewidth=1)
        ax.axvline(150, linestyle=':', color='#64748b', linewidth=1)
        if key == 'jounce_peak_mm':
            ax.axhline(155, linestyle='--', color='#92400e', linewidth=1)
        ax.set(xlabel='Configured depth (mm)', ylabel=label)
        ax.grid(alpha=.25)
    axes[0,0].legend()
    fig.suptitle('Passive suspension, equal torque PI: 3 km/h target, 0.8 m pit, 0.05 m edges\nDashed 100 mm: provisional research split; dotted 150 mm: reach warning. Neither is a validated safety limit.', fontsize=11)
    fig.savefig(output / 'depth_sweep.png', dpi=170)
    fig.savefig(output / 'depth_sweep.svg')
    svg = output / 'depth_sweep.svg'
    svg.write_text('\n'.join(line.rstrip() for line in svg.read_text(encoding='utf-8').splitlines()) + '\n', encoding='utf-8')
    plt.close(fig)
    print('Crossing diagnostics:', len(results), 'primary:', len(primary), 'static force probes:', len(statics))
    print('Vehicle identities:', sorted({r['vehicle_sha256'] for r in results}))


if __name__ == '__main__':
    main()
