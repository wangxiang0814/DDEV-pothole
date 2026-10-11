"""Audit immutable contact runs and produce a retrieval index and diagnostic plot."""
import hashlib
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analyze_speed_depth_baseline import read_channels

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'evidence/chassis_contact_20261011'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    if (OUT / 'verification.json').exists():
        raise FileExistsError('audit already exists; preserve previous evidence')
    history = json.loads((OUT / 'clearance_summary.json').read_text(encoding='utf-8'))
    for item in history['results']:
        run = ROOT / item['run_path']
        assert digest(run / 'native_5ms.csv') == item['csv_sha256']
    assert len(history['results']) == 90
    runs = []
    for path in sorted((ROOT / 'runs/shallow_press').glob('*/chassis*/contact_diagnostic.json')):
        run = path.parent
        result = json.loads(path.read_text(encoding='utf-8'))
        manifest = json.loads((run / 'run_manifest.json').read_text(encoding='utf-8'))
        options = manifest['diagnostic_options']
        assert result['native']['status'] == 'COMPLETED'
        assert result['native']['reached_configured_stop']
        assert not result['native']['error_message']
        assert result['native_reference_frame_error_m'] < 1e-6
        assert digest(run / 'model/run_all.par') == manifest['extended_model_sha256']
        # Earlier capability metadata did not distinguish disabled force input.
        # Preserve raw records and derive the actual enabled state from settings.
        actual = not options['contact_off']
        runs.append(dict(name=run.name, case=run.parent.name,
            run_path=str(run.relative_to(ROOT)), options=options,
            contact_config=manifest['contact_config'], result=result,
            contact_force_enabled=actual,
            metadata_erratum=('Legacy applied-force flag describes capability; contact_enabled and options determine application.'
                if result.get('normal_and_friction_forces_applied_to_sprung_reference_points') != actual else None),
            base_vehicle_sha256=manifest['base_vehicle_sha256'],
            extended_model_sha256=manifest['extended_model_sha256'],
            csv_sha256=digest(run / 'native_1ms.csv')))
    assert len(runs) == 9
    assert len(set(r['base_vehicle_sha256'] for r in runs)) == 1
    byname = {r['name']: r for r in runs}
    off = byname['chassis_check_20261011_wide_d200_off']
    old = read_channels(ROOT / 'runs/shallow_press/wide/speed_study_full30_20261011_v10_d200/native_5ms.csv')
    new = read_channels(ROOT / off['run_path'] / 'native_1ms.csv')
    exported = [k for k in old if k.startswith('exp_')]
    differences = {k: float(np.max(np.abs(old[k] - new[k]))) for k in exported}
    assert len(exported) == 126 and max(differences.values()) == 0
    zero = read_channels(ROOT / byname['chassis_forcecheck_20261011_zero']['run_path'] / 'native_1ms.csv')
    probe = read_channels(ROOT / byname['chassis_forcecheck_20261011_up1000']['run_path'] / 'native_1ms.csv')
    pulse = byname['chassis_forcecheck_20261011_up1000']['result']['impulse_world_ns'][2]
    rise = float(np.max(probe['exp_Zo'] - zero['exp_Zo']))
    assert abs(pulse - 100) < 1e-6 and rise > .001
    on = [r for r in runs if r['contact_force_enabled']]
    assert len(on) == 6 and all(r['result']['friction_work_j'] <= 0 for r in on)
    verification = dict(scope='PARAMETRIC_DIAGNOSTIC_NOT_REAL_VEHICLE_CERTIFICATION',
        historical_rows_verified=90, native_runs=9, full_30s_runs=7, short_interface_probes=2,
        original_export_count=126, zero_force_max_difference=max(differences.values()),
        force_probe_impulse_ns=pulse, force_probe_origin_rise_m=rise,
        maximum_native_reference_error_m=max(r['result']['native_reference_frame_error_m'] for r in runs),
        code_identity_at_audit={str(p.relative_to(ROOT)): digest(p) for p in [
            Path(__file__).resolve(), ROOT/'scripts/analyze_chassis_clearance.py',
            ROOT/'scripts/run_chassis_contact_diagnostic.py', ROOT/'src/ddevsim/chassis_contact.py']},
        provenance_note='Audit hashes identify current code; original run manifests and records are preserved, including legacy metadata errata.')
    (OUT / 'contact_runs.json').write_text(json.dumps(runs, indent=2), encoding='utf-8')
    lines = ['# Contact diagnostic run index', '',
        'Assumed underside geometry and uncalibrated contact parameters. No physical safety certification.', '',
        '| Case | Run name | Depth cm | Clearance cm | Contact | Duration s | Rear cleared |',
        '|---|---|---:|---:|---|---:|---|']
    for r in runs:
        o = r['options']; z = r['result']
        lines.append('| %s | `%s` | %.0f | %.0f | %s | %.0f | %s |' % (
            r['case'], r['name'], o['depth_m']*100, o['clearance_m']*100,
            r['contact_force_enabled'], z['native']['final_time_s'], z['rear_cleared']))
    lines += ['', 'The two 6-second runs only validate the force interface. Full parameters and immutable CSV hashes: contact_runs.json.',
        'Legacy applied-force capability flags in disabled runs are corrected in this index; original records are retained.']
    (OUT / 'case_index.md').write_text('\n'.join(lines), encoding='utf-8')
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.6))
    for case in ['right_track', 'wide']:
        rows = [r for r in history['results'] if r['case']==case and abs(r['nominal_static_clearance_m']-.15)<1e-9 and '_v10_' in r['run_path']]
        rows.sort(key=lambda r:r['depth_m'])
        axes[0].plot([r['depth_m']*100 for r in rows], [r['min_gap_m']*1000 for r in rows], 'o-', label=case)
    axes[0].axhline(0, color='k', linewidth=.8)
    axes[0].set(xlabel='Pit depth (cm)', ylabel='Minimum geometric gap (mm)', title='Historical trajectory; assumed clearance 15 cm')
    axes[0].text(.03, .03, 'After intersection: counterfactual trajectory', transform=axes[0].transAxes, fontsize=8)
    names = [off['name'], 'chassis_contact_20261011_wide_d200_h150',
        'chassis_contact_20261011_wide_d200_h150_grid45', 'chassis_contact_20261011_wide_d200_h150_stiff10']
    for name, label in zip(names, ['No force', '15 points / 1 MN/m', '45 points / 1 MN/m', '15 points / 10 MN/m']):
        v = read_channels(ROOT / byname[name]['run_path'] / 'native_1ms.csv')
        take = (v['time_s'] >= 10.8) & (v['time_s'] <= 16)
        axes[1].plot(v['time_s'][take], v['exp_Pitch'][take], label=label)
    axes[1].set(xlabel='Time (s)', ylabel='Pitch (deg)', title='Wide pit 20 cm: uncalibrated contact sensitivity')
    for ax in axes:
        ax.legend(fontsize=8); ax.grid(alpha=.25)
    fig.tight_layout(); fig.savefig(OUT / 'clearance_and_contact.png', dpi=160); plt.close(fig)
    (OUT / 'verification.json').write_text(json.dumps(verification, indent=2), encoding='utf-8')
    print(json.dumps(verification, indent=2))


if __name__ == '__main__':
    main()
