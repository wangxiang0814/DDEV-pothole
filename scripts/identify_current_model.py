"""Native four-contact identification of an existing full-cycle model copy.

Never replace historical evidence gains. Three-contact identification is separate.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import re
import shutil
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from ddevsim.cosim import run_stepwise
from ddevsim.interface_validation import IMPORT_NAMES
from ddevsim.static_wheel_lift.config import CURRENT_MODEL_PROBE
from ddevsim.static_wheel_lift.command_trace import cycle_hold_trace
from ddevsim.static_wheel_lift.support_geometry import assess_support
from ddevsim.static_wheel_lift.identification_settle import (
    IdentificationSettleLimits, assess_identification_settle)
from ddevsim.static_wheel_lift.quintic_trajectory import quintic_step
from ddevsim.static_wheel_lift.system_identification import assemble_settled_gains, validated_probe_window
from run_static_fr_m3 import response
from run_static_fr_closed_loop import EXPORTS

NAMES = ('FL', 'FR', 'RL', 'RR')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-model', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--contact-mode', choices=('FOUR_CONTACT', 'FR', 'RR'), default='FOUR_CONTACT')
    parser.add_argument('--reference-run', type=Path)
    args = parser.parse_args()
    source, output = args.source_model.resolve(), args.output.resolve()
    cfg = CURRENT_MODEL_PROBE
    text = (source / 'run_all.par').read_text(encoding='utf-8')
    imports = tuple(re.findall(r'(?m)^IMPORT\s+(\S+)', text))
    expected = IMPORT_NAMES + ('IMP_STEER_SW',)
    if imports != expected or not re.search(r'(?m)^VEHICLE_CODE i_i$', text):
        raise ValueError('expected current I_I full-cycle model and verified nine-input order')
    if output.exists():
        raise FileExistsError(output)
    output.mkdir(parents=True)
    source_hash = hashlib.sha256((source / 'run_all.par').read_bytes()).hexdigest()
    replay, freeze_s = None, 0.0
    if args.contact_mode != 'FOUR_CONTACT':
        if args.reference_run is None:
            raise ValueError('swing identification requires a successful reference run')
        ref = args.reference_run.resolve()
        if json.loads((ref / 'result.json').read_text(encoding='utf-8'))['status'] != 'PASS':
            raise ValueError('reference full cycle must pass')
        if hashlib.sha256((ref / 'model/run_all.par').read_bytes()).hexdigest() != source_hash:
            raise ValueError('reference model does not match selected source')
        logs = ['front_control_20ms.csv'] + (['rear_control_20ms.csv'] if args.contact_mode == 'RR' else [])
        command_rows = []
        for log in logs:
            with (ref / log).open(encoding='utf-8', newline='') as stream:
                command_rows.extend(csv.DictReader(stream))
        # At the stage handoff, the later logged command takes precedence.
        unique = {float(r['time_s']): r for r in command_rows}
        replay, freeze_s = cycle_hold_trace([unique[t] for t in sorted(unique)],
                                             args.contact_mode, hold_age_s=cfg.hold_age_s)
    observations = {}
    limits = IdentificationSettleLimits()
    baseline = None
    for name, corner, sign in [('baseline', None, 0)] + [
            (f'{wheel}_{suffix}', i, sign) for i, wheel in enumerate(NAMES)
            for sign, suffix in ((1, 'pos'), (-1, 'neg'))]:
        directory = output / name
        model = directory / 'model'
        shutil.copytree(source, model, ignore=shutil.ignore_patterns('output'))
        (model / 'output').mkdir()
        content, count = re.subn(r'(?m)^TSTOP\s+[-+0-9.eE]+\s*$',
                                 f'TSTOP {freeze_s + cfg.duration_s:g}', text)
        if count != 2:
            raise ValueError('expected two native stop-time declarations')
        (model / 'run_all.par').write_text(content, encoding='utf-8')

        def command(t, _exports, i=corner, multiplier=sign):
            inputs = list(replay.at(t)) if replay is not None else [0.0] * 9
            if i is not None:
                phase = quintic_step(t, freeze_s + cfg.ramp_start_s, cfg.ramp_duration_s)[0]
                inputs[4 + i] += multiplier * cfg.amplitude_n * phase
            return tuple(inputs)

        csv_path = directory / 'native_5ms.csv'
        native = run_stepwise(model / 'simfile.sim', command, csv_path,
                              expected, EXPORTS, log_decimation=cfg.log_decimation)
        with csv_path.open(encoding='utf-8', newline='') as stream:
            rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]
        window_start, window_end = freeze_s + cfg.window_start_s, freeze_s + cfg.window_end_s
        try:
            tail = validated_probe_window(native, rows, start_s=window_start,
                                          end_s=window_end,
                                          min_samples=2 * limits.min_samples_per_window)
        except ValueError as error:
            record = {'native': native, 'settle_status': 'INCOMPLETE_NATIVE_WINDOW',
                      'reason': str(error)}
            if name == 'baseline':
                baseline = record
            else:
                observations[name] = record
            (output / 'progress.json').write_text(json.dumps(
                {'baseline': baseline, 'cases': observations}, indent=2) + '\n', encoding='utf-8')
            result = {'status': 'FAIL', 'reason': f'{name}: {error}'}
            break
        settled = assess_identification_settle(rows, start_s=window_start,
                                               limits=limits)
        min_load = min(r[f'exp_Fz_{w}'] for r in tail for w in ('L1', 'R1', 'L2', 'R2'))
        record = {'native': native, 'settle_status': settled.status,
                  'settle_metrics': settled.metrics, 'minimum_wheel_load_n': min_load,
                  'response': response(csv_path, window_start, window_end)}
        record['response']['wheel_height_m'] = [float(np.mean([
            row[f'exp_Z_{w}'] - cfg.tyre_radius_m for row in tail])) for w in ('L1', 'R1', 'L2', 'R2')]
        record['maximum_attitude_deg'] = max(abs(row[f'exp_{field}']) for row in tail for field in ('Roll_E', 'Pitch'))
        record['travel_range_mm'] = [min(row[f'exp_Jnc_{w}'] for row in tail for w in ('L1', 'R1', 'L2', 'R2')),
                                     max(row[f'exp_Jnc_{w}'] for row in tail for w in ('L1', 'R1', 'L2', 'R2'))]
        if (record['maximum_attitude_deg'] > cfg.attitude_limit_deg or
                record['travel_range_mm'][0] < cfg.travel_min_mm or
                record['travel_range_mm'][1] > cfg.travel_max_mm):
            record['settle_status'] = 'ATTITUDE_OR_TRAVEL_UNSAFE'
        if args.contact_mode == 'FOUR_CONTACT' and min_load <= 0:
            record['settle_status'] = 'CONTACT_MODE_CHANGED'
        if args.contact_mode != 'FOUR_CONTACT':
            wheel_map = dict(zip(NAMES, ('L1', 'R1', 'L2', 'R2')))
            support = [n for n in NAMES if n != args.contact_mode]
            lifted = wheel_map[args.contact_mode]
            margin = []
            for row in tail:
                contacts = {n: (row[f'exp_Xctc_{w}i'], row[f'exp_Yctc_{w}i']) for n, w in wheel_map.items()}
                loads = {n: max(0., row[f'exp_Fz_{w}']) for n, w in wheel_map.items()}
                com = (row['exp_XCG_TM'], row['exp_YCG_TM'])
                geometry = assess_support(contacts, loads, com, cfg.lambda_safe, lifted_corner=args.contact_mode)
                # The same triangle evaluates both pressure centre and projected CoM.
                triangle = [contacts[n] for n in support]
                com_lambda = np.linalg.solve(np.vstack((np.asarray(triangle).T, np.ones(3))), [*com, 1.])
                margin.append(min(geometry.lambda_min, float(min(com_lambda))))
            record['minimum_support_load_n'] = min(row[f'exp_Fz_{wheel_map[n]}'] for row in tail for n in support)
            record['maximum_lifted_load_n'] = max(row[f'exp_Fz_{lifted}'] for row in tail)
            record['minimum_clearance_m'] = min(row[f'exp_Z_{lifted}'] - cfg.tyre_radius_m for row in tail)
            record['minimum_support_margin'] = min(margin)
            if (record['minimum_support_load_n'] < cfg.swing_support_floor_n or
                    record['maximum_lifted_load_n'] > cfg.swing_unloaded_n or
                    record['minimum_clearance_m'] < cfg.swing_clearance_m or
                    record['minimum_support_margin'] < cfg.lambda_safe):
                record['settle_status'] = 'CONTACT_MODE_OR_SUPPORT_UNSAFE'
        if name == 'baseline':
            baseline = record
        else:
            observations[name] = record
        (output / 'progress.json').write_text(json.dumps(
            {'baseline': baseline, 'cases': observations}, indent=2) + '\n', encoding='utf-8')
        print(f'{name}: {record["settle_status"]}', flush=True)
        if native['status'] != 'COMPLETED' or record['settle_status'] != 'SETTLED':
            result = {'status': 'FAIL', 'reason': f'unusable probe: {name}'}
            break
    else:
        result = {'status': 'PASS', **assemble_settled_gains(
            observations, amplitude_n=cfg.amplitude_n)}
    result.update({'mode': args.contact_mode, 'source_model_sha256': source_hash,
                   'probe_model_sha256': hashlib.sha256(
                       (output / 'baseline/model/run_all.par').read_bytes()).hexdigest(),
                   'source_model': str(source), 'config': asdict(cfg),
                   'settle_limits': asdict(limits), 'baseline': baseline,
                   'cases': observations,
                   'freeze_s': freeze_s,
                   'reference_run': str(args.reference_run) if replay is not None else None,
                   'scope': 'local frozen-input mean around the recorded contact mode; not global gains'})
    (output / 'gain_matrix.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items()
                      if k in ('status', 'reason', 'G_F_diagnostic')}, indent=2))


if __name__ == '__main__':
    main()
