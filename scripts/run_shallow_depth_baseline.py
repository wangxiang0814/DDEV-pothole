"""Native diagnostic only: equal wheel speed-regulating torque, passive suspension.

This is not either expert strategy, and never declares task PASS or chassis
clearance. Each invocation is a separate native process and exclusive run.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from ddevsim.cosim import run_stepwise
from ddevsim.runtime.run_workspace import claim_run, write_run_manifest, sha256_file
from ddevsim.scenarios.pit import load_case, validate_case
from ddevsim.scenarios.builder import prepare_case

WHEELS = ('L1', 'R1', 'L2', 'R2')


class Baseline:
    def __init__(self, names, speed_kph=3., torque_limit=250., probe_force=0., probe_corner='all', probe_ramp_s=10.):
        self.index = {name: i for i, name in enumerate(names)}
        self.speed = speed_kph / 3.6
        self.limit = torque_limit
        self.probe = probe_force
        self.corner = probe_corner
        self.probe_ramp_s = probe_ramp_s
        self.integral = 0.
        self.torque = 0.
        self.last_tick = -1.
        self.command = (0.,) * 9
        self.completed = False
        self.abort_reason = None
        self.end_station_m = 103.

    def __call__(self, t, exports):
        if t - self.last_tick < .02 - 1e-8:
            return self.command
        dt = .02 if self.last_tick < 0 else t - self.last_tick
        self.last_tick = t
        x = lambda name: float(exports[self.index[name]])
        if not all(math.isfinite(float(v)) for v in exports):
            raise ValueError('nonfinite native feedback')
        if abs(x('Roll_E')) > 20 or abs(x('Pitch')) > 20:
            self.abort_reason = 'attitude exceeds diagnostic 20 degree stop'
        self.completed = min(x('X_L2'), x('X_R2')) > self.end_station_m
        target = self.speed * min(1., max(0., (t - 5.) / 3.))
        if self.completed or self.abort_reason:
            target = 0.
        error = target - x('Vx') / 3.6
        requested = 220. * error + 100. * self.integral
        # Anti-windup; same tuning and equal torque on all four wheels.
        if abs(requested) < self.limit or requested * error < 0:
            self.integral += error * dt
        requested = float(np.clip(220. * error + 100. * self.integral, -self.limit, self.limit))
        self.torque += float(np.clip(requested - self.torque, -500. * dt, 500. * dt))
        if t < 5. and not self.probe:
            self.torque = 0.
            self.integral = 0.
        force = self.probe * min(1., max(0., (t - 5.) / self.probe_ramp_s))
        forces = [force if self.corner in ('all', w) else 0. for w in WHEELS]
        self.command = (self.torque,) * 4 + tuple(forces) + (0.,)
        return self.command


def longest(mask, dt=.005):
    longest_run = run = 0
    for item in mask:
        run = run + 1 if item else 0
        longest_run = max(run, longest_run)
    return longest_run * dt


def validate_native_result(native):
    if native.get('status') != 'COMPLETED' or native.get('error_message'):
        raise RuntimeError('native diagnostic failed: ' + json.dumps(native, ensure_ascii=False))


def summarize(csv_path, case, controller, native):
    import csv
    with Path(csv_path).open(encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    values = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
    get = lambda key: values['exp_' + key]
    time = values['time_s']
    settled = (time >= 4.) & (time < 5.)
    front = np.maximum(get('X_L1'), get('X_R1')) if case.case_id == 'wide' else get('X_R1')
    rear = np.minimum(get('X_L2'), get('X_R2')) if case.case_id == 'wide' else get('X_R2')
    crossing = (front >= case.geometry.leading_edge_m - .263) & (rear <= case.geometry.trailing_edge_m + .263)
    if not crossing.any():
        crossing = time >= 5.
    dt = float(np.median(np.diff(time)))
    y0, yaw0 = float(np.mean(get('Yo')[settled])), float(np.mean(get('Yaw')[settled]))
    result = {'scope': 'DIAGNOSTIC_BASELINE', 'task_status': 'UNASSESSED',
              'chassis_contact_status': 'NOT_OBSERVABLE', 'native': native,
              'case': case.case_id, 'depth_m': case.geometry.depth_m,
              'length_m': case.geometry.length_m, 'width_m': case.geometry.width_m,
              'edge_transition_m': case.geometry.edge_transition_m,
              'target_speed_kph': controller.speed * 3.6,
              'rear_cleared': bool(min(get('X_L2')[-1], get('X_R2')[-1]) > case.geometry.trailing_edge_m + .263),
              'abort_reason': controller.abort_reason,
              'speed_min_kph': float(get('Vx')[crossing].min()),
              'speed_max_kph': float(get('Vx')[crossing].max()),
              'speed_rmse_kph': float(np.sqrt(np.mean((get('Vx')[crossing] - controller.speed * 3.6)**2))),
              'roll_peak_deg': float(np.max(np.abs(get('Roll_E')[crossing]))),
              'pitch_peak_deg': float(np.max(np.abs(get('Pitch')[crossing]))),
              'lateral_peak_m': float(np.max(np.abs(get('Yo')[crossing] - y0))),
              'yaw_peak_deg': float(np.max(np.abs(get('Yaw')[crossing] - yaw0))),
              'body_vertical_drop_m': float(np.mean(get('Zo')[settled]) - get('Zo')[crossing].min()),
              'az_peak_m_s2': float(np.max(np.abs(get('Az_SM')[crossing])) * 9.80665),
              'az_rms_m_s2': float(np.sqrt(np.mean(get('Az_SM')[crossing]**2)) * 9.80665),
              'torque_peak_nm': float(np.max(np.abs(values['imp_IMP_MYUSM_L1'][crossing]))),
              'torque_saturation_s': float(np.sum(np.abs(values['imp_IMP_MYUSM_L1'][crossing]) >= controller.limit - 1e-6) * dt),
              'settled_jounce_mm': {w: float(np.mean(get('Jnc_' + w)[settled])) for w in WHEELS},
              'wheels': {}}
    for w in WHEELS:
        active = crossing
        loaded = (get('Fz_' + w) >= 500.) & (np.abs(get('Vx')) >= .5)
        loss = (get('Fz_' + w) < 100.) & (get('CmpT_' + w + 'i') <= .1)
        result['wheels'][w] = {
            'load_min_n': float(get('Fz_' + w)[active].min()),
            'load_max_n': float(get('Fz_' + w)[active].max()),
            'contact_loss_s': float(np.sum(loss & active) * dt),
            'longest_contact_loss_s': longest(loss & active, dt),
            'jounce_min_mm': float(get('Jnc_' + w)[active].min()),
            'jounce_max_mm': float(get('Jnc_' + w)[active].max()),
            'rebound_limit_s': float(np.sum((get('Jnc_' + w) <= -149.) & active) * dt),
            'jounce_stop_s': float(np.sum((get('Jnc_' + w) >= 159.) & active) * dt),
            'slip_peak_loaded': float(np.max(np.abs(get('Kappa_' + w + 'i')[active & loaded]))) if np.any(active & loaded) else None,
            'loaded_slip_over_02_s': float(np.sum((np.abs(get('Kappa_' + w + 'i')) > .2) & active & loaded) * dt),
            'ground_depth_seen_m': float(-get('Zgnd_' + w + 'i')[active].min()),
            'native_ground_below_configured_floor_s': float(np.sum((get('Zgnd_' + w + 'i') < -case.geometry.depth_m - .001) & active) * dt),
            'spin_speed_error_peak_m_s': float(np.max(np.abs(get('AVy_' + w)[active] * 2 * math.pi / 60 * .263 - get('Vx')[active] / 3.6))),
            'last_jounce_mm': float(get('Jnc_' + w)[-1]),
            'last_load_n': float(get('Fz_' + w)[-1])}
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case', choices=['right_track', 'wide'], required=True)
    p.add_argument('--depth-m', type=float, required=True)
    p.add_argument('--run-id', required=True)
    p.add_argument('--speed-kph', type=float, default=3.)
    p.add_argument('--length-m', type=float, default=.8)
    p.add_argument('--width-m', type=float)
    p.add_argument('--edge-m', type=float, default=.05)
    p.add_argument('--friction', type=float, default=.7)
    p.add_argument('--duration-s', type=float, default=30.)
    p.add_argument('--torque-limit-nm', type=float, default=250.)
    p.add_argument('--probe-force-n', type=float, default=0.)
    p.add_argument('--probe-corner', choices=['all', *WHEELS], default='all')
    p.add_argument('--probe-ramp-s', type=float, default=10.)
    a = p.parse_args()
    if not (0 <= a.speed_kph <= 5 and 0 < a.torque_limit_nm <= 1000 and abs(a.probe_force_n) <= 18200 and 15 <= a.duration_s <= 120 and 0 < a.probe_ramp_s <= 30):
        p.error('diagnostic settings outside declared software limits')
    case = load_case(ROOT / ('scenarios/shallow_press/' + a.case + '.json'),
                     depth_m=a.depth_m, length_m=a.length_m,
                     width_m=a.width_m if a.width_m is not None else (.9 if a.case == 'right_track' else 1.8),
                     stop_s=a.duration_s, friction=a.friction)
    case = replace(case, geometry=replace(case.geometry, edge_transition_m=a.edge_m))
    validate_case(case)
    run = claim_run(ROOT / 'runs', 'shallow_press', a.case, a.run_id)
    prepare_case(case, run)
    manifest = json.loads((run / 'run_manifest.json').read_text(encoding='utf-8'))
    manifest.update(diagnostic='passive_suspension_equal_torque_speed_PI', diagnostic_options=vars(a))
    manifest['diagnostic_code_sha256'] = sha256_file(Path(__file__))
    write_run_manifest(run, manifest)
    controller = Baseline(manifest['export_names'], a.speed_kph, a.torque_limit_nm,
                          a.probe_force_n, a.probe_corner, a.probe_ramp_s)
    controller.end_station_m = case.geometry.trailing_edge_m + .263 + .8
    native = None
    try:
        native = run_stepwise(run / 'model/simfile.sim', controller, run / 'native_5ms.csv',
                              manifest['import_names'], manifest['export_names'], log_decimation=10,
                              stop_when=lambda t: controller.abort_reason is not None or
                              (controller.completed and t > 15.))
        validate_native_result(native)
    except Exception as error:
        manifest.update(run_status='DIAGNOSTIC_FAILED', task_status='UNASSESSED',
                        acceptance_pass=False, error=str(error), native=native)
        write_run_manifest(run, manifest)
        raise
    result = summarize(run / 'native_5ms.csv', case, controller, native)
    (run / 'baseline_diagnostic.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    manifest.update(run_status='DIAGNOSTIC_COMPLETED', task_status='UNASSESSED', acceptance_pass=False)
    write_run_manifest(run, manifest)
    print(json.dumps({k:v for k,v in result.items() if k not in ('wheels', 'native')}, indent=2))


if __name__ == '__main__':
    main()
