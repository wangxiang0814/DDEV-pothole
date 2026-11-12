"""Run a selected expert in its own process and uniquely owned directory."""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime
import json
from pathlib import Path
import sys
import subprocess

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from ddevsim.runtime.registry import load_strategy
from ddevsim.runtime.run_workspace import claim_run, run_worker, sha256_file, write_run_manifest
from ddevsim.scenarios.approach import DEFAULT_INITIAL_DISTANCE_M, approach_distance, select_expert
from ddevsim.vehicle_profiles.tuned_ii import DEFAULT_PROFILE, vehicle_identity


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--strategy', required=True, choices=('wheel_lift', 'shallow_press', 'auto'))
    parser.add_argument('--case', required=True)
    parser.add_argument('--run-id', default=None)
    parser.add_argument('--manifest', type=Path)
    parser.add_argument('--reference-model', type=Path)
    parser.add_argument('--approach-distance-m', type=float, default=DEFAULT_INITIAL_DISTANCE_M)
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--demonstration-config', type=Path,
                        help='optional five-camera demonstration configuration')
    args,extra = parser.parse_known_args(argv)
    try:
        distance_m = approach_distance(args.approach_distance_m)
    except ValueError as exc:
        parser.error(str(exc))
    scene_route = None
    if args.strategy == 'auto':
        scene = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
        scene.add_argument('--depth-m', required=True, type=float)
        scene.add_argument('--length-m', type=float, default=.8)
        scene.add_argument('--width-m', type=float)
        dimensions = scene.parse_args(extra)
        width = dimensions.width_m if dimensions.width_m is not None else (1.8 if args.case == 'wide' else .9)
        try:
            route = select_expert(args.case, dimensions.depth_m, dimensions.length_m, width)
        except ValueError as exc:
            parser.error(str(exc))
        if args.manifest or args.reference_model:
            parser.error('automatic scene routing uses the nominal strategy configurations')
        scene_route = dict(coverage=args.case, depth_m=dimensions.depth_m, length_m=dimensions.length_m,
                           width_m=width, selected_strategy=route.strategy, selected_case=route.case)
        args.strategy, args.case = route.strategy, route.case
        extra = [] if route.speed_kph is None else [
            '--depth-m', str(dimensions.depth_m), '--length-m', str(dimensions.length_m),
            '--width-m', str(width), '--speed-kph', str(route.speed_kph), '--duration-s', '100',
            '--delay-s', '.04', '--config', 'configs/shallow_press/preview_multispeed_v2_20261024.json',
        ]
    if args.strategy=='shallow_press':
        if args.manifest or args.reference_model:
            parser.error('shallow_press uses --config and explicit --depth-m, not wheel_lift manifests')
        run_id=args.run_id or datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        command=[sys.executable,str(ROOT/'scripts/run_shallow_preview.py'),
                 '--case',args.case,'--run-id',run_id,
                 '--approach-distance-m',str(distance_m)]+extra
        if args.demonstration_config:
            command+=['--demonstration-config',str(args.demonstration_config.resolve())]
        # Validate geometry/options in a separate process before claiming output.
        checked=subprocess.run(command+['--dry-run'],cwd=ROOT,capture_output=True,text=True)
        if checked.returncode:
            print(checked.stderr,file=sys.stderr);return checked.returncode
        if args.dry_run:
            print(checked.stdout,end='');return 0
        run=claim_run(ROOT/'runs',args.strategy,args.case,run_id)
        command+=['--claimed-output']
        with (run/'run.log').open('x',encoding='utf-8') as log:
            process=subprocess.run(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
        record=json.loads((run/'run_manifest.json').read_text(encoding='utf-8'))
        record.update(argv=command,process_exit_code=process.returncode)
        if scene_route is not None:
            record['scene_route'] = scene_route
        if process.returncode and record.get('run_status')!='ERROR':
            record.update(run_status='ERROR',task_status='ERROR',acceptance_pass=False)
        write_run_manifest(run,record)
        print('Run directory:',run)
        print('Simulation screen:',record.get('task_status'),'Full safety acceptance:',record.get('acceptance_pass',False))
        return process.returncode or (0 if record.get('simulation_pass') else 1)
    if extra:parser.error('unrecognized arguments: '+' '.join(extra))
    if not load_strategy(args.strategy).implemented:
        parser.error('shallow_press control law is not implemented; use prepare_strategy_case.py to prepare scenes')
    # Import only after strategy selection. All options retain the verified launcher semantics.
    from ddevsim.strategies.wheel_lift.verified_cli import build_command
    path = args.manifest or ROOT / 'configs/wheel_lift/recommended.json'
    path = path.resolve()
    manifest = json.loads(path.read_text(encoding='utf-8'))
    if manifest.get('strategy_id', 'wheel_lift') != args.strategy:
        parser.error('manifest belongs to another strategy')
    run_id = args.run_id or datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    proposed = ROOT / 'runs' / args.strategy / args.case / run_id
    try:
        command = build_command(manifest, args.case, proposed, args.reference_model)
    except ValueError as exc:
        parser.error(str(exc))
    command.extend(('--claimed-output', '--approach-distance-m', str(distance_m)))
    if args.demonstration_config:
        from ddevsim.demonstration.cameras import load_config
        capture_config = load_config(args.demonstration_config)
        command += ['--demonstration-config', str(args.demonstration_config.resolve())]
    record = {'strategy_id': args.strategy, 'case_id': args.case, 'run_id': run_id,
              'output': str(proposed), 'argv': command, 'manifest_sha256': sha256_file(path),
              'effective_options': {**manifest['common'], **manifest['cases'][args.case]['options']},
              'vehicle_profile_id': DEFAULT_PROFILE.profile_id,
              'input_capability': 'torque4_force4_steering', 'wheel_order': ['FL', 'FR', 'RL', 'RR'],
              'solver_step_s': .0005, 'control_period_s': .02,
              'source_assets': {}, 'scope': manifest['scope']}
    options = record['effective_options']
    options['approach-distance-m'] = distance_m
    if scene_route is not None:
        record['scene_route'] = scene_route
    if args.demonstration_config:
        record['source_assets'][str(args.demonstration_config.resolve())] = sha256_file(args.demonstration_config)
        record['demonstration_config'] = capture_config
    for name in ('support-allocation-gains', 'contact-gain-bundle'):
        if name in options:
            asset = ROOT / options[name]
            record['source_assets'][str(asset)] = sha256_file(asset)
    for asset in (DEFAULT_PROFILE.source_dir / 'run_all.par',
                  DEFAULT_PROFILE.source_dir / 'simfile.sim',
                  ROOT / 'evidence/static_fr_ii/preload_trace.csv.gz',
                  ROOT / 'evidence/static_fr_ii/swing_support_gain.json',
                  ROOT / 'evidence/static_fr_ii/gain_matrix.json',
                  ROOT / 'evidence/static_fr_ii/lift_reference.csv.gz'):
        record['source_assets'][str(asset)] = sha256_file(asset)
    if args.reference_model:
        record['source_assets'][str(args.reference_model.resolve())] = sha256_file(args.reference_model)
    print(json.dumps(record, ensure_ascii=False, indent=2), flush=True)
    if args.dry_run:
        return 0
    run_dir = claim_run(ROOT / 'runs', args.strategy, args.case, run_id)
    claim = json.loads((run_dir / 'run_manifest.json').read_text(encoding='utf-8'))
    write_run_manifest(run_dir, {**claim, **record})
    (run_dir / 'control_config.json').write_text(json.dumps(options, indent=2) + '\n', encoding='utf-8')
    # Generated config is allowed by the owned-output worker check; it is immutable input provenance.
    code = run_worker(command, run_dir=run_dir, cwd=ROOT)
    if (run_dir / 'model/run_all.par').is_file():
        record = json.loads((run_dir / 'run_manifest.json').read_text(encoding='utf-8'))
        record['vehicle_sha256'] = vehicle_identity(run_dir / 'model/run_all.par')
        write_run_manifest(run_dir, record)
    print('Run directory:', run_dir)
    print('Worker exit:', code, '(details in run.log)')
    if args.demonstration_config and capture_config['enabled'] and (run_dir / 'result.json').exists():
        from ddevsim.demonstration.capture import capture_episode
        record = json.loads((run_dir / 'run_manifest.json').read_text(encoding='utf-8'))
        try:
            collected = capture_episode(run_dir, capture_config)
            record['demonstration'] = {
                'status': collected['collection_status'],
                'trainable': collected['trainable'],
                'episode': 'demonstration/episode/episode.json',
            }
            if collected['collection_status'] != 'PASS':
                code = code or 2
        except Exception as exc:
            record['demonstration'] = {'status': 'ERROR', 'trainable': False, 'error': repr(exc)}
            code = code or 2
            print('Demonstration collection failed:', exc)
        write_run_manifest(run_dir, record)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
