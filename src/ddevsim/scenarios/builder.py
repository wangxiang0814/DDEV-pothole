"""Prepare a shared vehicle with scenario-local physical road and display assets."""
from __future__ import annotations

from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import re

from ddevsim.runtime.run_workspace import sha256_file, write_run_manifest, code_identity
from ddevsim.vehicle_profiles.tuned_ii import DEFAULT_PROFILE, build_vehicle, vehicle_identity
from ddevsim.visual_mesh import write_visual_mesh
from .pit import validate_case
from .parameters import replace_pit_geometry_parameters


def prepare_case(case, run_dir: Path, profile=DEFAULT_PROFILE) -> dict:
    validate_case(case, profile)
    run_dir = Path(run_dir).resolve()
    claim = json.loads((run_dir / 'run_claim.json').read_text(encoding='utf-8'))
    if claim['strategy_id'] != case.strategy_id or claim['output'] != str(run_dir):
        raise ValueError('scene does not belong to this run claim')
    model = run_dir / 'model'
    vehicle = build_vehicle(model, profile)
    geometry = case.geometry
    path = model / 'run_all.par'
    text = replace_pit_geometry_parameters(path.read_text(encoding='utf-8'), geometry)
    replacements = {
        'SSTART': case.initial_state['vehicle_start_station_m'], 'TSTOP': geometry.stop_s,
        'MU_ROAD_CONSTANT': geometry.friction,
        'SEGMENT_LENGTH': geometry.road_end_m,
        'SET_AZIMUTH': geometry.camera_azimuth_deg, 'SET_ELEVATION': geometry.camera_elevation_deg,
        'SET_DISTANCE': geometry.camera_distance_m, 'SET_FIELD_OF_VIEW': geometry.camera_field_of_view_deg,
        'SET_LOOKPOINT_X': geometry.camera_look_x_m, 'SET_LOOKPOINT_Y': geometry.camera_look_y_m,
        'SET_LOOKPOINT_Z': geometry.camera_look_z_m,
    }
    for name, value in replacements.items():
        text, count = re.subn(r'(?m)^' + name + r'\s+[-+0-9.eE]+\s*$',
                             name + ' ' + format(value, '.9g'), text)
        if count == 0:
            raise ValueError('missing scene field ' + name)
    path.write_text(text, encoding='utf-8')
    write_visual_mesh(model / 'visual_assets', geometry)
    if vehicle_identity(path) != vehicle['vehicle_sha256']:
        raise ValueError('scenario construction changed vehicle identity')
    scenario = {**asdict(geometry), 'strategy_id': case.strategy_id, 'case_id': case.case_id,
                'coverage': case.coverage, 'wheel_sequence': case.wheel_sequence,
                'initial_state': dict(case.initial_state), 'vehicle': vehicle,
                'controller_period_s': profile.control_period_s}
    scenario_path = model / 'scenario.json'
    scenario_path.write_text(json.dumps(scenario, indent=2) + '\n', encoding='utf-8')
    inputs = [line.split()[1] for line in text.splitlines() if line.startswith('IMPORT ')]
    exports = [line.split()[1] for line in text.splitlines() if line.startswith('EXPORT ')]
    hashes = {str(p.relative_to(run_dir)): sha256_file(p)
              for p in (path, model / 'simfile.sim', scenario_path,
                        model / 'visual_assets/DDEV_pothole_scene.obj',
                        model / 'visual_assets/DDEV_pothole_scene.mtl')}
    result = {'strategy_id': case.strategy_id, 'case_id': case.case_id,
              'run_status': 'PREPARED', 'task_status': 'UNASSESSED', 'task_kind': 'SCENE_PREPARATION',
              'acceptance_pass': False, 'vehicle': vehicle,
              'scenario_sha256': sha256_file(scenario_path), 'generated_assets': hashes,
              'import_names': inputs, 'export_names': exports, 'wheel_order': ['FL', 'FR', 'RL', 'RR'],
              'code': code_identity(), 'initial_state': dict(case.initial_state)}
    write_run_manifest(run_dir, {**claim, **result})
    return result
