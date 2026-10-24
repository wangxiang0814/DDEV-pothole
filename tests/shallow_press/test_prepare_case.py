from pathlib import Path
import hashlib
import json
import os
import re
import subprocess
import sys

import pytest

from test_scenario_geometry import make_case


@pytest.mark.parametrize('coverage', ['right_track', 'both_tracks'])
def test_preparation_keeps_vehicle_and_synchronizes_physics_and_mesh(tmp_path, coverage):
    from ddevsim.scenarios.builder import prepare_case
    from ddevsim.runtime.run_workspace import claim_run
    from ddevsim.vehicle_profiles.tuned_ii import DEFAULT_PROFILE, build_vehicle, vehicle_identity
    source = DEFAULT_PROFILE.source_dir / 'run_all.par'
    before = source.read_bytes()
    run_dir = claim_run(tmp_path / 'runs', 'shallow_press', coverage, 'trial')
    result = prepare_case(make_case(coverage), run_dir, DEFAULT_PROFILE)
    text = (run_dir / 'model/run_all.par').read_text(encoding='utf-8')
    assert '-0.05' in re.search(r'ROAD_DZ_CARPET.*?ENDTABLE', text, re.S).group()
    obj = (run_dir / 'model/visual_assets/DDEV_pothole_scene.obj').read_text()
    vertices = [float(line.split()[3]) for line in obj.splitlines() if line.startswith('v ')]
    assert -.03 in vertices  # display offset .02 minus physical depth .05
    assert re.findall(r'(?m)^SPEED_TARGET_CONSTANT (.+)$', text) == ['0']
    assert re.findall(r'(?m)^SSTART (.+)$', text) == ['99.1', '99.1']
    assert source.read_bytes() == before
    shared = build_vehicle(tmp_path / 'shared', DEFAULT_PROFILE)
    assert result['vehicle']['vehicle_sha256'] == shared['vehicle_sha256']
    assert vehicle_identity(run_dir / 'model/run_all.par') == shared['vehicle_sha256']
    assert result['task_status'] == 'UNASSESSED'
    with pytest.raises(FileExistsError):
        prepare_case(make_case(coverage), run_dir, DEFAULT_PROFILE)


def test_template_requires_explicit_unprovided_pit_dimensions():
    from ddevsim.scenarios.pit import load_case
    root = Path(__file__).resolve().parents[2]
    template = root / 'scenarios/shallow_press/right_track.json'
    with pytest.raises(ValueError, match='depth_m'):
        load_case(template)
    case = load_case(template, depth_m=.05, length_m=.8, width_m=.9)
    assert case.geometry.depth_m == .05


def test_shallow_run_rejects_before_solver_launch_or_output_claim(tmp_path):
    root = Path(__file__).resolve().parents[2]
    process = subprocess.run([sys.executable, str(root / 'scripts/run_strategy.py'),
                              '--strategy', 'shallow_press', '--case', 'right_track',
                              '--run-id', 'must_not_launch'], cwd=tmp_path,
                             capture_output=True, text=True)
    assert process.returncode != 0
    assert '--depth-m' in process.stderr
    assert not (root / 'runs/shallow_press/right_track/must_not_launch').exists()


def test_preview_launcher_dry_run_is_independent_and_does_not_claim(tmp_path):
    root=Path(__file__).resolve().parents[2]
    process=subprocess.run([sys.executable,str(root/'scripts/run_strategy.py'),
        '--strategy','shallow_press','--case','wide','--depth-m','.04',
        '--run-id','preview_dry_run_no_claim','--dry-run'],cwd=tmp_path,capture_output=True,text=True)
    assert process.returncode==0,process.stderr
    record=json.loads(process.stdout)
    assert record['route']=='PREVIEW_FEEDFORWARD_FEEDBACK'
    assert record['scene']['depth_m']==.04
    assert not (root/'runs/shallow_press/wide/preview_dry_run_no_claim').exists()
