from dataclasses import replace
import json
import re

import pytest


def test_shared_vehicle_matches_legacy_tuned_recipe_and_does_not_change_sources(tmp_path):
    from ddevsim.vehicle_profiles.tuned_ii import DEFAULT_PROFILE, build_vehicle, vehicle_identity
    from ddevsim.strategies.wheel_lift.model_builder import (
        prepare_model, _clamp_kinematics_at_original_bounds, _scale_lateral_ride_movement,
        _enable_steering_import)
    source_before = DEFAULT_PROFILE.source_dir.joinpath('run_all.par').read_bytes()
    legacy, _ = prepare_model(tmp_path / 'legacy', crawl=True)
    _clamp_kinematics_at_original_bounds(legacy)
    _scale_lateral_ride_movement(legacy, 0.)
    _enable_steering_import(legacy)
    shared = tmp_path / 'shared'
    build_vehicle(shared, DEFAULT_PROFILE)
    assert vehicle_identity(legacy / 'run_all.par') == vehicle_identity(shared / 'run_all.par')
    assert DEFAULT_PROFILE.source_dir.joinpath('run_all.par').read_bytes() == source_before
    assert 'PORTS_IMP 9' in (shared / 'simfile.sim').read_text()


def test_profile_identity_changes_when_recipe_changes(tmp_path):
    from ddevsim.vehicle_profiles.tuned_ii import DEFAULT_PROFILE, build_vehicle
    first = build_vehicle(tmp_path / 'a', DEFAULT_PROFILE)
    second = build_vehicle(tmp_path / 'b', replace(DEFAULT_PROFILE, lateral_ride_scale=.5))
    assert first['vehicle_sha256'] != second['vehicle_sha256']


def test_shared_builder_rejects_overwrite_and_invalid_profile(tmp_path):
    from ddevsim.vehicle_profiles.tuned_ii import DEFAULT_PROFILE, build_vehicle
    build_vehicle(tmp_path / 'a', DEFAULT_PROFILE)
    with pytest.raises(FileExistsError):
        build_vehicle(tmp_path / 'a', DEFAULT_PROFILE)
    with pytest.raises(ValueError):
        replace(DEFAULT_PROFILE, lateral_ride_scale=float('nan'))


def test_new_recommended_manifest_keeps_original_case_options():
    from pathlib import Path
    root = Path(__file__).resolve().parents[2]
    new = json.loads((root / 'configs/wheel_lift/recommended.json').read_text())
    old = json.loads((root / 'configs/recommended_right_side_20261005.json').read_text())
    assert new['common'] == old['common']
    assert new['cases'] == old['cases']
