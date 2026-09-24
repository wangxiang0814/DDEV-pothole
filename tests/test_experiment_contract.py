"""Frozen-case checks before identification or native closed-loop runs."""

import hashlib

import pytest

from ddevsim.experiment_contract import freeze_contract, require_contract, write_frozen_contract
from ddevsim.interface_validation import EXPORT_NAMES, IMPORT_NAMES
from ddevsim.pothole_case import SCENARIO_EXPORTS


EXPORTS = tuple(EXPORT_NAMES) + tuple(SCENARIO_EXPORTS)


def _case(tmp_path):
    par = tmp_path / "run_all.par"
    scene = tmp_path / "scenario.json"
    par.write_bytes(b"frozen vehicle")
    scene.write_text('{"depth_m": 0.2}', encoding="utf-8")
    manifest = {"generated_sha256": hashlib.sha256(par.read_bytes()).hexdigest()}
    return par, scene, manifest


def test_changed_plant_is_rejected_before_solver(tmp_path):
    par, scene, manifest = _case(tmp_path)
    par.write_bytes(b"modified vehicle")
    with pytest.raises(ValueError, match="plant hash"):
        freeze_contract(par, scene, manifest, IMPORT_NAMES, EXPORTS)


def test_missing_applied_rear_force_is_rejected(tmp_path):
    par, scene, manifest = _case(tmp_path)
    without_applied_rr = tuple(name for name in EXPORTS if name != "FsExt_R2")
    with pytest.raises(ValueError, match="FsExt_R2"):
        freeze_contract(par, scene, manifest, IMPORT_NAMES, without_applied_rr)


def test_missing_tyre_road_height_is_rejected(tmp_path):
    par, scene, manifest = _case(tmp_path)
    without_rear_road = tuple(name for name in EXPORTS if name != "Zgnd_R2i")
    with pytest.raises(ValueError, match="Zgnd_R2i"):
        freeze_contract(par, scene, manifest, IMPORT_NAMES, without_rear_road)


def test_contract_fixes_scenario_hash_and_ordered_channels(tmp_path):
    par, scene, manifest = _case(tmp_path)
    contract = freeze_contract(par, scene, manifest, IMPORT_NAMES, EXPORTS)
    assert contract["scenario_sha256"] == hashlib.sha256(scene.read_bytes()).hexdigest()
    assert contract["import_names"] == list(IMPORT_NAMES)
    assert contract["export_names"] == list(EXPORTS)
    assert contract["input_units"] == ["N-m"] * 4 + ["N"] * 4


def test_stale_scenario_contract_is_rejected(tmp_path):
    par, scene, manifest = _case(tmp_path)
    expected = freeze_contract(par, scene, manifest, IMPORT_NAMES, EXPORTS)
    stale = dict(expected, scenario_sha256="0" * 64)
    with pytest.raises(ValueError, match="scenario_sha256"):
        require_contract(stale, expected)


def test_changed_force_authority_is_rejected(tmp_path):
    par, scene, manifest = _case(tmp_path)
    expected = freeze_contract(par, scene, manifest, IMPORT_NAMES, EXPORTS)
    expected["simulation_authority"] = {"force_limit_n": 10000.0,
                                        "force_slew_n_per_s": 20000.0}
    changed = dict(expected, simulation_authority={"force_limit_n": 12000.0,
                                                    "force_slew_n_per_s": 20000.0})
    with pytest.raises(ValueError, match="simulation_authority"):
        require_contract(changed, expected)


def test_contract_records_declared_simulation_force_authority(tmp_path):
    par, scene, manifest = _case(tmp_path)
    manifest["simulation_authority"] = {
        "force_limit_n": 18000.0,
        "force_slew_n_per_s": 45000.0,
    }
    contract = freeze_contract(par, scene, manifest, IMPORT_NAMES, EXPORTS)
    assert contract["simulation_authority"] == manifest["simulation_authority"]


def test_nonpositive_force_authority_is_rejected(tmp_path):
    par, scene, manifest = _case(tmp_path)
    manifest["simulation_authority"] = {
        "force_limit_n": -1.0,
        "force_slew_n_per_s": 45000.0,
    }
    with pytest.raises(ValueError, match="force_limit_n"):
        freeze_contract(par, scene, manifest, IMPORT_NAMES, EXPORTS)


def test_frozen_metadata_is_written_once(tmp_path):
    par, scene, manifest = _case(tmp_path)
    contract = freeze_contract(par, scene, manifest, IMPORT_NAMES, EXPORTS)
    target = tmp_path / "audit" / "experiment_contract.json"
    write_frozen_contract(target, contract)
    assert '"terrain_preview_source": "ground_truth"' in target.read_text(encoding="utf-8")
    with pytest.raises(FileExistsError):
        write_frozen_contract(target, contract)
