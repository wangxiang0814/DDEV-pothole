import hashlib
import subprocess
import sys
from pathlib import Path

import pytest

def test_rejects_controller_neutralized_mode_against_normal_springs():
    from ddevsim.plant_mode import validate_plant_mode
    plant = b"spring-loaded TruckSim case"
    manifest = {
        "generated_sha256": hashlib.sha256(plant).hexdigest(),
        "neutralize_springs": {"applied": False},
    }
    with pytest.raises(ValueError, match="plant mode"):
        validate_plant_mode(manifest, plant, ball_screw_mode=True)


def test_rejects_changed_parsfile_after_manifest_generation():
    from ddevsim.plant_mode import validate_plant_mode
    manifest = {
        "generated_sha256": hashlib.sha256(b"original").hexdigest(),
        "neutralize_springs": {"applied": False},
    }
    with pytest.raises(ValueError, match="plant hash"):
        validate_plant_mode(manifest, b"modified", ball_screw_mode=False)


def test_accepts_matching_normal_spring_model():
    from ddevsim.plant_mode import validate_plant_mode
    plant = b"spring-loaded TruckSim case"
    manifest = {
        "generated_sha256": hashlib.sha256(plant).hexdigest(),
        "neutralize_springs": {"applied": False},
    }
    validate_plant_mode(manifest, plant, ball_screw_mode=False)


def test_expert_runner_accepts_distinct_output_directory():
    runner = Path(__file__).resolve().parents[1] / "scripts" / "run_expert_pothole.py"
    completed = subprocess.run(
        [sys.executable, str(runner), "--help"],
        capture_output=True, text=True, check=True,
    )
    assert "--run-dir" in completed.stdout
    assert "--gain-report" in completed.stdout
    assert "--pre-lift-distance-m" in completed.stdout
    assert "--force-slew-time-s" in completed.stdout
    assert "--force-limit-static-multiple" in completed.stdout
    assert "--attitude-deflection-m" in completed.stdout
    assert "--identified-max-roll-target-deg" in completed.stdout


def test_isolated_run_copies_plant_but_not_old_solver_output(tmp_path):
    from ddevsim.plant_mode import prepare_run_model

    source = tmp_path / "source"
    source.mkdir()
    (source / "run_all.par").write_text("PARSFILE\nEND\n", encoding="utf-8")
    (source / "simfile.sim").write_text("SIMFILE\nEND\n", encoding="utf-8")
    (source / "scenario.json").write_text("{}", encoding="utf-8")
    (source / "output").mkdir()
    (source / "output" / "old.erd").write_bytes(b"old")

    target = prepare_run_model(source, tmp_path / "new_run")

    assert (target / "run_all.par").read_text(encoding="utf-8") == "PARSFILE\nEND\n"
    assert (target / "simfile.sim").exists()
    assert (target / "scenario.json").exists()
    assert not (target / "output" / "old.erd").exists()
