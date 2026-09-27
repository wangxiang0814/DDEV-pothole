"""Validate that a controller mode belongs to the exact generated TruckSim plant."""

from __future__ import annotations

import hashlib
import shutil
from pathlib import Path
from typing import Any, Mapping


def validate_plant_mode(
    manifest: Mapping[str, Any], run_all: bytes, ball_screw_mode: bool
) -> None:
    """Reject a stale parsfile or controller/plant spring-mode mismatch."""
    neutralized = manifest.get("neutralize_springs", {}).get("applied")
    if neutralized is None or bool(neutralized) != ball_screw_mode:
        raise ValueError("plant mode does not match controller mode")
    if hashlib.sha256(run_all).hexdigest() != manifest.get("generated_sha256"):
        raise ValueError("plant hash does not match source manifest")


def prepare_run_model(source: Path, run_root: Path) -> Path:
    """Copy a flattened case into a new run without copying old solver outputs."""
    source, run_root = Path(source).resolve(), Path(run_root).resolve()
    if run_root.exists():
        raise FileExistsError("run directory already exists: %s" % run_root)
    target = run_root / "model"
    target.mkdir(parents=True)
    for name in ("run_all.par", "simfile.sim", "scenario.json"):
        shutil.copy2(source / name, target / name)
    if (source / "visual_assets").is_dir():
        shutil.copytree(source / "visual_assets", target / "visual_assets")
    (target / "output").mkdir()
    return target
