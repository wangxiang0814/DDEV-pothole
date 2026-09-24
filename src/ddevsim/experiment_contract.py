"""Immutable identity and observability contract for a TruckSim experiment."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Mapping, Sequence

from .interface_validation import IMPORT_NAMES
from .units import unit_of


_CORNERS = ("L1", "R1", "L2", "R2")
_REQUIRED_EXPORTS = {
    "%s_%s" % (prefix, corner)
    for prefix in ("Fz", "X", "Y", "Z", "Jnc", "FsExt")
    for corner in _CORNERS
} | {"Zgnd_%si" % corner for corner in _CORNERS} | {
    "Roll_E", "Pitch", "Yaw", "AVx", "AVy", "AVz", "Yo"
}


def freeze_contract(
    run_all: Path,
    scenario: Path,
    manifest: Mapping[str, object],
    import_names: Sequence[str],
    export_names: Sequence[str],
) -> dict:
    """Verify the generated plant and every observation needed by lift control.

    The returned dictionary is deterministic so it can be embedded in a run
    manifest and compared with an identification dataset before reuse.
    """
    model_sha = hashlib.sha256(Path(run_all).read_bytes()).hexdigest()
    if model_sha != manifest.get("generated_sha256"):
        raise ValueError("plant hash does not match source manifest")

    actual_imports = tuple(import_names)
    if actual_imports != tuple(IMPORT_NAMES):
        raise ValueError("eight import channels must match IMPORT_NAMES order")
    actual_exports = tuple(export_names)
    if len(actual_exports) != len(set(actual_exports)):
        raise ValueError("duplicate export channel name")
    missing = sorted(_REQUIRED_EXPORTS - set(actual_exports))
    if missing:
        raise ValueError("missing export: " + ", ".join(missing))

    contract = {
        "model_sha256": model_sha,
        "scenario_sha256": hashlib.sha256(Path(scenario).read_bytes()).hexdigest(),
        "import_names": list(actual_imports),
        "export_names": list(actual_exports),
        "input_units": ["N-m"] * 4 + ["N"] * 4,
        "output_units": {name: unit_of(name) for name in actual_exports},
        "terrain_preview_source": "ground_truth",
    }
    if "simulation_authority" in manifest:
        authority = manifest["simulation_authority"]
        if not isinstance(authority, Mapping):
            raise ValueError("simulation_authority must be a mapping")
        validated = {}
        for key in ("force_limit_n", "force_slew_n_per_s"):
            try:
                value = float(authority[key])
            except (KeyError, TypeError, ValueError):
                raise ValueError("simulation_authority requires " + key)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("simulation_authority requires positive " + key)
            validated[key] = value
        contract["simulation_authority"] = validated
    return contract


def require_contract(data: Mapping[str, object], expected: Mapping[str, object]) -> None:
    """Reject a dataset from a different plant, road, port order or authority."""
    for key in (
        "model_sha256", "scenario_sha256", "import_names", "export_names",
        "input_units", "output_units", "terrain_preview_source",
        "simulation_authority",
    ):
        if key in expected and data.get(key) != expected[key]:
            raise ValueError("experiment contract mismatch: " + key)


def write_frozen_contract(path: Path, contract: Mapping[str, object]) -> Path:
    """Persist one experiment identity without overwriting an earlier audit."""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("x", encoding="utf-8") as stream:
        json.dump(dict(contract), stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return target
