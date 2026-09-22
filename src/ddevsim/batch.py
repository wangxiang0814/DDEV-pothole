"""Batch simulation and dataset export for the deep-pothole study.

Design constraints that shape this module
-----------------------------------------
1. **The TruckSim solver DLL is a process-wide singleton.**  ``vs_read_configuration``
   loads one case at a time, so cases cannot share a process.  Parallelism is
   therefore *process*-level, one solver per worker.
2. **Every case needs its own model directory.**  ``simfile.sim`` names the native
   history, echo, log and ERD files; two cases sharing a directory would overwrite
   each other's history (and the video export needs a per-case history).  Each case
   therefore gets its own ``run_all.par`` and ``simfile.sim`` with unique FILEBASE
   names.
3. **Not every run is usable data.**  A case that drove the tyre model out of its
   table, exceeded suspension travel, or tripped SAFE_STOP is still emitted but
   flagged, so a training set can filter on the flags instead of silently learning
   from extrapolated dynamics.

Dataset layout (per batch)::

    runs/<batch>/
      cases/<case>/model/{run_all.par,simfile.sim}
      cases/<case>/output/...            native TruckSim history (animatable)
      cases/<case>/dataset.npz           obs/act/time arrays
      cases/<case>/qa.json               quality gates
      dataset_index.csv                  one row per case, all QA flags
      batch_manifest.json                parameters, code paths, timings
"""

from __future__ import annotations

import csv
import json
import re
import sys
import time
import traceback
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from .calibration import StaticCalibration
from .cosim import run_stepwise
from .expert_controller import CORNERS, DeepPotholeExpertController, ExpertConfig
from .interface_validation import EXPORT_NAMES, IMPORT_NAMES
from .pothole_case import (
    SCENARIO_EXPORTS,
    PotholeScenario,
    build_single_wheel_pothole_case,
)
from .units import CHANNEL_UNITS, UNVERIFIED, require_verified, to_si
from .vehicle_params import VehicleControllerParams

#: Channels excluded from the ML observation vector because their unit could not
#: be established empirically (see ddevsim.units).
DEFAULT_EXCLUDED_CHANNELS = tuple(
    name for name, unit in CHANNEL_UNITS.items() if unit == UNVERIFIED
)

_SUFFIX = {"FL": "L1", "FR": "R1", "RL": "L2", "RR": "R2"}


@dataclass
class BatchCase:
    """One simulation case: a scenario plus optional controller overrides."""

    name: str
    scenario: PotholeScenario
    config_overrides: Dict[str, Any] = field(default_factory=dict)

    def config(self) -> ExpertConfig:
        config = ExpertConfig()
        for key, value in self.config_overrides.items():
            if not hasattr(config, key):
                raise KeyError("ExpertConfig has no field %r" % (key,))
            setattr(config, key, value)
        return config


def default_cases() -> List[BatchCase]:
    """The single-wheel deep-pothole family the platform currently supports.

    Only one scenario class is in scope (right-track single-wheel pothole), so this
    sweeps the pothole depth and the crawl speed around the retained baseline.
    """
    base = PotholeScenario()
    cases = [BatchCase(name="baseline", scenario=base)]
    for depth in (0.10, 0.20, 0.30):
        cases.append(
            BatchCase(
                name="depth_%03d" % int(depth * 1000),
                scenario=PotholeScenario(**{**asdict(base), "depth_m": depth}),
            )
        )
    for speed in (1.4, 5.6):
        cases.append(
            BatchCase(
                name="speed_%03d" % int(speed * 10),
                scenario=PotholeScenario(**{**asdict(base), "target_speed_kph": speed}),
            )
        )
    return cases


# --------------------------------------------------------------------- model prep
def _safe_name(name: str) -> str:
    """Restrict a case name to characters that are safe inside a TruckSim simfile."""
    cleaned = re.sub(r"[^A-Za-z0-9_]", "_", name)
    if not cleaned or not cleaned[0].isalpha():
        cleaned = "case_" + cleaned
    return cleaned


def prepare_case_model(
    base_run_all: Path,
    base_simfile: Path,
    case_dir: Path,
    name: str,
    scenario: PotholeScenario,
) -> Path:
    """Build this case's own model directory and return its simfile.

    The model is rebuilt from the base DDEV model and the case's scenario, so the
    pothole geometry and the speed target follow the scenario.  The history
    basename is made unique per case so parallel cases cannot collide.
    """
    model_dir = Path(case_dir) / "model"
    model_dir.mkdir(parents=True, exist_ok=True)
    history_name = _safe_name(name)
    artifacts = build_single_wheel_pothole_case(
        Path(base_run_all), Path(base_simfile), model_dir,
        scenario=scenario, history_name=history_name,
    )
    return artifacts["simfile"]


# ------------------------------------------------------------------------ quality
def parse_solver_log(log_path: Path) -> Dict[str, Any]:
    """Extract the solver's own warnings, especially out-of-table extrapolation."""
    log_path = Path(log_path)
    if not log_path.exists():
        return {"log_found": False, "extrapolation_count": 0, "extrapolation_functions": [], "stopped_reason": ""}
    text = log_path.read_text(encoding="utf-8", errors="replace")
    functions = sorted(set(re.findall(r"Function\s+(\S+)\s+extrapolated", text)))
    stopped = ""
    match = re.search(r"(?m)^(Run stopped at .*|.*time step too small.*)$", text, re.I)
    if match:
        stopped = match.group(1).strip()
    return {
        "log_found": True,
        "extrapolation_count": len(re.findall(r"extrapolated \(out of range", text)),
        "extrapolation_functions": functions,
        "stopped_reason": stopped,
    }


def quality_gates(
    rows: Sequence[Dict[str, float]],
    vehicle: VehicleControllerParams,
    solver: Dict[str, Any],
    log: Dict[str, Any],
    controller: DeepPotholeExpertController,
    tyre_reference_load_n: float,
) -> Dict[str, Any]:
    """Decide whether a finished run is usable as data, and say exactly why not."""
    def column(name: str) -> List[float]:
        return [float(row[name]) for row in rows]

    loads = {c: column("exp_Fz_%s" % _SUFFIX[c]) for c in CORNERS}
    roll = column("exp_Roll_E")
    deflections = {c: column("exp_CmpS_%s" % _SUFFIX[c]) for c in CORNERS}

    peak_load = max(max(values) for values in loads.values())
    samples = len(rows)
    airborne = {
        c: sum(1 for value in values if value < 500.0) / samples for c, values in loads.items()
    }
    all_airborne = sum(
        1 for i in range(samples) if all(loads[c][i] < 500.0 for c in CORNERS)
    ) / samples
    travel = vehicle.jounce_limit_m * 1000.0
    peak_deflection = max(max(values) for values in deflections.values())
    min_deflection = min(min(values) for values in deflections.values())

    failures: List[str] = []
    if solver.get("status") != "COMPLETED":
        failures.append("run_incomplete")
    if log.get("extrapolation_count", 0) > 0:
        failures.append("tyre_or_suspension_table_extrapolated")
    if peak_load > 3.0 * tyre_reference_load_n:
        failures.append("wheel_load_above_3x_tyre_reference")
    if peak_deflection > travel * 1.02 or abs(min_deflection) > travel * 1.02:
        failures.append("suspension_travel_exceeded")
    if controller.safe_stop:
        failures.append("controller_safe_stop:%s" % controller.safe_stop_reason)
    if all_airborne > 0.001:
        failures.append("whole_vehicle_airborne")

    return {
        "usable": not failures,
        "failures": failures,
        "samples": samples,
        "peak_wheel_load_n": peak_load,
        "tyre_reference_load_n": tyre_reference_load_n,
        "peak_load_over_tyre_reference": peak_load / tyre_reference_load_n,
        "airborne_fraction": airborne,
        "all_wheels_airborne_fraction": all_airborne,
        "peak_roll_deg": max(abs(value) for value in roll),
        "peak_deflection_mm": peak_deflection,
        "min_deflection_mm": min_deflection,
        "travel_limit_mm": travel,
        "extrapolation_functions": log.get("extrapolation_functions", []),
        "solver_stopped_reason": log.get("stopped_reason", ""),
        "final_step_name": controller.step_summary()["final_step_name"],
        "safe_stop": controller.safe_stop,
        "safe_stop_reason": controller.safe_stop_reason,
    }


# ---------------------------------------------------------------------- dataset IO
def observations_in_si(
    rows: Sequence[Dict[str, float]],
    include_unverified: bool = False,
) -> Tuple[List[str], List[List[float]]]:
    """Convert exported channels into SI observation columns.

    Unverified-unit channels are excluded by default so an unmeasured unit cannot
    leak into a training set.
    """
    names = [name for name in list(EXPORT_NAMES) + list(SCENARIO_EXPORTS)]
    if not include_unverified:
        names = [name for name in names if name not in DEFAULT_EXCLUDED_CHANNELS]
    for name in names:
        require_verified(name)
    matrix = [
        [to_si(name, float(row["exp_" + name])) for name in names] for row in rows
    ]
    return names, matrix


def write_case_dataset(
    case_dir: Path,
    rows: Sequence[Dict[str, float]],
    include_unverified: bool = False,
) -> Dict[str, Any]:
    """Write ``dataset.npz`` (obs/act/time) plus a column-name sidecar."""
    import numpy as np

    obs_names, obs = observations_in_si(rows, include_unverified=include_unverified)
    act_names = list(IMPORT_NAMES)
    act = [[float(row["imp_" + name]) for name in act_names] for row in rows]
    time_s = [float(row["time_s"]) for row in rows]

    path = Path(case_dir) / "dataset.npz"
    np.savez_compressed(
        path,
        obs=np.asarray(obs, dtype=np.float32),
        act=np.asarray(act, dtype=np.float32),
        time_s=np.asarray(time_s, dtype=np.float64),
        obs_names=np.asarray(obs_names),
        act_names=np.asarray(act_names),
    )
    return {
        "dataset": str(path),
        "obs_shape": [len(obs), len(obs_names)],
        "act_shape": [len(act), len(act_names)],
        "obs_names": obs_names,
        "act_names": act_names,
        "excluded_unverified_channels": list(DEFAULT_EXCLUDED_CHANNELS),
    }


# --------------------------------------------------------------------- single case
def run_case(
    case: BatchCase,
    base_run_all: Path,
    base_simfile: Path,
    run_root: Path,
    vehicle: VehicleControllerParams,
    calibration: StaticCalibration,
    gain_matrix: Optional[Sequence[Sequence[float]]],
    tyre_reference_load_n: float,
    log_decimation: int,
    include_unverified: bool = False,
) -> Dict[str, Any]:
    """Run one case to completion and write its model, dataset and QA report."""
    case_dir = Path(run_root) / "cases" / case.name
    case_dir.mkdir(parents=True, exist_ok=True)
    simfile = prepare_case_model(
        base_run_all, base_simfile, case_dir, case.name, case.scenario
    )
    history_name = _safe_name(case.name)

    export_names = list(EXPORT_NAMES) + list(SCENARIO_EXPORTS)
    controller = DeepPotholeExpertController(
        scenario=case.scenario,
        vehicle=vehicle,
        export_names=export_names,
        config=case.config(),
        gain_matrix=gain_matrix,
        static_deflection_m=calibration.deflection_m,
    )

    started = time.time()
    result = run_stepwise(
        simfile,
        controller,
        case_dir / "closed_loop.csv",
        IMPORT_NAMES,
        export_names,
        log_decimation=log_decimation,
    )
    elapsed = time.time() - started

    with (case_dir / "closed_loop.csv").open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    numeric = [{k: float(v) for k, v in row.items()} for row in rows]

    output_dir = simfile.parent / "output"
    log = parse_solver_log(output_dir / ("%s_log.txt" % history_name))
    gates = quality_gates(
        numeric, vehicle, result, log, controller, tyre_reference_load_n
    )
    dataset = write_case_dataset(case_dir, numeric, include_unverified=include_unverified)

    qa = {
        "case": case.name,
        "scenario": asdict(case.scenario),
        "config_overrides": case.config_overrides,
        "solver": result,
        "wall_time_s": elapsed,
        "solver_log": log,
        "gates": gates,
        "dataset": dataset,
        "plan": controller.step_summary(),
    }
    (case_dir / "qa.json").write_text(
        json.dumps(qa, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return {
        "case": case.name,
        "usable": gates["usable"],
        "failures": gates["failures"],
        "solver_status": result.get("status"),
        "wall_time_s": elapsed,
        "peak_load_over_tyre_reference": gates["peak_load_over_tyre_reference"],
        "peak_roll_deg": gates["peak_roll_deg"],
        "all_wheels_airborne_fraction": gates["all_wheels_airborne_fraction"],
        "case_dir": str(case_dir),
        "native_history": str(output_dir),
    }


# -------------------------------------------------------------------------- driver
def _worker(payload: Dict[str, Any]) -> Dict[str, Any]:
    """ProcessPoolExecutor entry point.  One solver per process."""
    try:
        return run_case(**payload)
    except Exception as error:  # a single bad case must not kill the batch
        return {
            "case": payload["case"].name,
            "usable": False,
            "failures": ["case_exception:%s" % type(error).__name__],
            "error": str(error),
            "traceback": traceback.format_exc(),
        }


def run_batch(
    cases: Sequence[BatchCase],
    base_run_all: Path,
    base_simfile: Path,
    run_root: Path,
    vehicle: VehicleControllerParams,
    calibration: StaticCalibration,
    gain_matrix: Optional[Sequence[Sequence[float]]],
    tyre_reference_load_n: float,
    workers: int = 1,
    log_decimation: int = 10,
    include_unverified: bool = False,
    batch_name: Optional[str] = None,
) -> Dict[str, Any]:
    """Run every case, then write the batch index and manifest."""
    run_root = Path(run_root)
    run_root.mkdir(parents=True, exist_ok=True)
    common: Dict[str, Any] = {
        "base_run_all": base_run_all,
        "base_simfile": base_simfile,
        "run_root": run_root,
        "vehicle": vehicle,
        "calibration": calibration,
        "gain_matrix": gain_matrix,
        "tyre_reference_load_n": tyre_reference_load_n,
        "log_decimation": log_decimation,
        "include_unverified": include_unverified,
    }
    payloads = [dict(common, case=case) for case in cases]

    started = time.time()
    results: List[Dict[str, Any]] = []
    parallelism = "serial"
    if workers > 1:
        try:
            with ProcessPoolExecutor(max_workers=workers) as pool:
                futures = {
                    pool.submit(_worker, payload): payload["case"].name
                    for payload in payloads
                }
                for future in as_completed(futures):
                    results.append(future.result())
            parallelism = "process_pool"
        except (OSError, PermissionError) as error:
            # Some sandboxed/restricted environments forbid the named pipes that
            # multiprocessing needs.  Fall back to serial rather than failing the
            # whole batch, and record why.
            parallel_error = "%s: %s" % (type(error).__name__, error)
            results = []
            parallelism = "serial_fallback:%s" % parallel_error
    if not results:
        for payload in payloads:
            results.append(_worker(payload))
    elapsed = time.time() - started

    results.sort(key=lambda item: item["case"])
    index_path = run_root / "dataset_index.csv"
    with index_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            ["case", "usable", "failures", "solver_status", "wall_time_s",
             "peak_load_over_tyre_reference", "peak_roll_deg",
             "all_wheels_airborne_fraction", "case_dir", "native_history"]
        )
        for item in results:
            writer.writerow([
                item["case"],
                item.get("usable"),
                ";".join(item.get("failures") or []),
                item.get("solver_status", ""),
                "%.3f" % item.get("wall_time_s", 0.0),
                "%.4f" % item.get("peak_load_over_tyre_reference", 0.0),
                "%.4f" % item.get("peak_roll_deg", 0.0),
                "%.5f" % item.get("all_wheels_airborne_fraction", 0.0),
                item.get("case_dir", ""),
                item.get("native_history", ""),
            ])

    usable = [item for item in results if item.get("usable")]
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "batch_name": batch_name or run_root.name,
        "python": sys.version,
        "workers": workers,
        "parallelism": parallelism,
        "case_count": len(cases),
        "usable_count": len(usable),
        "wall_time_s": elapsed,
        "simulated_seconds": sum(case.scenario.stop_s for case in cases),
        "tyre_reference_load_n": tyre_reference_load_n,
        "log_decimation": log_decimation,
        "excluded_unverified_channels": list(DEFAULT_EXCLUDED_CHANNELS),
        "gain_matrix_used": gain_matrix is not None,
        "cases": [asdict(case) for case in cases],
        "results": results,
        "dataset_index": str(index_path),
    }
    (run_root / "batch_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return manifest
