"""Pre-registered, isolated TruckSim FR/RR lift reachability scan.

Run one target corner and an unused output directory at a time. This probe
uses the frozen physical plant and pit; added native CG/contact exports are
observation-only. Failed finite searches are never labelled physical no-go.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from ddevsim.contact_geometry import GEOMETRY_EXPORTS, augment_geometry_exports  # noqa: E402
from ddevsim.cosim import run_stepwise  # noqa: E402
from ddevsim.interface_validation import EXPORT_NAMES, IMPORT_NAMES  # noqa: E402
from ddevsim.pothole_case import PotholeScenario, SCENARIO_EXPORTS  # noqa: E402
from ddevsim.reachability import (  # noqa: E402
    build_profile, classify_reachability, isolated_rear_start_station,
)

SOURCE = ROOT / "models" / "corner_module_ddev" / "single_wheel_deep_pothole"
CONTRACT = ROOT / "runs" / "contact_mpc_interface_stationary_gate1_a" / "primary_experiment_contract.json"


def _hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _prepare_case(source: Path, dest: Path, speed_kph: float,
                  initial_station_m: float) -> Path:
    shutil.copytree(source, dest, ignore=shutil.ignore_patterns("output"))
    (dest / "output").mkdir()
    run_all = dest / "run_all.par"
    simfile = dest / "simfile.sim"
    model_text, sim_text = augment_geometry_exports(
        run_all.read_text(encoding="utf-8", errors="replace"),
        simfile.read_text(encoding="utf-8", errors="replace"),
    )
    original_speed = "SPEED_TARGET_CONSTANT 2.8"
    if model_text.count(original_speed) != 1:
        raise ValueError("frozen model speed parameter not unique")
    model_text = model_text.replace(original_speed, "SPEED_TARGET_CONSTANT %.9g" % speed_kph)
    if initial_station_m != 99.1:
        model_text, count = re.subn(
            r"(?m)^SSTART 99\.1$", "SSTART %.9g" % initial_station_m,
            model_text,
        )
        if count != 2:
            raise ValueError("frozen model must have two matching initial stations")
    run_all.write_text(model_text, encoding="utf-8")
    simfile.write_text(sim_text, encoding="utf-8")
    return simfile


def _read_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def _cases(corner: str) -> list[dict]:
    # Registered before any solver call. The support bias acts at the other
    # *right* corner; the target is the only corner commanded to unload.
    cases = []
    for speed in (2.8, 2.0):
        for preload in (0.5, 0.8):
            for bias in (0.0, 6.0, 12.0, 17.0):
                for target in (-4.0, -8.0, -12.0):
                    forces = [0.0, 0.0, 0.0, 0.0]
                    forces[1 if corner == "RR" else 3] = bias
                    forces[1 if corner == "FR" else 3] = target
                    cases.append({
                        "speed_kph": speed,
                        "preload_s": preload,
                        "lift_s": 1.0,
                        "support_bias_kn": bias,
                        "target_force_kn": target,
                        "force_kn": forces,
                    })
    return cases


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--corner", choices=("FR", "RR"), required=True)
    parser.add_argument("--run-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    run_dir = args.run_dir.resolve()
    if run_dir.exists():
        raise SystemExit("refusing existing run directory: " + str(run_dir))

    frozen = json.loads(CONTRACT.read_text(encoding="utf-8"))
    source_model_hash = _hash(SOURCE / "run_all.par")
    source_scenario_hash = _hash(SOURCE / "scenario.json")
    # The Gate-1 contract records the base generated model, whereas SOURCE is
    # the scenario-expanded copy. Keep both hashes and refuse the wrong pit.
    if source_scenario_hash != frozen["scenario_sha256"]:
        raise SystemExit("scenario drift since Gate 1")
    scenario = PotholeScenario(**json.loads((SOURCE / "scenario.json").read_text(encoding="utf-8")))
    force_limit_n = float(frozen["simulation_authority"]["force_limit_n"])
    slew_n_per_s = float(frozen["simulation_authority"]["force_slew_n_per_s"])
    limits = {
        "pit_start_m": scenario.leading_edge_m,
        "pit_end_m": scenario.trailing_edge_m,
        "pit_center_y_m": scenario.center_y_m,
        "pit_width_m": scenario.width_m,
        "pit_depth_m": scenario.depth_m,
        "edge_transition_m": scenario.edge_transition_m,
        "tyre_radius_m": 0.263,
        "interior_start_m": scenario.leading_edge_m + 0.35,
        "interior_end_m": scenario.trailing_edge_m - 0.35,
        "cg_margin_m": 0.01,
        "cg_error_m": 0.001,
        "gap_error_m": 0.001,
        "support_floor_n": 300.0,
        "target_fz_max_n": 150.0,
        "roll_max_deg": 7.0,
        "yaw_max_deg": 2.0,
        "lateral_max_m": 0.15,
        "jounce_min_m": -0.10,
        "jounce_max_m": 0.16,
    }
    cases = _cases(args.corner)
    initial_station_m = (
        99.1 if args.corner == "FR" else
        math.ceil(100.0 * isolated_rear_start_station(
            scenario.trailing_edge_m, 0.263, 99.10788162 - 99.1, 0.05,
        )) / 100.0
    )
    run_dir.mkdir(parents=True)
    (run_dir / "registered_scan.json").write_text(json.dumps({
        "corner": args.corner,
        "source_model_sha256": source_model_hash,
        "source_scenario_sha256": source_scenario_hash,
        "frozen_primary_model_sha256": frozen["model_sha256"],
        "simulation_authority": frozen["simulation_authority"],
        "geometry_export_names": list(GEOMETRY_EXPORTS),
        "limits": limits,
        "initial_station_m": initial_station_m,
        "isolation": "front tyre fully beyond the pit exit at t=0" if args.corner == "RR" else "front tyre approaches pit",
        "cases": cases,
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    export_names = EXPORT_NAMES + SCENARIO_EXPORTS + GEOMETRY_EXPORTS
    results = []
    for index, case in enumerate(cases):
        candidate_dir = run_dir / ("candidate_%03d" % index)
        simfile = _prepare_case(SOURCE, candidate_dir / "model", case["speed_kph"],
                                initial_station_m)
        # Wheel-centre initial stations come from the frozen native Echo.
        initial_wheel_x = (
            99.10788162 if args.corner == "FR" else
            97.18216411 + (initial_station_m - 99.1)
        )
        nominal_entry_s = (scenario.leading_edge_m - initial_wheel_x) / (case["speed_kph"] / 3.6)
        start_s = max(0.0, nominal_entry_s - case["preload_s"] - 0.3)
        profile = build_profile(
            args.corner, tuple(case["force_kn"]), case["preload_s"],
            case["lift_s"], force_limit_n, slew_n_per_s,
        )
        command = lambda t, exports: profile(t - start_s, exports)
        csv_path = candidate_dir / "native_5ms.csv"
        try:
            native = run_stepwise(simfile, command, csv_path,
                                  IMPORT_NAMES, export_names, log_decimation=10)
            rows = _read_rows(csv_path)
            outcome = classify_reachability(rows, args.corner, limits)
            event = any(
                float(row["exp_Fz_" + ("R1" if args.corner == "FR" else "R2")]) <= 150.0
                and scenario.leading_edge_m <= float(row["exp_X_" + ("R1" if args.corner == "FR" else "R2")]) <= scenario.trailing_edge_m
                for row in rows
            )
            if event:
                # A separate fresh solver history is required for 0.5 ms
                # event evidence; never overwrite the 5 ms native history.
                event_dir = candidate_dir / "event_fullrate_model"
                event_simfile = _prepare_case(SOURCE, event_dir, case["speed_kph"],
                                              initial_station_m)
                full = run_stepwise(event_simfile, command,
                                    candidate_dir / "native_0p5ms.csv",
                                    IMPORT_NAMES, export_names, log_decimation=1)
            else:
                full = None
            result = {"index": index, "case": case, "start_s": start_s,
                      "native": native, "outcome": outcome,
                      "fullrate_native": full}
        except (ValueError, OSError, KeyError) as exc:
            result = {"index": index, "case": case, "start_s": start_s,
                      "error": repr(exc),
                      "outcome": {"status": "feasibility_unresolved",
                                  "binding_constraint": "solver_or_data_error"}}
        results.append(result)
        (run_dir / "results.json").write_text(
            json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        print("%s %d/%d speed %.1f preload %.1f bias %.1f target %.1f: %s / %s" % (
            args.corner, index + 1, len(cases), case["speed_kph"],
            case["preload_s"], case["support_bias_kn"], case["target_force_kn"],
            result["outcome"]["status"], result["outcome"].get("binding_constraint")),
            flush=True)
    accepted = [result for result in results
                if result["outcome"]["status"] == "admissible_candidate"]
    print(json.dumps({"corner": args.corner, "admissible": len(accepted),
                      "attempted": len(results),
                      "status": "admissible_candidate" if accepted else "feasibility_unresolved"}),
          flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
