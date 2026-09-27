"""Run the experimental deep-pothole expert strategy on the real solver.

Pipeline
--------
1. measure the model's static operating point (zero-input settle run),
2. build the expert controller with the scenario geometry and vehicle constants
   parsed from the generated TruckSim model,
3. close the loop through the TruckSim Solver API at the full 0.5 ms solver step,
4. archive the CSV, the native history and a manifest.

Usage
-----
    $env:PYTHONPATH='src'; python scripts\\run_expert_pothole.py
"""

from __future__ import annotations

import hashlib
import csv
import json
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from ddevsim.calibration import load_or_measure  # noqa: E402
from ddevsim.cosim import run_stepwise  # noqa: E402
from ddevsim.expert_controller import DeepPotholeExpertController, ExpertConfig  # noqa: E402
from ddevsim.interface_validation import EXPORT_NAMES, IMPORT_NAMES  # noqa: E402
from ddevsim.identified_suspension import roll_gain_from_probe  # noqa: E402
from ddevsim.plant_mode import prepare_run_model, validate_plant_mode  # noqa: E402
from ddevsim.traversal_metrics import evaluate_traversal, run_exit_code  # noqa: E402
from ddevsim.pothole_case import SCENARIO_EXPORTS, PotholeScenario  # noqa: E402
from ddevsim.vehicle_params import load_vehicle  # noqa: E402

DEFAULT_LOG_DECIMATION = 10  # 0.5 ms solver step -> 5 ms CSV


def main(argv=None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model", default="corner_module", choices=("corner_module",),
        help="I_I Compact Utility Truck with four independent suspension corners.",
    )
    parser.add_argument("--log-decimation", type=int, default=DEFAULT_LOG_DECIMATION)
    parser.add_argument(
        "--run-dir", type=Path, default=None,
        help="Unique output directory for this experiment; avoids overwriting previous runs.",
    )
    parser.add_argument(
        "--gain-report", type=Path, default=None,
        help="Measured same-plant actuator gain report; its model hash is checked.",
    )
    parser.add_argument("--pre-lift-distance-m", type=float, default=None)
    parser.add_argument("--force-slew-time-s", type=float, default=None)
    parser.add_argument("--force-limit-static-multiple", type=float, default=None)
    parser.add_argument("--attitude-deflection-m", type=float, default=None)
    parser.add_argument("--identified-max-roll-target-deg", type=float, default=None)
    parser.add_argument(
        "--ball-screw", action="store_true",
        help="experimental spring-neutralised variant only; requires the matching "
             "generated model, and is not a validated electromechanical ball-screw model",
    )
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)
    log_decimation = args.log_decimation

    base_dir = ROOT / "models" / "corner_module_ddev"
    model_dir = base_dir / "single_wheel_deep_pothole"
    run_dir = ROOT / "runs" / "corner_module_expert_pothole"
    base_run_all = base_dir / "run_all.par"
    gain_report_path = ROOT / "runs" / "_actuator_gain_corner_module" / "gain_matrix.json"
    model_label = "Corner Module DDEV (Compact Utility Truck I_I)"
    if not model_dir.exists():
        raise SystemExit(
            "model %s not built; run the matching build script first" % model_dir
        )
    validate_plant_mode(
        json.loads((base_dir / "source_manifest.json").read_text(encoding="utf-8")),
        base_run_all.read_bytes(),
        args.ball_screw,
    )
    if args.gain_report is not None:
        gain_report_path = args.gain_report.resolve()
    if args.run_dir is not None:
        run_dir = args.run_dir.resolve()
        model_dir = prepare_run_model(model_dir, run_dir)

    data_dir = run_dir / "data"
    native_dir = run_dir / "native"
    data_dir.mkdir(parents=True, exist_ok=True)
    native_dir.mkdir(parents=True, exist_ok=True)

    scenario = PotholeScenario(
        **json.loads((model_dir / "scenario.json").read_text(encoding="utf-8"))
    )
    export_names = list(EXPORT_NAMES) + list(SCENARIO_EXPORTS)
    simfile = model_dir / "simfile.sim"

    # 1. measured static operating point
    calibration = load_or_measure(
        model_dir / "calibration.json", simfile, IMPORT_NAMES, export_names,
        model_label=model_label,
    )

    # 2. controller built from the generated model + the measured baseline
    vehicle = load_vehicle(base_run_all, static_wheel_load_n=calibration.wheel_load_n)

    # Use the measured actuator effectiveness matrix when it is available **and still
    # valid**.  A gain matrix is a measurement of one particular plant, so it becomes
    # wrong the moment the model changes: a matrix measured before the front-spring
    # correction was applied produced a yaw excursion of 70 deg, against 34 deg for the
    # same controller with no matrix at all.  The probe records the SHA-256 of the
    # ``run_all.par`` it measured, and this gate refuses anything that does not match
    # the model about to be simulated.
    gain_matrix = None
    travel_gain_matrix = None
    roll_gain_deg_per_n = None
    if gain_report_path.exists():
        payload = json.loads(gain_report_path.read_text(encoding="utf-8"))
        measured_sha = payload.get("model_run_all_sha256")
        current_sha = hashlib.sha256(base_run_all.read_bytes()).hexdigest()
        if measured_sha is None:
            print(
                "WARNING: %s predates the model hash gate and cannot be shown to match "
                "this model; ignoring it. Re-run scripts\\probe_actuator_gain.py."
                % gain_report_path.name
            )
        elif measured_sha != current_sha:
            print(
                "WARNING: %s was measured against a different model "
                "(recorded %s..., current %s...); ignoring it."
                % (gain_report_path.name, measured_sha[:12], current_sha[:12])
            )
        else:
            table = payload["gain_matrix_command_to_load"]
            gain_matrix = [[table[j][i] for i in ("FL", "FR", "RL", "RR")]
                           for j in ("FL", "FR", "RL", "RR")]
            travel_table = payload.get("jounce_matrix_command_to_jnc_mm_per_n")
            if travel_table is not None:
                travel_gain_matrix = [
                    [travel_table[j][i] for i in ("FL", "FR", "RL", "RR")]
                    for j in ("FL", "FR", "RL", "RR")
                ]
            if "roll_response_deg" in payload and "force_amplitude_n" in payload:
                roll_gain_deg_per_n = roll_gain_from_probe(payload)

    config_overrides = {}
    if args.pre_lift_distance_m is not None:
        config_overrides["pre_lift_distance_m"] = args.pre_lift_distance_m
    if args.force_slew_time_s is not None:
        config_overrides["force_slew_time_s"] = args.force_slew_time_s
    if args.force_limit_static_multiple is not None:
        config_overrides["force_limit_static_multiple"] = args.force_limit_static_multiple
    if args.attitude_deflection_m is not None:
        config_overrides["attitude_deflection_m"] = args.attitude_deflection_m
    if args.identified_max_roll_target_deg is not None:
        config_overrides["identified_max_roll_target_deg"] = args.identified_max_roll_target_deg
    controller = DeepPotholeExpertController(
        scenario=scenario,
        vehicle=vehicle,
        export_names=export_names,
        # The published paper supplies the sequential three-wheel support targets but
        # not the numerical T-SRSMC gains.  The primary law is SD (suspension-deflection)
        # tracking, which is bounded by the travel envelope and cannot collapse a support
        # corner the way the measured load-matrix allocator can.  The measured matrices
        # are still loaded and are available to the controller as a secondary path.
        config=ExpertConfig(
            sd_tracking=True,
            ball_screw_mode=args.ball_screw,
            # The roll regulator's sign was measured on the passive-spring plant; with
            # the spring neutralised the roll response changes sign/magnitude, so the
            # regulator is disabled in ball-screw mode until it is re-identified.
            roll_gain_n_per_deg=0.0 if args.ball_screw else 200.0,
            **config_overrides,
        ),
        gain_matrix=gain_matrix,
        travel_gain_matrix=travel_gain_matrix,
        roll_gain_deg_per_n=roll_gain_deg_per_n,
        static_deflection_m=calibration.deflection_m,
    )
    if gain_matrix is None:
        print("WARNING: no measured gain matrix for %s; run scripts\\probe_actuator_gain.py"
              % args.model)

    # 3. closed loop through the solver
    result = run_stepwise(
        simfile,
        controller,
        data_dir / "expert_and_dynamics.csv",
        IMPORT_NAMES,
        export_names,
        log_decimation=log_decimation,
    )
    with (data_dir / "expert_and_dynamics.csv").open(
        "r", encoding="utf-8", newline=""
    ) as stream:
        dynamics_rows = list(csv.DictReader(stream))
    verification = evaluate_traversal(
        dynamics_rows, scenario, safe_stop=controller.safe_stop,
        jounce_stop_m=vehicle.jounce_limit_m,
        landing_load_limit_n=4.0 * max(
            vehicle.static_load(corner) for corner in ("FL", "FR", "RL", "RR")
        ),
    )

    # 4. archive native history
    copied = []
    for source in sorted((model_dir / "output").glob("single_wheel_deep_pothole*")):
        if source.is_file():
            target = native_dir / source.name
            shutil.copy2(str(source), str(target))
            copied.append(str(target))

    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "model": model_label,
        "scenario_name": "right single-wheel deep pothole",
        "scenario": {
            "leading_edge_m": scenario.leading_edge_m,
            "trailing_edge_m": scenario.trailing_edge_m,
            "length_m": scenario.length_m,
            "width_m": scenario.width_m,
            "depth_m": scenario.depth_m,
            "friction": scenario.friction,
            "target_speed_kph": scenario.target_speed_kph,
            "stop_s": scenario.stop_s,
        },
        "controller": (
            "Identified constrained force allocation with phase logic and safety guards"
            if roll_gain_deg_per_n is not None else
            "Phase-based suspension travel tracking with safety guards"
        ),
        "input_order": list(IMPORT_NAMES),
        "output_order": export_names,
        "calibration": json.loads(calibration.to_json()),
        "vehicle": json.loads(vehicle.to_json()),
        "plan": controller.step_summary(),
        "result": result,
        "verification": verification,
        "native_history_files": copied,
    }
    (run_dir / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    print("solver   : %s  final t=%.3f s  rows=%d" % (
        result["status"], result["final_time_s"], result["rows_written"]))
    print("error    : %r" % result.get("error_message", ""))
    print("step     : %s" % manifest["plan"]["final_step_name"])
    print("safe_stop: %s %s" % (manifest["plan"]["safe_stop"], manifest["plan"]["safe_stop_reason"]))
    print("verified : %s %s" % (verification["passed"], verification["failure_reasons"]))
    print("samples  : %d control updates" % len(controller.trace))
    print()
    print("target loads for the lift phases:")
    for corner, solution in manifest["plan"]["support_solutions"].items():
        print("  lift %s: %s" % (corner, json.dumps(solution["target_load_n"])))
    print()
    print("manifest -> %s" % (run_dir / "manifest.json"))
    return run_exit_code(result["status"], verification)


if __name__ == "__main__":
    raise SystemExit(main())
