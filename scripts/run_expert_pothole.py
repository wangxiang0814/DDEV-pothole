"""Run the paper-faithful deep-pothole expert strategy on the real solver.

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
from ddevsim.pothole_case import SCENARIO_EXPORTS, PotholeScenario  # noqa: E402
from ddevsim.vehicle_params import load_vehicle  # noqa: E402

DEFAULT_LOG_DECIMATION = 10  # 0.5 ms solver step -> 5 ms CSV


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    log_decimation = DEFAULT_LOG_DECIMATION
    if argv and argv[0].isdigit():
        log_decimation = int(argv[0])

    model_dir = ROOT / "models" / "hd_utility_ddev" / "single_wheel_deep_pothole"
    run_dir = ROOT / "runs" / "hd_utility_ddev_expert_pothole"
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
        model_label="HD Utility DDEV 4x4 Active Suspension",
    )

    # 2. controller built from the generated model + the measured baseline
    vehicle = load_vehicle(
        ROOT / "models" / "hd_utility_ddev" / "run_all.par",
        static_wheel_load_n=calibration.wheel_load_n,
    )

    # Use the measured actuator effectiveness matrix when it is available.  Without
    # it the feedforward assumes a 1:1 actuator, which on this solid-axle vehicle
    # under-commands by more than an order of magnitude.
    gain_matrix = None
    gain_report = ROOT / "runs" / "_actuator_gain" / "gain_matrix.json"
    if gain_report.exists():
        payload = json.loads(gain_report.read_text(encoding="utf-8"))
        table = payload["gain_matrix_command_to_load"]
        gain_matrix = [[table[j][i] for i in ("FL", "FR", "RL", "RR")]
                       for j in ("FL", "FR", "RL", "RR")]

    controller = DeepPotholeExpertController(
        scenario=scenario,
        vehicle=vehicle,
        export_names=export_names,
        config=ExpertConfig(),
        gain_matrix=gain_matrix,
        static_deflection_m=calibration.deflection_m,
    )
    if gain_matrix is None:
        print("WARNING: no measured gain matrix; run scripts\\probe_actuator_gain.py first")

    # 3. closed loop through the solver
    result = run_stepwise(
        simfile,
        controller,
        data_dir / "expert_and_dynamics.csv",
        IMPORT_NAMES,
        export_names,
        log_decimation=log_decimation,
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
        "model": "HD Utility DDEV 4x4 Active Suspension",
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
        "controller": "Liu et al. 2024 feedforward + integral sliding mode (three-wheel support)",
        "input_order": list(IMPORT_NAMES),
        "output_order": export_names,
        "calibration": json.loads(calibration.to_json()),
        "vehicle": json.loads(vehicle.to_json()),
        "plan": controller.step_summary(),
        "result": result,
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
    print("samples  : %d control updates" % len(controller.trace))
    print()
    print("target loads for the lift phases:")
    for corner, solution in manifest["plan"]["support_solutions"].items():
        print("  lift %s: %s" % (corner, json.dumps(solution["target_load_n"])))
    print()
    print("manifest -> %s" % (run_dir / "manifest.json"))
    return 0 if result["status"] == "COMPLETED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
