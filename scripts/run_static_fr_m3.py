"""Eight isolated ±500 N I_I TruckSim probes on the braked M1 flat-road case."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import shutil
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

from ddevsim.cosim import run_stepwise
from ddevsim.interface_validation import IMPORT_NAMES, EXPORT_NAMES
from ddevsim.pothole_case import SCENARIO_EXPORTS
from ddevsim.contact_geometry import GEOMETRY_EXPORTS
from ddevsim.static_wheel_lift.quintic_trajectory import quintic_step
from ddevsim.static_wheel_lift.system_identification import central_difference, diagnose_gain
from ddevsim.static_wheel_lift.support_geometry import assess_support
from run_static_fr_m1_m2 import analyze

SOURCE = ROOT / "runs/fr_static_ii_m1_braked_repro/model"
OUTPUT = ROOT / "runs/fr_static_ii_m3_longsettle"
WHEELS = ("L1", "R1", "L2", "R2")
NAMES = ("FL", "FR", "RL", "RR")
EXPORTS = EXPORT_NAMES + SCENARIO_EXPORTS + GEOMETRY_EXPORTS


def response(csv_path: Path, start_s: float, end_s: float) -> dict:
    with csv_path.open(encoding="utf-8", newline="") as stream:
        rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]
    window = [r for r in rows if start_s <= r["time_s"] <= end_s]
    if len(window) < 50:
        raise ValueError("insufficient settled samples")
    fz, att, travel, com, zmp = [], [], [], [], []
    for row in window:
        loads = {n: row[f"exp_Fz_{w}"] for n, w in zip(NAMES, WHEELS)}
        contacts = {n: (row[f"exp_Xctc_{w}i"], row[f"exp_Yctc_{w}i"])
                    for n, w in zip(NAMES, WHEELS)}
        c = (row["exp_XCG_TM"], row["exp_YCG_TM"])
        geometry = assess_support(contacts, loads, c, 0.05)
        fz.append([loads[n] for n in NAMES])
        att.append([np.deg2rad(row["exp_Roll_E"]), np.deg2rad(row["exp_Pitch"])])
        travel.append([row[f"exp_Jnc_{w}"] / 1000 for w in WHEELS])
        com.append(c)
        zmp.append(geometry.zmp_xy)
    return {key: np.mean(value, axis=0).tolist() for key, value in
            (("Fz_n", fz), ("attitude_rad", att), ("travel_m", travel),
             ("CoM_xy_m", com), ("ZMP_xy_m", zmp))}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--amplitude-n", type=float, default=500.0)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--source-model", type=Path, default=SOURCE)
    parser.add_argument("--settled-start-s", type=float, default=10.0)
    parser.add_argument("--settled-end-s", type=float, default=11.0)
    parser.add_argument("--ramp-start-s", type=float, default=7.0)
    parser.add_argument("--ramp-duration-s", type=float, default=1.0)
    args = parser.parse_args()
    source = args.source_model.resolve()
    if not 0 < args.amplitude_n <= 500:
        raise ValueError("M3 probe amplitude limited to 500 N")
    if (args.ramp_start_s < 0 or args.ramp_duration_s <= 0 or
            args.settled_start_s < args.ramp_start_s + args.ramp_duration_s + 0.5 or
            args.settled_end_s <= args.settled_start_s):
        raise ValueError("invalid settled observation window")
    baseline_result = json.loads((source.parent / "result.json").read_text(encoding="utf-8"))
    if baseline_result["M1_status"] != "PASS":
        raise RuntimeError("M1 gate must pass before M3")
    source_hash = hashlib.sha256((source / "run_all.par").read_bytes()).hexdigest()
    if baseline_result["model_sha256"] != source_hash:
        raise RuntimeError("M1 result does not match the selected model")
    if args.settled_end_s > baseline_result["metrics"]["window_end_s"]:
        raise ValueError("settled window exceeds the M1 model duration")
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    baseline = response(source.parent / "native_5ms.csv", args.settled_start_s,
                        args.settled_end_s)
    observations: dict[str, dict] = {}
    for corner, wheel in enumerate(NAMES):
        for sign, suffix in ((1, "pos"), (-1, "neg")):
            case = (args.output / f"{wheel}_{suffix}").resolve()
            shutil.copytree(source, case / "model", ignore=shutil.ignore_patterns("output"))
            (case / "model" / "output").mkdir()
            def command(time_s, _exports, i=corner, multiplier=sign):
                phase = quintic_step(time_s, args.ramp_start_s, args.ramp_duration_s)[0]
                force = [0.0] * 4
                force[i] = multiplier * args.amplitude_n * phase
                return (0.0,) * 4 + tuple(force)
            csv_path = case / "native_5ms.csv"
            native = run_stepwise(case / "model" / "simfile.sim", command,
                                  csv_path, IMPORT_NAMES, EXPORTS, log_decimation=10)
            metrics = analyze(csv_path, settle_start_s=args.settled_start_s,
                              lambda_safe=0.05)
            observations[f"{wheel}_{suffix}"] = {
                "native": native, "gate": metrics["M1_status"],
                "metrics": metrics["metrics"],
                "response": response(csv_path, args.settled_start_s, args.settled_end_s),
            }
            (args.output / "progress.json").write_text(json.dumps(observations, indent=2), encoding="utf-8")
    gains = {}
    for key in baseline:
        columns = [central_difference(observations[f"{wheel}_pos"]["response"][key],
                                      observations[f"{wheel}_neg"]["response"][key],
                                      args.amplitude_n) for wheel in NAMES]
        gains[key] = np.column_stack(columns).tolist()
    gf = np.asarray(gains["Fz_n"])
    gain_diagnostic = diagnose_gain(gf, relative_cutoff=1e-3)
    result = {
        "status": "PASS" if all(v["native"]["status"] == "COMPLETED" and
                                v["gate"] == "PASS" for v in observations.values()) else "FAIL",
        "vehicle": "corner_module_ddev I_I",
        "model_sha256": source_hash,
        "amplitude_n": args.amplitude_n,
        "ramp_s": [args.ramp_start_s, args.ramp_start_s + args.ramp_duration_s],
        "settled_window_s": [args.settled_start_s, args.settled_end_s],
        "baseline_response": baseline, "cases": observations,
        "gains": gains, "G_F_rank": gain_diagnostic["algebraic_rank"],
        "G_F_effective_rank": gain_diagnostic["effective_rank"],
        "G_F_condition": gain_diagnostic["condition_number"],
        "G_F_singular_values": gain_diagnostic["singular_values"],
        "requires_damping": gain_diagnostic["requires_damping"],
    }
    (args.output / "gain_matrix.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps({k: result[k] for k in ("status", "amplitude_n", "G_F_rank",
                                                "G_F_effective_rank", "G_F_condition")}, indent=2))


if __name__ == "__main__":
    main()
