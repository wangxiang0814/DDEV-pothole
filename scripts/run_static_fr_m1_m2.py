"""Native, zero-command I_I stationary baseline and read-only support monitor."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import re
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from ddevsim.cosim import run_stepwise
from ddevsim.interface_validation import IMPORT_NAMES, EXPORT_NAMES
from ddevsim.pothole_case import SCENARIO_EXPORTS
from ddevsim.contact_geometry import GEOMETRY_EXPORTS
from ddevsim.static_wheel_lift.support_geometry import assess_support

SOURCE = ROOT / "evidence/static_fr_ii/m1_source"
DEFAULT_OUTPUT = ROOT / "runs/fr_static_ii_m1_braked_repro"
ORDER = ("L1", "R1", "L2", "R2")
NAMES = ("FL", "FR", "RL", "RR")


def prepare_model(target: Path, duration_s: float, brake_mpa: float) -> Path:
    if not (SOURCE / "simfile.sim").is_file():
        raise FileNotFoundError(SOURCE / "simfile.sim")
    if target.exists():
        raise FileExistsError(target)
    shutil.copytree(SOURCE, target, ignore=shutil.ignore_patterns("output"))
    (target / "output").mkdir()
    path = target / "run_all.par"
    source = path.read_text(encoding="utf-8", errors="replace")
    if not re.search(r"(?m)^VEHICLE_CODE i_i$", source):
        raise ValueError("source is not I_I")
    source, n = re.subn(r"(?m)^OPT_SC 3$", "OPT_SC 0", source)
    if n != 1:
        raise ValueError("unexpected speed controller configuration")
    if not 0.0 <= brake_mpa <= 1.0:
        raise ValueError("brake pressure outside registered 0–1 MPa probe range")
    if brake_mpa > 0:
        source = source.replace("OPT_SC 0\n", f"OPT_SC 0\nPBK_CON_CONSTANT {brake_mpa:g}\n", 1)
    source, n = re.subn(r"(?m)^TSTOP 1$", f"TSTOP {duration_s:g}", source)
    if n != 2:
        raise ValueError("unexpected TSTOP configuration")
    # Only the two known right-track pit rows are changed; tyre/suspension stay fixed.
    for station in ("101.15", "102.25"):
        old = f"{station}, 0, 0, -0.45, -0.45, 0, 0"
        if source.count(old) != 1:
            raise ValueError(f"unexpected road row {station}")
        source = source.replace(old, f"{station}, 0, 0, 0, 0, 0, 0")
    path.write_text(source, encoding="utf-8")
    return target / "simfile.sim"


def analyze(csv_path: Path, *, settle_start_s: float, lambda_safe: float) -> dict:
    with csv_path.open(encoding="utf-8", newline="") as stream:
        rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]
    tail = [r for r in rows if r["time_s"] >= settle_start_s]
    if len(tail) < 100:
        raise ValueError("settled observation window too short")
    support = []
    for row in rows:
        contacts = {name: (row[f"exp_Xctc_{wheel}i"], row[f"exp_Yctc_{wheel}i"])
                    for name, wheel in zip(NAMES, ORDER)}
        loads = {name: row[f"exp_Fz_{wheel}"] for name, wheel in zip(NAMES, ORDER)}
        assessment = assess_support(contacts, loads,
                                    (row["exp_XCG_TM"], row["exp_YCG_TM"]),
                                    lambda_safe)
        support.append((row["time_s"], assessment))
    tail_support = [s for t, s in support if t >= settle_start_s]
    metrics = {
        "window_start_s": settle_start_s,
        "window_end_s": rows[-1]["time_s"],
        "max_abs_vx_kph": max(abs(r["exp_Vx"]) for r in tail),
        "max_abs_wheel_rpm": max(abs(r[f"exp_AVy_{wheel}"]) for r in tail for wheel in ORDER),
        "max_abs_roll_rate_deg_s": max(abs(r["exp_AVx"]) for r in tail),
        "max_abs_pitch_rate_deg_s": max(abs(r["exp_AVy"]) for r in tail),
        "min_fz_n": {name: min(r[f"exp_Fz_{wheel}"] for r in tail)
                     for name, wheel in zip(NAMES, ORDER)},
        "max_abs_roll_deg": max(abs(r["exp_Roll_E"]) for r in tail),
        "max_abs_pitch_deg": max(abs(r["exp_Pitch"]) for r in tail),
        "min_lambda": min(s.lambda_min for s in tail_support),
        "min_edge_margin_m": min(s.edge_distance_m for s in tail_support),
        "max_com_zmp_gap_m": max(((s.com_xy[0]-s.zmp_xy[0])**2 +
                                   (s.com_xy[1]-s.zmp_xy[1])**2)**0.5 for s in tail_support),
        "final_zmp_xy": tail_support[-1].zmp_xy,
        "final_com_xy": tail_support[-1].com_xy,
        "final_target_zmp_xy": tail_support[-1].target_zmp_xy,
    }
    # Registered preliminary simulation thresholds; not hardware specifications.
    limits = {"max_abs_vx_kph": 0.036, "max_abs_wheel_rpm": 1.0,
              "max_abs_roll_rate_deg_s": 0.1, "max_abs_pitch_rate_deg_s": 0.1,
              "min_fz_n": 300.0}
    passed = (all(metrics[k] <= limits[k] for k in limits if k != "min_fz_n") and
              min(metrics["min_fz_n"].values()) >= limits["min_fz_n"])
    return {"M1_status": "PASS" if passed else "FAIL",
            "M2_status": "MONITORED", "limits": limits, "metrics": metrics}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--duration-s", type=float, default=12.0)
    parser.add_argument("--settle-start-s", type=float, default=9.0)
    parser.add_argument("--brake-mpa", type=float, default=1.0)
    parser.add_argument("--lambda-safe", type=float, default=0.05)
    args = parser.parse_args()
    args.output = args.output.resolve()
    if args.output.exists():
        raise FileExistsError(args.output)
    simfile = prepare_model(args.output / "model", args.duration_s, args.brake_mpa)
    native = run_stepwise(simfile, lambda _t, _x: (0.0,) * 8,
                          args.output / "native_5ms.csv", IMPORT_NAMES,
                          EXPORT_NAMES + SCENARIO_EXPORTS + GEOMETRY_EXPORTS,
                          log_decimation=10)
    result = analyze(args.output / "native_5ms.csv",
                     settle_start_s=args.settle_start_s, lambda_safe=args.lambda_safe)
    result["native"] = native
    result["model_sha256"] = hashlib.sha256((simfile.parent / "run_all.par").read_bytes()).hexdigest()
    result["source_sha256"] = hashlib.sha256((SOURCE / "run_all.par").read_bytes()).hexdigest()
    result["brake_mpa"] = args.brake_mpa
    (args.output / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
