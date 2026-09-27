"""Isolated I_I flat-road baseline with one repositioned 200 kg payload."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

from ddevsim.cosim import run_stepwise
from ddevsim.interface_validation import IMPORT_NAMES, EXPORT_NAMES
from ddevsim.pothole_case import SCENARIO_EXPORTS
from ddevsim.contact_geometry import GEOMETRY_EXPORTS
from ddevsim.static_wheel_lift.mass_study import shift_rear_payload
from run_static_fr_m1_m2 import analyze
from run_static_fr_m3 import response


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path,
                        default=ROOT / "runs/fr_static_ii_m1_braked_repro")
    parser.add_argument("--output", type=Path,
                        default=ROOT / "runs/fr_static_ii_ballast_rear150_left230")
    parser.add_argument("--rearward-mm", type=float, default=150.0)
    parser.add_argument("--leftward-mm", type=float, default=230.0)
    args = parser.parse_args()
    args.output = args.output.resolve()
    source = args.source / "model"
    m1 = json.loads((args.source / "result.json").read_text(encoding="utf-8"))
    digest = hashlib.sha256((source / "run_all.par").read_bytes()).hexdigest()
    if m1["M1_status"] != "PASS" or m1["model_sha256"] != digest:
        raise RuntimeError("requires matching braked I_I M1 source")
    if args.output.exists():
        raise FileExistsError(args.output)
    shutil.copytree(source, args.output / "model", ignore=shutil.ignore_patterns("output"))
    (args.output / "model/output").mkdir()
    model_file = args.output / "model/run_all.par"
    text = shift_rear_payload(model_file.read_text(encoding="utf-8", errors="replace"),
                              rearward_mm=args.rearward_mm, leftward_mm=args.leftward_mm)
    text, count = re.subn(r"(?m)^TSTOP 12$", "TSTOP 20", text)
    if count != 2:
        raise RuntimeError("unexpected M1 duration declaration")
    model_file.write_text(text, encoding="utf-8")
    model_hash = hashlib.sha256(model_file.read_bytes()).hexdigest()
    csv_path = args.output / "native_5ms.csv"
    native = run_stepwise(args.output / "model/simfile.sim", lambda _t, _x: (0.0,) * 8,
                          csv_path, IMPORT_NAMES,
                          EXPORT_NAMES + SCENARIO_EXPORTS + GEOMETRY_EXPORTS,
                          log_decimation=10)
    settled = analyze(csv_path, settle_start_s=17.0, lambda_safe=0.05)
    result = {
        "vehicle": "corner_module_ddev I_I",
        "source_model_sha256": digest, "model_sha256": model_hash,
        "total_mass_kg": 1360, "moved_payload_mass_kg": 200,
        "rearward_mm": args.rearward_mm, "leftward_mm": args.leftward_mm,
        "model_changes": "rear 200 kg payload LX/Y and visual offset, plus TSTOP in isolated copy",
        "native": native, "M1_status": settled["M1_status"],
        "M2_status": settled["M2_status"], "metrics": settled["metrics"],
        "response_17_18s": response(csv_path, 17.0, 18.0),
    }
    (args.output / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
