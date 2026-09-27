"""Validate measured near-zero FR load and three-wheel static support."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from run_static_fr_small_unload import _support_margin
from ddevsim.static_wheel_lift.config import (
    COUPLED_PROBE, HOLD_VALIDATION, PRELOAD_GATE,
)


WHEELS = ("L1", "R1", "L2", "R2")


def validate_rows(rows: list[dict[str, str]]) -> dict:
    longest: list[dict[str, float]] = []
    current: list[dict[str, float]] = []
    for row in rows:
        export = {key[4:]: float(value) for key, value in row.items()
                  if key.startswith("exp_")}
        t = float(row["time_s"])
        loads = [export[f"Fz_{wheel}"] for wheel in WHEELS]
        travels = [export[f"Jnc_{wheel}"] for wheel in WHEELS]
        margin = _support_margin(export)
        valid = (
            0 <= loads[1] <= HOLD_VALIDATION.near_zero_fr_n and
            min(loads[i] for i in (0, 2, 3)) >= COUPLED_PROBE.min_wheel_load_n and
            margin >= PRELOAD_GATE.lambda_safe and
            min(travels) >= COUPLED_PROBE.min_travel_mm and
            max(travels) <= COUPLED_PROBE.max_travel_mm and
            max(abs(export["Roll_E"]), abs(export["Pitch"])) <=
            COUPLED_PROBE.max_attitude_deg and
            abs(export["Vx"]) <= COUPLED_PROBE.max_vx_kph and
            max(abs(export[f"AVy_{wheel}"]) for wheel in WHEELS) <=
            COUPLED_PROBE.max_wheel_rpm)
        if valid:
            current.append({"time_s": t, "fr_n": loads[1],
                            "support_min_n": min(loads[i] for i in (0, 2, 3)),
                            "lambda_min": margin,
                            "min_travel_mm": min(travels),
                            "max_attitude_deg": max(abs(export["Roll_E"]),
                                                    abs(export["Pitch"]))})
        else:
            if len(current) > len(longest):
                longest = current
            current = []
    if len(current) > len(longest):
        longest = current
    duration = (longest[-1]["time_s"] - longest[0]["time_s"]
                if len(longest) >= 2 else 0.)
    return {
        "status": "PASS" if duration >= HOLD_VALIDATION.required_hold_s else "FAIL",
        "near_zero_fr_n": HOLD_VALIDATION.near_zero_fr_n,
        "required_hold_s": HOLD_VALIDATION.required_hold_s,
        "hold_duration_s": duration,
        "hold_start_s": longest[0]["time_s"] if longest else None,
        "hold_end_s": longest[-1]["time_s"] if longest else None,
        "min_support_load_n": min((p["support_min_n"] for p in longest),
                                  default=None),
        "min_lambda": min((p["lambda_min"] for p in longest), default=None),
        "min_travel_mm": min((p["min_travel_mm"] for p in longest),
                             default=None),
        "max_attitude_deg": max((p["max_attitude_deg"] for p in longest),
                                default=None),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result", type=Path)
    args = parser.parse_args()
    result = json.loads(args.result.read_text(encoding="utf-8"))
    with Path(result["native"]["csv"]).open(encoding="utf-8", newline="") as stream:
        report = validate_rows(list(csv.DictReader(stream)))
    report["source_result"] = str(args.result.resolve())
    report["abort_time_s"] = result["abort_time_s"]
    report["allocator_status"] = result["allocator"]["status"]
    if result["abort_time_s"] is not None or report["allocator_status"] != "OPTIMAL":
        report["status"] = "FAIL"
    target = args.result.parent / "hold_validation.json"
    target.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
