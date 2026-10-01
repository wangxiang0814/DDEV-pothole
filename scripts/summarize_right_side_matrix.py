"""Export compact native-run evidence without copying large TruckSim histories."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path


def summarize(directory: Path) -> dict:
    result = json.loads((directory / "result.json").read_text(encoding="utf-8"))
    with (directory / "rear_control_20ms.csv").open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    complete = next((float(r["time_s"]) for r in rows if r["mode"] == "RR_COMPLETE"), None)
    stops = [r for r in rows if r["mode"] == "RR_STOP"]
    native = result.pop("native", {})
    return {
        "run_name": directory.name,
        "model_run_all_sha256": hashlib.sha256(
            (directory / "model" / "run_all.par").read_bytes()).hexdigest(),
        "result": result,
        "cycle_complete_s": complete,
        "native_final_time_s": native.get("final_time_s"),
        "native_stop_on_complete": native.get("stopped_on_controller_complete", False),
        "stop_min_fr_travel_mm": min((float(r["travel_fr_mm"]) for r in stops), default=None),
        "stop_travel_relief_min_n": min((float(r.get("stop_travel_relief_n", 0.)) for r in rows), default=0.),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", type=Path, nargs="+")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    records = [summarize(path) for path in args.runs]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({"runs": records}, ensure_ascii=False, indent=2) + "\n",
                           encoding="utf-8")
    for record in records:
        print(f"{record['run_name']}: {record['result']['status']}")


if __name__ == "__main__":
    main()
