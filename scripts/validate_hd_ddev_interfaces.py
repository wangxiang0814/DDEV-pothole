from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from ddevsim.hd_ddev_case import DDEV_EXPORTS
from ddevsim.interface_validation import (
    EXPORT_NAMES,
    analyze_validation_directory,
    run_validation_suite,
)
from ddevsim.pothole_case import SCENARIO_EXPORTS
from ddevsim.solver_api import parse_simfile


def exports_for(simfile: Path):
    """Return the export list that matches the simfile's declared PORTS_EXP.

    A control object built for the scenario carries the pose and wheel-station
    channels as well, so the suite must send exactly as many names as the solver
    declares or it aborts on the port-count check.
    """
    declared = int(parse_simfile(simfile).get("PORTS_EXP", len(DDEV_EXPORTS)))
    names = list(EXPORT_NAMES)
    if declared >= len(names) + len(SCENARIO_EXPORTS):
        names = names + list(SCENARIO_EXPORTS)
    names = names[:declared]
    if len(names) != declared:
        raise SystemExit(
            "simfile declares PORTS_EXP %d but only %d export names are known"
            % (declared, len(names))
        )
    return names


def main() -> int:
    parser = argparse.ArgumentParser(description="Run 17 same-vehicle HD DDEV interface cases")
    parser.add_argument("--simfile", type=Path, default=ROOT / "models" / "hd_utility_ddev" / "simfile.sim")
    parser.add_argument(
        "--target", type=Path, default=ROOT / "runs" / "hd_utility_ddev_interface_validation"
    )
    args = parser.parse_args()
    manifest = run_validation_suite(
        args.simfile, args.target, export_names=exports_for(args.simfile)
    )
    verification = analyze_validation_directory(args.target)
    statuses = {name: result["status"] for name, result in manifest["results"].items()}
    print(json.dumps(statuses, ensure_ascii=False, indent=2))
    completed = all(status == "COMPLETED" for status in statuses.values())
    return 0 if completed and verification["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
