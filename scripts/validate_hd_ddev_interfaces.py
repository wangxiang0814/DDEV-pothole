from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from ddevsim.interface_validation import analyze_validation_directory, run_validation_suite


def main() -> int:
    parser = argparse.ArgumentParser(description="Run 17 same-vehicle HD DDEV interface cases")
    parser.add_argument("--simfile", type=Path, default=ROOT / "models" / "hd_utility_ddev" / "simfile.sim")
    parser.add_argument(
        "--target", type=Path, default=ROOT / "runs" / "hd_utility_ddev_interface_validation"
    )
    args = parser.parse_args()
    manifest = run_validation_suite(args.simfile, args.target)
    verification = analyze_validation_directory(args.target)
    statuses = {name: result["status"] for name, result in manifest["results"].items()}
    print(json.dumps(statuses, ensure_ascii=False, indent=2))
    completed = all(status == "COMPLETED" for status in statuses.values())
    return 0 if completed and verification["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
