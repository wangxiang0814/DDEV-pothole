from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from ddevsim.solver_api import probe_solver


def main() -> int:
    parser = argparse.ArgumentParser(description="Probe the generated HD Utility DDEV case")
    parser.add_argument("--simfile", type=Path, default=ROOT / "models" / "hd_utility_ddev" / "simfile.sim")
    args = parser.parse_args()
    result = probe_solver(args.simfile)
    report = args.simfile.parent / "probe_report.json"
    report.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    config = result.get("configuration", {})
    ready = result.get("status") == "READY" and config.get("n_import") == 8 and config.get("n_export") == 16
    return 0 if ready else 2


if __name__ == "__main__":
    raise SystemExit(main())
