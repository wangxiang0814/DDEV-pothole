from __future__ import annotations

import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from ddevsim.pothole_case import build_single_wheel_pothole_case


def main() -> int:
    artifacts = build_single_wheel_pothole_case(
        ROOT / "models" / "corner_module_ddev" / "run_all.par",
        ROOT / "models" / "corner_module_ddev" / "simfile.sim",
        ROOT / "models" / "corner_module_ddev" / "single_wheel_deep_pothole",
    )
    print(json.dumps({key: str(value) for key, value in artifacts.items()}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
