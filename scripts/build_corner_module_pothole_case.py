"""Build the deep-pothole scenario on the corner-module DDEV control object.

The pothole geometry is rescaled for the corner-module vehicle, because the original
case was sized for an 8.9 t truck on 565 mm-radius tyres and this vehicle is a light
commercial utility truck on 263 mm-radius tyres:

======================  ==============  ==============
quantity                HD truck        corner module
======================  ==============  ==============
wheelbase               3.900 m         1.925 m
track                   1.975 m         1.260 m
tyre radius             0.565 m         0.263 m
pothole length          1.200 m         0.800 m
pothole width           0.800 m         0.600 m
pothole depth           0.450 m         0.200 m
======================  ==============  ==============

0.45 m would be deeper than this vehicle's entire tyre radius and would swallow the
wheel completely; 0.20 m still exceeds its rebound travel, so the lift strategy is
still the required response and the depth-threshold activation triggers.

Usage
-----
    $env:PYTHONPATH='src'; python scripts\\build_corner_module_pothole_case.py
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from ddevsim.pothole_case import (  # noqa: E402
    PotholeScenario,
    build_single_wheel_pothole_case,
    corner_module_scenario,
)

#: Scenario for this vehicle class.  Built from the library's own factory rather than
#: repeated here: the inline copy this file used to carry had already drifted from
#: ``corner_module_scenario`` (it silently ignored a camera-framing change made there),
#: which is exactly the kind of duplication that makes a build disagree with its source.
CORNER_MODULE_SCENARIO = corner_module_scenario()


def main() -> int:
    base = ROOT / "models" / "corner_module_ddev"
    target = base / "single_wheel_deep_pothole"
    artifacts = build_single_wheel_pothole_case(
        base / "run_all.par",
        base / "simfile.sim",
        target,
        scenario=CORNER_MODULE_SCENARIO,
    )
    print(json.dumps({key: str(value) for key, value in artifacts.items()}, ensure_ascii=False, indent=2))
    print()
    print("scenario:")
    for key, value in CORNER_MODULE_SCENARIO.__dict__.items():
        print("  %-26s %s" % (key, value))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
