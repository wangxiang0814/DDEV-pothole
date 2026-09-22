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

from ddevsim.pothole_case import PotholeScenario, build_single_wheel_pothole_case  # noqa: E402

#: Grid of pothole geometry for this vehicle class.
CORNER_MODULE_SCENARIO = PotholeScenario(
    start_station_m=101.1,
    length_m=0.8,
    width_m=0.6,
    depth_m=0.20,
    edge_transition_m=0.05,
    # right-wheel track centre: -L_TRACK/2 = -0.630 m
    center_y_m=-0.63,
    friction=0.7,
    road_length_m=40.0,
    stop_s=9.0,
    target_speed_kph=2.8,
    # Camera framing scaled to a 1.9 m wheelbase vehicle: a 3.5 m vehicle at a 26 deg
    # lens needs far less than the truck's 32 m stand-off to fill a third of frame.
    camera_distance_m=16.0,
    camera_field_of_view_deg=30.0,
    camera_elevation_deg=16.0,
    camera_look_z_m=0.6,
)


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
