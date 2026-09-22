"""Build the corner-module DDEV control object from a TruckSim source case.

Why this replaces the HD Utility Vehicle
----------------------------------------
The HD Utility Vehicle is ``VEHICLE_CODE s_s``: solid (rigid) axles front *and*
rear.  Its eight actuator channels are independently *addressable* — that was
verified — but the wheel vertical response is not independent, because the two
wheels on each axle are joined by a rigid beam.  Measured on that model, a force at
one spring seat changes the diagonally opposite wheel's load by 1.5x more than its
own corner's, and building the paper's three-point support with actuators would
have needed roughly 300 kN per corner.

The paper being reproduced (Liu et al., IEEE/ASME ToM 2024) uses a corner-module
vehicle whose four corners are genuinely independent.  This script therefore builds
the control object from TruckSim's ``Compact Utility Truck (I_I)``:

* ``VEHICLE_CODE i_i`` — **independent front and independent rear**;
* ``Suspension: Independent System Kinematics`` datasets on both axles;
* a separate spring/damper compliance dataset per axle, i.e. a spring per corner.

so a force at one corner's spring seat acts on that corner alone, and the four
corner heights can be commanded independently — a true corner-module DDEV.

Usage
-----
    $env:PYTHONPATH='src'; python scripts\\build_corner_module_ddev.py
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from ddevsim.channels import write_contract  # noqa: E402
from ddevsim.hd_ddev_case import build_hd_ddev_case, detect_vehicle_code  # noqa: E402
from ddevsim.pothole_case import SCENARIO_EXPORTS  # noqa: E402

#: TruckSim's cached run of ``Compact Utility Truck (I_I)``: a single-unit, two-axle
#: utility truck with independent suspension at both axles and no trailer.  Two
#: cached runs exist (Run_b20aee53..., Run_e5020644...) and both are the same vehicle.
DEFAULT_SOURCE = Path(
    r"F:\TruckSim2019\TruckSim2019.0_Data\Results"
    r"\Run_b20aee53-c150-44b1-a804-20e43bfde604\run_all.par"
)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--target", type=Path, default=ROOT / "models" / "corner_module_ddev")
    parser.add_argument("--stop-s", type=float, default=1.0)
    parser.add_argument(
        "--tyre-load-reference-n",
        type=float,
        default=None,
        help="Rescale FZ_REF only if the stock rating is below the static corner "
             "load. The Compact Utility Truck's 4100 N rating already exceeds its "
             "~2354 N static corner load, so the default leaves it unchanged.",
    )
    parser.add_argument(
        "--steer-spring-rate-n-per-mm",
        type=float,
        default=0.0,
        help=(
            "Front spring rate. The stock dataset uses 30 N/mm while this vehicle's "
            "static front corner load is ~3483 N, leaving it at ~116-168 mm of "
            "deflection against an ~80 mm suspension kinematic table, so the solver "
            "extrapolates the front geometry from t=0 (the shipped source case's own "
            "log reports it). 70 N/mm puts the static ride height inside the tables. "
            "Pass 0 (the default) to keep the stock rate."
        ),
    )
    args = parser.parse_args(argv)

    rate = args.steer_spring_rate_n_per_mm or None

    source_text = args.source.read_text(encoding="utf-8", errors="replace")
    code = detect_vehicle_code(source_text)
    if code != "I_I":
        raise SystemExit(
            "source vehicle code is %s, but a corner-module control object requires "
            "I_I (independent front and rear)" % code
        )

    artifacts = build_hd_ddev_case(
        args.source,
        args.target,
        program_dir=Path(r"F:\TruckSim2019\TruckSim2019.0_Prog"),
        data_dir=Path(r"F:\TruckSim2019\TruckSim2019.0_Data"),
        stop_s=args.stop_s,
        tyre_load_reference_n=args.tyre_load_reference_n,
        # The expert controller's support-phase state machine needs the wheel-centre
        # stations, and the QA gates need roll/pitch, so the control object exports
        # the scenario channels alongside the 16 DDEV feedback channels.
        extra_exports=SCENARIO_EXPORTS,
        steer_spring_rate_n_per_mm=rate,
    )
    write_contract(args.target / "interface_contract.json")

    manifest = json.loads((args.target / "source_manifest.json").read_text(encoding="utf-8"))
    arch = manifest["suspension_architecture"]
    print("vehicle code          : %s" % arch["vehicle_code"])
    print("front axle            : %s" % arch["front_axle"])
    print("rear axle             : %s" % arch["rear_axle"])
    print("corners independent   : %s" % arch["corners_mechanically_independent"])
    print("kinematics datasets   : %s" % ", ".join(arch["independent_kinematics_datasets"]))
    print("compliance datasets   : %s" % ", ".join(arch["compliance_datasets"]))
    print("powertrain            : %s" % manifest["powertrain"])
    print("ports                 : imp %d / exp %d" % (manifest["ports_import"], manifest["ports_export"]))
    values = manifest.get("parameter_values", {})
    for key in ("M_SU", "M_PL", "M_US", "IXX_SU", "IYY_SU", "IZZ_SU", "L_TRACK", "TSTEP"):
        if values.get(key):
            print("  %-9s %s" % (key, values[key]))
    print()
    print("artifacts:")
    for key, value in artifacts.items():
        print("  %-10s %s" % (key, value))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
