"""Run feedback-controlled I_I FR lift, optionally followed by three-wheel crawl.

The isolated lift evidence supplies the vehicle and preload feedforward.  Crawl
mode copies the already-built right-track pothole road into that same vehicle;
the original TruckSim projects and models are never modified.
"""

from __future__ import annotations

import argparse
import csv
import gzip
import hashlib
import json
import math
import re
import shutil
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "src"))

from ddevsim.cosim import run_stepwise
from ddevsim.contact_geometry import GEOMETRY_EXPORTS
from ddevsim.interface_validation import EXPORT_NAMES, IMPORT_NAMES
from ddevsim.pothole_case import SCENARIO_EXPORTS
from ddevsim.static_wheel_lift.closed_loop import (
    CrawlTorqueFeedback, PreloadProgress, ScalarLoadFeedback,
    SupportForceFeedback, blend_contact_gain, front_clearance_guard_m,
    front_abort_recovery_ready, front_restoration_checks,
    WheelLoadFilter,
)
from ddevsim.static_wheel_lift.command_trace import load_command_trace
from ddevsim.static_wheel_lift.config import CLOSED_LOOP_RUN as CFG, speed_trial_config
from ddevsim.static_wheel_lift.quintic_trajectory import quintic_step
from ddevsim.static_wheel_lift.support_geometry import assess_support


EXPORTS = EXPORT_NAMES + SCENARIO_EXPORTS + GEOMETRY_EXPORTS
WHEELS = ("L1", "R1", "L2", "R2")
SUPPORT = np.array([0, 2, 3])
EVIDENCE = ROOT / "evidence" / "static_fr_ii"
PIT = ROOT / "models" / "corner_module_ddev" / "single_wheel_deep_pothole"



from .front import LiftCrawlController, _reference, _barycentric
from .model_builder import prepare_model as _prepare_model
from .acceptance import evaluate_control_rows

def main(argv=None) -> None:
    config = CFG
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--crawl", action="store_true")
    parser.add_argument("--road-friction", type=float,
                        help="physical road mu in an isolated crawl copy")
    parser.add_argument("--target-speed-kph", type=float)
    parser.add_argument("--min-pit-speed-kph", type=float)
    args = parser.parse_args(argv)
    if (args.target_speed_kph is None) != (args.min_pit_speed_kph is None):
        parser.error("target and minimum pit speed must be supplied together")
    if args.target_speed_kph is not None:
        config = speed_trial_config(args.target_speed_kph, args.min_pit_speed_kph)
    output = args.output.resolve()
    model, scenario = _prepare_model(output, crawl=args.crawl,
                                     road_friction=args.road_friction, config=config)
    controller = LiftCrawlController(crawl=args.crawl, scenario=scenario, config=config)
    native = run_stepwise(model / "simfile.sim", controller,
                          output / "native_5ms.csv", IMPORT_NAMES, EXPORTS,
                          log_decimation=10)
    with (output / "control_20ms.csv").open("w", newline="", encoding="utf-8") as dest:
        writer = csv.DictWriter(dest, fieldnames=controller.rows[0].keys())
        writer.writeheader()
        writer.writerows(controller.rows)
    hold_rows = [r for r in controller.rows if r["mode"] == "THREE_WHEEL_HOLD"]
    moving_rows = [r for r in controller.rows if r["mode"] in ("CRAWL", "STOP")]
    result = {
        "native": native, "mode": controller.mode,
        "abort_reason": controller.abort_reason,
        "three_wheel_hold_s": (hold_rows[-1]["time_s"] - hold_rows[0]["time_s"]
                               if hold_rows else 0.),
        "min_hold_zmp_lambda": (min(r["zmp_lambda_min"] for r in hold_rows)
                                if hold_rows else None),
        "min_hold_com_lambda": (min(r["com_lambda_min"] for r in hold_rows)
                                if hold_rows else None),
        "min_crawl_zmp_lambda": (min(r["zmp_lambda_min"] for r in moving_rows)
                                 if moving_rows else None),
        "min_crawl_com_lambda": (min(r["com_lambda_min"] for r in moving_rows)
                                 if moving_rows else None),
        "max_crawl_lateral_error_m": (max(abs(r["yo_m"] - (controller.initial_yo_m or 0.))
                                          for r in moving_rows) if moving_rows else None),
        "max_crawl_yaw_error_deg": (max(abs(r["yaw_deg"] - (controller.initial_yaw_deg or 0.))
                                         for r in moving_rows) if moving_rows else None),
        "last_fr_x_m": controller.rows[-1]["x_fr_m"],
    }
    result.update(evaluate_control_rows(
        controller.rows, crawl=args.crawl, scenario=scenario,
        native_completed=native["status"] == "COMPLETED",
        abort_reason=controller.abort_reason, config=config))
    (output / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps({k: v for k, v in result.items() if k != "native"}, indent=2))


