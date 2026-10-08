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
from ddevsim.visual_mesh import write_visual_mesh
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


def prepare_model(output: Path, *, crawl: bool,
                   road_friction: float | None = None, config=CFG,
                   claimed_output: bool = False) -> tuple[Path, dict]:
    if road_friction is not None and (not np.isfinite(road_friction) or
                                      road_friction <= 0.):
        raise ValueError("road friction must be positive and finite")
    if output.exists():
        if not claimed_output:
            raise FileExistsError(output)
        from ddevsim.runtime.run_workspace import verify_claimed_output
        verify_claimed_output(output)
    source = EVIDENCE / "model"
    shutil.copytree(source, output / "model", ignore=shutil.ignore_patterns("output"))
    model = output / "model"
    (model / "output").mkdir()
    content = (model / "run_all.par").read_text(encoding="utf-8", errors="replace")
    scenario = json.loads((source / "scenario.json").read_text(encoding="utf-8"))
    if crawl:
        pit_text = (PIT / "run_all.par").read_text(encoding="utf-8", errors="replace")
        pattern = r"(?m)^ROAD_DZ_CARPET 2D_LINEAR\s*\n(?:.*\n)*?ENDTABLE"
        pit_grid = re.search(pattern, pit_text)
        if pit_grid is None or len(re.findall(pattern, content)) != 1:
            raise ValueError("cannot locate unique TruckSim physical road grid")
        content = re.sub(pattern, lambda _: pit_grid.group(), content, count=1)
        shape_start = r"ENTER_PARSFILE Roads\Shapes\DDEV_single_wheel_pothole.par"
        shape_end = r"EXIT_PARSFILE Roads\Shapes\DDEV_single_wheel_pothole.par"
        for source_text in (content, pit_text):
            if source_text.count(shape_start) != 1 or source_text.count(shape_end) != 1:
                raise ValueError("expected one native pothole visual shape block")
        old_end = content.index(shape_end) + len(shape_end)
        new_end = pit_text.index(shape_end) + len(shape_end)
        content = (content[:content.index(shape_start)] +
                   pit_text[pit_text.index(shape_start):new_end] +
                   content[old_end:])
        content = configure_crawl_text(content, config.tyre_low_speed_slip_m_s)
        scenario = json.loads((PIT / "scenario.json").read_text(encoding="utf-8"))
        if road_friction is not None:
            old_mu = f'MU_ROAD_CONSTANT {scenario["friction"]:g}'
            if content.count(old_mu) != 1:
                raise ValueError("expected one physical road-friction value")
            content = content.replace(old_mu,
                                      f"MU_ROAD_CONSTANT {road_friction:g}")
            scenario["friction"] = road_friction
        shutil.copytree(PIT / "visual_assets", model / "visual_assets",
                        dirs_exist_ok=True)
        scenario["target_speed_kph"] = config.crawl_speed_kph
    scenario["controller_period_s"] = config.control_period_s
    content, count = re.subn(r"(?m)^TSTOP\s+[-+0-9.eE]+\s*$",
                             "TSTOP 125" if crawl else "TSTOP 82", content)
    if count != 2:
        raise ValueError("unexpected TruckSim stop-time declarations")
    (model / "run_all.par").write_text(content, encoding="utf-8")
    (model / "scenario.json").write_text(
        json.dumps(scenario, indent=2) + "\n", encoding="utf-8")
    return model, scenario



from ddevsim.static_wheel_lift.config import RIGHT_SIDE_MODEL, TUNED_REAR_RUN
from ddevsim.pothole_case import corner_module_scenario
from ddevsim.static_wheel_lift.pit_scene_transfer import replace_pit_geometry_parameters

def _set_pit_geometry(model: Path, scenario: dict,
                      *, width_m: float, depth_m: float) -> None:
    """Regenerate the copied road and its display mesh for one boundary case."""
    if not (0.0 < width_m < RIGHT_SIDE_MODEL.max_boundary_pit_width_m and
            0.0 < depth_m < TUNED_REAR_RUN.tyre_radius_m):
        raise ValueError("pit boundary case would reach the other tyre or tyre centre")
    variant = corner_module_scenario(
        width_m=width_m, depth_m=depth_m,
        target_speed_kph=float(scenario["target_speed_kph"]))
    path = model / "run_all.par"
    content = path.read_text(encoding="utf-8")
    content = replace_pit_geometry_parameters(content, variant)
    path.write_text(content, encoding="utf-8")
    write_visual_mesh(model / "visual_assets", variant)
    scenario["width_m"], scenario["depth_m"] = width_m, depth_m
    (model / "scenario.json").write_text(json.dumps(scenario, indent=2) + "\n",
                                          encoding="utf-8")


def _set_vehicle_start_offset(model: Path, scenario: dict, offset_m: float) -> None:
    """Move the copied vehicle start while leaving road and pit fixed."""
    path = model / "run_all.par"
    content = path.read_text(encoding="utf-8")
    pattern = r"(?m)^SSTART\s+([-+0-9.eE]+)\s*$"
    starts = [float(value) for value in re.findall(pattern, content)]
    if len(starts) != 2 or abs(starts[0] - starts[1]) > 1e-9:
        raise ValueError("expected two identical vehicle starting stations")
    new_start = starts[0] + offset_m
    content = re.sub(pattern, f"SSTART {new_start:g}", content)
    path.write_text(content, encoding="utf-8")
    scenario["vehicle_start_station_m"] = new_start
    scenario["approach_distance_m"] = float(scenario["start_station_m"]) - new_start
    (model / "scenario.json").write_text(json.dumps(scenario, indent=2) + "\n",
                                          encoding="utf-8")


from ddevsim.vehicle_profiles.transformations import (configure_crawl_text,
    enable_steering_import as _enable_steering_import,
    clamp_kinematics_at_original_bounds as _clamp_kinematics_at_original_bounds,
    scale_lateral_ride_movement as _scale_lateral_ride_movement)
