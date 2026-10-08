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
                   road_friction: float | None = None, config=CFG) -> tuple[Path, dict]:
    if road_friction is not None and (not np.isfinite(road_friction) or
                                      road_friction <= 0.):
        raise ValueError("road friction must be positive and finite")
    if output.exists():
        raise FileExistsError(output)
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
        # The stock Pacejka low-speed regularizer is 2 m/s: at a sub-1 km/h
        # crawl it suppresses usable longitudinal tyre force.  This isolated
        # simulation-only copy resolves slip at the intended speed instead.
        for key in ("VLOW_KAPPA", "VLOW_ALPHA"):
            content, changed = re.subn(rf"(?m)^{key} 2$",
                                       f"{key} {config.tyre_low_speed_slip_m_s:g}",
                                       content)
            if changed != 4:
                raise ValueError(f"expected four {key} tyre parameters")
        # The isolated I_I suspension geometry is simulation-configurable.  The
        # original -100 mm rebound stop is hit during a 3 km/h launch; extend the
        # stop table for this moving study while keeping the static case intact.
        old_rebound = "F_REB_STOP_TABLE LINEAR\n-100, -7000\n-99, 0\n-90, 0"
        new_rebound = "F_REB_STOP_TABLE LINEAR\n-150, -7000\n-149, 0\n-140, 0"
        if content.count(old_rebound) != 4:
            raise ValueError("expected four I_I rebound-stop tables")
        content = content.replace(old_rebound, new_rebound)
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

def _enable_steering_import(model: Path) -> None:
    """Extend only this copied run with TruckSim's steering wheel angle input."""
    path = model / "run_all.par"
    content = path.read_text(encoding="utf-8")
    anchor = "IMPORT IMP_FS_R2 Add 0.0! 0\n"
    if content.count(anchor) != 1:
        raise ValueError("expected one final suspension import")
    path.write_text(content.replace(anchor, anchor +
                                    "IMPORT IMP_STEER_SW Replace 0.0! 0\n"),
                    encoding="utf-8")
    simfile = model / "simfile.sim"
    content = simfile.read_text(encoding="utf-8")
    if content.count("PORTS_IMP 8") != 1:
        raise ValueError("expected eight original TruckSim imports")
    simfile.write_text(content.replace("PORTS_IMP 8", "PORTS_IMP 9"),
                       encoding="utf-8")


def _clamp_kinematics_at_original_bounds(model: Path) -> None:
    """Diagnostic: avoid extrapolating the original ±70 mm ride tables."""
    path = model / "run_all.par"
    content = path.read_text(encoding="utf-8")
    names = ("SUSP_DIVE_TABLE", "SUSP_X_TABLE", "CAMBER_TABLE",
             "SUSP_LAT_TABLE", "TOE_TABLE")
    changed = 0
    for name in names:
        pattern = rf"(?m)^({name} SPLINE\n)((?:[-+0-9.eE]+, [-+0-9.eE]+\n)+)(ENDTABLE)$"

        def extend(match):
            nonlocal changed
            lines = match.group(2).splitlines()
            if len(lines) < 2 or not lines[0].startswith("-70, ") or not lines[-1].startswith("70, "):
                raise ValueError(f"unexpected {name} endpoints")
            changed += 1
            lower = lines[0].split(", ", 1)[1]
            upper = lines[-1].split(", ", 1)[1]
            return (match.group(1) + f"-150, {lower}\n" + match.group(2) +
                    f"150, {upper}\n" + match.group(3))

        content = re.sub(pattern, extend, content)
    if changed != 20:
        raise ValueError(f"expected 20 independent ride tables, found {changed}")
    path.write_text(content, encoding="utf-8")


def _scale_lateral_ride_movement(model: Path, factor: float) -> None:
    """Scale lateral wheel-centre movement in the copied I_I suspension."""
    if not 0.0 <= factor <= 1.0:
        raise ValueError("lateral ride scale must be within [0, 1]")
    path = model / "run_all.par"
    content = path.read_text(encoding="utf-8")
    pattern = r"(?m)^(SUSP_LAT_TABLE SPLINE\n)((?:[-+0-9.eE]+, [-+0-9.eE]+\n)+)(ENDTABLE)$"

    def scale(match):
        points = "".join(
            f"{line.split(', ', 1)[0]}, {float(line.split(', ', 1)[1]) * factor:.9g}\n"
            for line in match.group(2).splitlines())
        return match.group(1) + points + match.group(3)

    content, count = re.subn(pattern, scale, content)
    if count != 4:
        raise ValueError(f"expected four lateral kinematic tables, found {count}")
    path.write_text(content, encoding="utf-8")


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

