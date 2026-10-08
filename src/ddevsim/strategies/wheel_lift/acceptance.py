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



from ddevsim.static_wheel_lift.rear_cycle import RearCycleController

def evaluate_control_rows(rows: list[dict], *, crawl: bool, scenario: dict,
                          native_completed: bool,
                          abort_reason: str | None,
                          path_reference_yo_m: float | None = None,
                          path_reference_yaw_deg: float | None = None, config=CFG) -> dict:
    """Apply phase-specific acceptance to measured native control samples."""
    if not rows:
        return {"status": "FAIL", "criteria": {"has_samples": False}}
    hold = [r for r in rows if r["mode"] == "THREE_WHEEL_HOLD"]
    moving = [r for r in rows if r["mode"] in ("CRAWL", "STOP")]
    edge = float(scenario["start_station_m"])
    far_edge = edge + float(scenario["length_m"])
    pit = [r for r in moving if edge + 0.1 <= float(r["x_fr_m"]) <= far_edge - 0.1]
    lowering = [r for r in rows if r["mode"] == "LOWERING"]
    end = rows[-1]

    def min_value(samples, key):
        return min(float(r[key]) for r in samples) if samples else None

    def max_value(samples, key):
        return max(float(r[key]) for r in samples) if samples else None

    hold_start = float(hold[0]["time_s"]) if hold else None
    after_hold = next((float(r["time_s"]) for r in rows
                       if hold_start is not None and float(r["time_s"]) > hold_start
                       and r["mode"] != "THREE_WHEEL_HOLD"), None)
    hold_duration = after_hold - hold_start if after_hold is not None else 0.
    support_keys = ("fz_fl_n", "fz_rl_n", "fz_rr_n")
    min_support = (min(float(r[k]) for r in hold + moving for k in support_keys)
                   if hold or moving else None)
    baseline_yo = (float(rows[0]["yo_m"]) if path_reference_yo_m is None else path_reference_yo_m)
    baseline_yaw = (float(rows[0]["yaw_deg"]) if path_reference_yaw_deg is None else path_reference_yaw_deg)
    lateral_max = (max(abs(float(r["yo_m"]) - baseline_yo) for r in moving)
                   if moving else None)
    yaw_max = (max(abs(float(r["yaw_deg"]) - baseline_yaw) for r in moving)
               if moving else None)
    saturation_start = None
    last_saturation_time = None
    longest_saturation_s = 0.
    for sample in hold + moving:
        instant = float(sample["time_s"])
        saturated = sample.get("support_feedback_saturated", False) in (True, "True", "1", 1)
        if saturated:
            if (saturation_start is None or last_saturation_time is not None and
                    instant - last_saturation_time > 1.5 * config.control_period_s):
                saturation_start = instant
            last_saturation_time = instant
            longest_saturation_s = max(
                longest_saturation_s,
                instant - saturation_start + config.control_period_s)
        else:
            saturation_start = None
            last_saturation_time = None
    criteria = {
        "native_completed": native_completed,
        "no_abort": abort_reason is None,
        "fsm_complete": end["mode"] == "COMPLETE",
        "hold_at_least_5_s": hold_duration >= config.three_wheel_hold_s - 1e-6,
        "hold_fr_clearance": bool(hold) and min_value(hold, "fr_top_clearance_m") >= config.lift_clearance_m,
        "hold_fr_unloaded": bool(hold) and max_value(hold, "fz_fr_n") <= config.fr_swing_load_n,
        "support_wheels_loaded": min_support is not None and min_support >= config.support_floor_n,
        "support_triangle_zmp": bool(hold) and min_value(hold + moving, "zmp_lambda_min") >= config.lambda_target,
        "support_triangle_com": bool(hold) and min_value(hold + moving, "com_lambda_min") >= config.lambda_target,
        "four_wheel_recovered": all(float(end[k]) >= config.support_floor_n
                                    for k in ("fz_fl_n", "fz_fr_n", "fz_rl_n", "fz_rr_n")),
        "no_sustained_support_saturation":
            longest_saturation_s <= config.max_continuous_support_saturation_s,
    }
    if crawl:
        criteria.update(front_restoration_checks(rows,config=config,scenario=scenario,
                        path_reference_yo_m=baseline_yo))
        criteria.update({
            "pit_crossed": float(end["x_fr_m"]) >= far_edge + config.crossing_clearance_m,
            "pit_speed_in_requested_range": bool(pit) and
            min_value(pit, "vx_kph") >= config.crawl_min_speed_kph and
            max_value(pit, "vx_kph") <= config.crawl_max_speed_kph,
            "lifted_fr_over_pit": bool(pit) and
            min_value(pit, "fr_top_clearance_m") >= config.lift_clearance_m and
            max_value(pit, "fz_fr_n") <= config.fr_swing_load_n,
            "straight_path": lateral_max is not None and
            lateral_max <= config.crawl_max_lateral_error_m and
            yaw_max <= config.crawl_max_yaw_error_deg,
            "stopped_before_lowering": bool(lowering) and
            abs(float(lowering[0]["vx_kph"])) <= config.stop_speed_kph,
            "rear_stopped_before_pit": bool(lowering) and
            float(lowering[0]["x_rr_m"]) <=
            edge - config.rear_stop_clearance_m,
        })
    return {
        "status": "PASS" if all(criteria.values()) else "FAIL",
        "criteria": criteria,
        "hold_duration_s": hold_duration,
        "pit_speed_min_kph": min_value(pit, "vx_kph"),
        "pit_speed_max_kph": max_value(pit, "vx_kph"),
        "pit_clearance_min_m": min_value(pit, "fr_top_clearance_m"),
        "min_support_load_n": min_support,
        "moving_lateral_max_m": lateral_max,
        "moving_yaw_max_deg": yaw_max,
        "longest_support_saturation_s": longest_saturation_s,
    }


def evaluate_full_cycle(front_result: dict, rear: RearCycleController,
                        scenario: dict, native_completed: bool,
                        truth_rows: list[dict] | None = None,
                        path_reference_yo_m: float | None = None,
                        path_reference_yaw_deg: float | None = None) -> dict:
    rows = rear.rows if truth_rows is None else truth_rows
    cfg = rear.config
    edge = float(scenario["start_station_m"])
    far_edge = edge + float(scenario["length_m"])
    hold = [row for row in rows if row["mode"] == "RR_HOLD"]
    moving = [row for row in rows if row["mode"] in ("RR_CRAWL", "RR_STOP")]
    pit = [row for row in moving if
           edge + cfg.pit_evaluation_edge_trim_m <= row["x_rr_m"] <=
           far_edge - cfg.pit_evaluation_edge_trim_m]
    lowering = next((row for row in rows if row["mode"] == "RR_LOWERING"), None)
    end = rows[-1] if rows else None
    hold_duration = ((next((row["time_s"] for row in rows if
                            row["mode"] != "RR_HOLD" and
                            row["time_s"] > hold[0]["time_s"]),
                           hold[-1]["time_s"]) - hold[0]["time_s"])
                     if hold else 0.)
    stability = hold + moving
    recovery = [row for row in rows if row['mode'] in ('RR_LOWERING','RR_RETURN','RR_COMPLETE')]
    min_support = min((row["min_support_n"] for row in stability), default=None)
    min_zmp = min((float(row["zmp_lambda_min"]) for row in stability), default=None)
    min_com = min((float(row["com_lambda_min"]) for row in stability), default=None)
    min_clearance = min((row["rr_clearance_m"] for row in pit), default=None)
    path_yo = rear.initial_yo_m if rear.initial_yo_m is not None else (rows[0]["yo_m"] if rows else 0.)
    path_yaw = rear.initial_yaw_deg if rear.initial_yaw_deg is not None else (rows[0]["yaw_deg"] if rows else 0.)
    if path_reference_yo_m is not None:
        path_yo = path_reference_yo_m
    if path_reference_yaw_deg is not None:
        path_yaw = path_reference_yaw_deg
    max_lateral = (max(abs(row["yo_m"] - path_yo) for row in moving)
                   if moving else None)
    max_yaw = (max(abs(row["yaw_deg"] - path_yaw) for row in moving)
               if moving else None)
    max_roll = (max(abs(row["roll_deg"]) for row in stability)
                if stability else None)
    force_keys = tuple(f"fact_{wheel}_n" for wheel in ("fl", "fr", "rl", "rr"))
    max_force = max((abs(row[key]) for row in rows for key in force_keys),
                    default=None)
    max_slew = max((abs(row[key] - previous[key]) /
                    (row["time_s"] - previous["time_s"])
                    for previous, row in zip(rows, rows[1:])
                    for key in force_keys if row["time_s"] > previous["time_s"]),
                   default=None)
    saturation_start = None
    longest_saturation = 0.0
    for row in rows:
        if abs(row["preload_feedback_n"]) >= cfg.preload_feedback_limit_n - 1e-6:
            if saturation_start is None:
                saturation_start = row["time_s"]
            longest_saturation = max(
                longest_saturation,
                row["time_s"] - saturation_start + cfg.control_period_s)
        else:
            saturation_start = None
    static_shift = (abs(moving[0]["yo_m"] - rows[0]["yo_m"])
                    if moving else None)
    criteria = {
        "native_completed": native_completed,
        "fr_stage_pass": front_result["status"] == "PASS",
        "rr_fsm_complete": end is not None and end["mode"] == "RR_COMPLETE",
        "rr_no_abort": rear.abort_reason is None,
        "rr_no_sustained_feedback_saturation": (
            longest_saturation <= cfg.max_continuous_feedback_saturation_s),
        "rr_forces_within_sim_limits": (
            max_force is not None and max_force <= cfg.sim_force_limit_n and
            max_slew is not None and max_slew <= cfg.sim_force_slew_n_s),
        "rr_hold_at_least_5_s": hold_duration >= cfg.hold_s - 1e-6,
        "rr_hold_unloaded": bool(hold) and
        max(row["fz_rr_n"] for row in hold) <= cfg.wheel_unloaded_n,
        "rr_support_loaded": min_support is not None and
        bool(min_support >= cfg.support_floor_n),
        "rr_zmp_safe": min_zmp is not None and min_zmp >= cfg.lambda_safe,
        "rr_com_safe": min_com is not None and min_com >= cfg.lambda_safe,
        "rr_clear_over_pit": bool(pit) and min_clearance >= cfg.clearance_m and
        max(row["fz_rr_n"] for row in pit) <= cfg.wheel_unloaded_n,
        "rr_pit_speed": bool(pit) and
        min(row["vx_kph"] for row in pit) >= cfg.min_pit_speed_kph,
        "rr_straight": (max_lateral is not None and
                        max_lateral <= cfg.max_lateral_error_m),
        "rr_stopped_before_lowering": lowering is not None and
        abs(lowering["vx_kph"]) <= cfg.stationary_kph and
        lowering["x_rr_m"] >= far_edge + cfg.crossing_clearance_m,
        'rr_recovery_attitude_safe': bool(recovery) and
        max(abs(row[k]) for row in recovery for k in ('roll_deg','pitch_deg')) <= cfg.attitude_limit_deg,
        'rr_recovery_support_loaded': bool(recovery) and
        min(row['min_support_n'] for row in recovery) >= cfg.support_floor_n,
        'rr_recovery_travel_safe': bool(recovery) and
        min(row[f'travel_{c}_mm'] for row in recovery for c in ('fl','fr','rl','rr')) >= cfg.travel_rebound_abort_mm and
        max(row[f'travel_{c}_mm'] for row in recovery for c in ('fl','fr','rl','rr')) <= cfg.travel_jounce_abort_mm,
        'rr_recovery_straight': bool(recovery) and
        max(abs(row['yo_m']-path_yo) for row in recovery) <= cfg.max_lateral_error_m,
        'rr_recovery_stopped_on_solid_ground': bool(recovery) and
        all(abs(row['vx_kph']) <= cfg.stationary_kph and
            row['x_rr_m'] >= far_edge + cfg.crossing_clearance_m for row in recovery),
        "four_wheel_recovered": end is not None and
        min(end[f"fz_{wheel}_n"] for wheel in ("fl", "fr", "rl", "rr")) >=
        cfg.support_floor_n,
    }
    criteria = {key: bool(value) for key, value in criteria.items()}
    return {"status": "PASS" if all(criteria.values()) else "FAIL",
            "path_reference_yo_m": path_yo,
            "path_reference_yaw_deg": path_yaw,
            "rr_local_lateral_max_m": (max(abs(row["yo_m"] - rows[0]["yo_m"])
                                           for row in moving) if moving else None),
            "criteria": criteria,
            "rr_abort_reason": rear.abort_reason,
            "rr_hold_duration_s": hold_duration,
            "rr_min_support_n": min_support,
            "rr_min_zmp_lambda": min_zmp,
            "rr_min_com_lambda": min_com,
            "rr_pit_min_clearance_m": min_clearance,
            "rr_pit_speed_range_kph": ([min(row["vx_kph"] for row in pit),
                                        max(row["vx_kph"] for row in pit)]
                                       if pit else None),
            "rr_max_lateral_m": max_lateral,
            "rr_max_yaw_error_deg": max_yaw,
            "rr_max_abs_roll_deg": max_roll,
            "rr_max_actuator_force_n": max_force,
            "rr_max_actuator_slew_n_s": max_slew,
            "rr_longest_feedback_saturation_s": longest_saturation,
            "rr_static_lateral_shift_m": static_shift,
            "rr_last_x_m": end["x_rr_m"] if end else None}


