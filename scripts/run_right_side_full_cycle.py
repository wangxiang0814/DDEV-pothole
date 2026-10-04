"""Native TruckSim FR crossing followed by RR crossing and four-wheel return."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import re
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np

import run_static_fr_closed_loop as front_run

from ddevsim.cosim import run_stepwise
from ddevsim.pothole_case import corner_module_scenario, _procedure_block
from ddevsim.visual_mesh import write_visual_mesh
from ddevsim.static_wheel_lift.config import (REAR_CYCLE_RUN,
                                             RIGHT_SIDE_MODEL, TUNED_REAR_RUN,
                                             FAST_CYCLE_FRONT, FAST_CYCLE_REAR,
                                             BALANCED_CYCLE_FRONT, CLOSED_LOOP_RUN,
                                             COMPACT_CYCLE_FRONT, COMPACT_REAR_RETURN_RAMP_S,
                                             ROBUST_REAR_RUN,
                                             rear_speed_trial_config)
from ddevsim.static_wheel_lift.rear_cycle import RearCycleController
from ddevsim.static_wheel_lift.observation_noise import (
    ObservationNoiseConfig, NoisyFeedbackTrial)
from ddevsim.static_wheel_lift.system_identification import validated_contact_gains, validated_support_models
from ddevsim.static_wheel_lift.support_allocation_feedback import SupportAllocationFeedback
from ddevsim.static_wheel_lift.path_reference import StaticPathReference
from ddevsim.static_wheel_lift.config import PATH_REFERENCE

STEERING_IMPORT = "IMP_STEER_SW"


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
    generated = _procedure_block(variant)
    table_pattern = r"(?m)^ROAD_DZ_CARPET 2D_LINEAR\s*\n(?:.*\n)*?ENDTABLE"
    table = re.search(table_pattern, generated)
    if table is None:
        raise ValueError("generated pit has no physical road grid")
    content, count = re.subn(table_pattern, lambda _: table.group(), content)
    if count != 1:
        raise ValueError("expected one physical road grid")
    shape_start = r"ENTER_PARSFILE Roads\Shapes\DDEV_single_wheel_pothole.par"
    shape_end = r"EXIT_PARSFILE Roads\Shapes\DDEV_single_wheel_pothole.par"
    for source in (content, generated):
        if source.count(shape_start) != 1 or source.count(shape_end) != 1:
            raise ValueError("expected one pothole visual shape block")
    old_end = content.index(shape_end) + len(shape_end)
    new_end = generated.index(shape_end) + len(shape_end)
    content = (content[:content.index(shape_start)] +
               generated[generated.index(shape_start):new_end] +
               content[old_end:])
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

def _write_rows(path: Path, rows: list[dict]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


class FullRightSideController:
    """Keep the validated FR controller intact; hand over after recovery."""

    def __init__(self, *, scenario: dict, rr_gain_per_coupled_force: float,
                 identify_rr_swing: bool = False,
                 steer_probe_deg: float | None = None,
                 front_steering_feedback: bool = False,
                 contact_gains=None,
                 allocation=None,
                 path_reference=None,
                 rear_config=REAR_CYCLE_RUN):
        self.front = front_run.LiftCrawlController(crawl=True, scenario=scenario,
                                                   contact_gains=contact_gains)
        self.rear = RearCycleController(
            scenario=scenario,
            rr_gain_per_coupled_force=rr_gain_per_coupled_force,
            config=rear_config)
        self.rear_started = False
        self.identify_rr_swing = identify_rr_swing
        self.probe_start_s: float | None = None
        self.steer_probe_deg = steer_probe_deg
        self.front_steering_feedback = front_steering_feedback
        self.front_steer_deg = 0.0
        self.front_steer_tick_s: float | None = None
        self.allocation = allocation
        self.path_reference = path_reference

    def _allocate(self, now_s, x, command, stage):
        if self.allocation is None:
            return command
        selected = self.front if stage == 'FR' else self.rear
        output = self.allocation.apply(now_s, x, command, stage=stage, phase=selected.mode)
        if stage == 'RR':
            self.rear.last_suspension_force_n = np.asarray(output[4:8]).copy()
        if selected.rows and selected.rows[-1]['time_s'] == now_s:
            row = selected.rows[-1]
            row.update(self.allocation.telemetry)
            for i, c in enumerate(('fl', 'fr', 'rl', 'rr')):
                row[f'fact_{c}_n'] = float(output[4 + i])
        if (self.allocation.active and self.allocation.telemetry['support_qp_failure_s'] >=
                self.allocation.config.max_continuous_failure_s):
            selected.abort_reason = 'support allocation persistently infeasible'
            selected._enter('ABORT_STOP' if stage == 'FR' else 'RR_ABORT_STOP', now_s)
        return output

    def __call__(self, now_s: float, exports) -> tuple[float, ...]:
        if not self.rear_started:
            was_settling = self.front.mode == 'INIT_SETTLE'
            if self.path_reference is not None and was_settling:
                x = dict(zip(front_run.EXPORTS, exports))
                self.path_reference.update(now_s, x['Yo'], x['Yaw'])
            command = self.front(now_s, exports)
            if self.path_reference is not None and was_settling and self.front.mode != 'INIT_SETTLE':
                self.front.initial_yo_m, self.front.initial_yaw_deg = self.path_reference.freeze()
            if self.front_steering_feedback:
                cfg = self.rear.config
                if (self.front_steer_tick_s is None or
                        now_s - self.front_steer_tick_s >= cfg.control_period_s - 1e-8):
                    self.front_steer_tick_s = now_s
                    x = dict(zip(front_run.EXPORTS, exports))
                    desired = 0.0
                    if self.front.mode in ("CRAWL", "STOP"):
                        desired = float(np.clip(
                            -cfg.steer_lateral_deg_per_m * (x["Yo"] - (self.front.initial_yo_m or 0.))
                            -cfg.steer_yaw_deg_per_deg * (x["Yaw"] - (self.front.initial_yaw_deg or 0.)),
                            -cfg.steer_limit_deg, cfg.steer_limit_deg))
                    step = cfg.steer_slew_deg_s * cfg.control_period_s
                    self.front_steer_deg += float(np.clip(desired - self.front_steer_deg, -step, step))
                if self.front.rows and self.front.rows[-1]["time_s"] == now_s:
                    self.front.rows[-1]["steer_sw_deg"] = self.front_steer_deg
            if self.front.mode == "COMPLETE" and self.front.abort_reason is None:
                self.rear.initial_yo_m = self.front.initial_yo_m
                self.rear.initial_yaw_deg = self.front.initial_yaw_deg
                self.rear_started = True
            return self._allocate(now_s, dict(zip(front_run.EXPORTS, exports)),
                                  (*command, self.front_steer_deg), 'FR')
        x = dict(zip(front_run.EXPORTS, exports))
        if self.identify_rr_swing and self.rear.mode == "RR_HOLD":
            if self.probe_start_s is None:
                self.probe_start_s = now_s
            self.rear.mode_start_s = now_s - 1.0
        command = list(self.rear(now_s, x))
        if self.identify_rr_swing and self.probe_start_s is not None:
            elapsed = now_s - self.probe_start_s - 1.0
            pulse = int(elapsed // 3.0)
            if 0 <= pulse < 8:
                phase = elapsed - 3.0 * pulse
                window = min(1.0, max(0.0, phase / 0.5),
                             max(0.0, (2.0 - phase) / 0.5))
                delta = (250.0 if pulse % 2 == 0 else -250.0) * window
                corner = pulse // 2
                command[4 + corner] += delta
                if (self.rear.rows and
                        self.rear.rows[-1]["time_s"] == now_s):
                    self.rear.rows[-1]["probe_corner"] = corner
                    self.rear.rows[-1]["probe_delta_n"] = delta
                    name = ("fl", "fr", "rl", "rr")[corner]
                    self.rear.rows[-1][f"fact_{name}_n"] = command[4 + corner]
        steer_deg = self.rear.steer_deg
        if self.steer_probe_deg is not None:
            steer_deg = (self.steer_probe_deg if self.rear.mode in
                         ("RR_CRAWL", "RR_STOP") else 0.0)
        return self._allocate(now_s, x, (*command, steer_deg), 'RR')


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
        "four_wheel_recovered": end is not None and
        min(end[f"fz_{wheel}_n"] for wheel in ("fl", "fr", "rl", "rr")) >=
        cfg.support_floor_n,
    }
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


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--efficiency-profile", choices=("baseline", "balanced", "fast", "compact"),
                        default="baseline")
    parser.add_argument("--identify-rr-swing", action="store_true")
    parser.add_argument('--contact-gain-bundle', type=Path,
                        help='Optional native mode gains; exact selected model hash must match.')
    parser.add_argument('--support-allocation', choices=('off', 'monitor', 'active'), default='off')
    parser.add_argument('--support-allocation-gains', type=Path)
    parser.add_argument('--support-allocation-transitions', action='store_true')
    parser.add_argument('--average-path-reference', action='store_true')
    parser.add_argument("--steer-probe-deg", type=float)
    parser.add_argument("--stop-at", type=float)
    parser.add_argument("--original-kinematics", action="store_true",
                        help="diagnostic: preserve the source I_I ride tables")
    parser.add_argument("--lateral-ride-scale", type=float,
                        default=RIGHT_SIDE_MODEL.lateral_ride_scale,
                        help="copied suspension lateral movement scale, 0..1; default 0.5")
    parser.add_argument("--rear-target-speed-kph", type=float)
    parser.add_argument("--rear-min-pit-speed-kph", type=float)
    parser.add_argument("--rear-accel-ramp-s", type=float)
    parser.add_argument("--rear-preload-fl-force-n", type=float)
    parser.add_argument("--rear-preload-fr-force-n", type=float)
    parser.add_argument("--rear-preload-rl-force-n", type=float)
    parser.add_argument("--rear-adaptive-preload", action="store_true")
    parser.add_argument("--rear-sequential-preload", action="store_true")
    parser.add_argument("--rear-static-steering", action="store_true")
    parser.add_argument("--front-steering-feedback", action="store_true")
    parser.add_argument("--measurement-noise", action="store_true",
                        help="deterministic sensor noise; acceptance uses plant truth")
    parser.add_argument("--feedback-delay-s", type=float, default=0.,
                        help="sampled observation packet delay; requires --measurement-noise")
    parser.add_argument("--noise-seed", type=int, default=ObservationNoiseConfig().seed)
    parser.add_argument("--rear-parking-damping-nm-per-rpm", type=float)
    parser.add_argument("--rear-lift-force-n", type=float)
    parser.add_argument("--rear-posture-fl-force-n", type=float)
    parser.add_argument("--rear-control-profile", choices=("baseline", "robust"),
                        default="baseline")
    parser.add_argument("--rear-posture-hold-fl-force-n", type=float)
    parser.add_argument("--front-target-speed-kph", type=float)
    parser.add_argument("--front-min-pit-speed-kph", type=float)
    parser.add_argument("--front-accel-ramp-s", type=float)
    parser.add_argument("--front-lift-force-n", type=float)
    parser.add_argument("--front-support-feedback-limit-n", type=float)
    parser.add_argument("--front-brake-lead-m", type=float)
    parser.add_argument("--pit-width-m", type=float)
    parser.add_argument("--pit-depth-m", type=float)
    parser.add_argument("--road-friction", type=float)
    parser.add_argument("--vehicle-start-offset-m", type=float, default=0.0)
    args = parser.parse_args()
    if (args.support_allocation != 'off') != (args.support_allocation_gains is not None):
        parser.error('support allocation requires explicit gain bundle; off mode takes no bundle')
    if args.support_allocation_transitions and args.support_allocation == 'off':
        parser.error('transition allocation requires active or monitor support allocation')
    if not np.isfinite(args.lateral_ride_scale) or not 0. <= args.lateral_ride_scale <= 1.:
        parser.error("lateral ride scale must be finite and in 0..1")
    if (not np.isfinite(args.feedback_delay_s) or args.feedback_delay_s < 0. or
            (args.feedback_delay_s > 0. and not args.measurement_noise)):
        parser.error("feedback delay must be finite, nonnegative and requires --measurement-noise")
    if args.noise_seed < 0:
        parser.error("noise seed must be nonnegative")
    if ((args.rear_target_speed_kph is None) !=
            (args.rear_min_pit_speed_kph is None)):
        parser.error("rear target and minimum pit speed must be supplied together")
    if ((args.front_target_speed_kph is None) !=
            (args.front_min_pit_speed_kph is None)):
        parser.error("front target and minimum pit speed must be supplied together")
    front_run.CFG = {
        "baseline": CLOSED_LOOP_RUN,
        "balanced": BALANCED_CYCLE_FRONT,
        "fast": FAST_CYCLE_FRONT,
        "compact": COMPACT_CYCLE_FRONT,
    }[args.efficiency_profile]
    if args.front_target_speed_kph is not None:
        front_run.CFG = front_run.speed_trial_config(
            args.front_target_speed_kph, args.front_min_pit_speed_kph,
            args.front_accel_ramp_s, base=front_run.CFG)
    elif args.front_accel_ramp_s is not None:
        parser.error("front acceleration ramp requires a front speed trial")
    if args.front_lift_force_n is not None:
        if (not np.isfinite(args.front_lift_force_n) or
                not -front_run.CFG.max_lift_trial_force_n <=
                args.front_lift_force_n < 0.0):
            parser.error("front lift force must be negative and within the configured simulation trial range")
        front_run.CFG = replace(front_run.CFG,
                                fr_lift_force_n=args.front_lift_force_n)
    if args.front_support_feedback_limit_n is not None:
        if (not np.isfinite(args.front_support_feedback_limit_n) or
                not 0.0 < args.front_support_feedback_limit_n <=
                front_run.CFG.max_support_trial_feedback_n):
            parser.error("front support feedback limit exceeds the simulation trial range")
        front_run.CFG = replace(front_run.CFG,
                                support_feedback_limit_n=args.front_support_feedback_limit_n)
    if args.front_brake_lead_m is not None:
        if not np.isfinite(args.front_brake_lead_m) or args.front_brake_lead_m < 0.0:
            parser.error("front brake lead must be nonnegative and finite")
        front_run.CFG = replace(
            front_run.CFG,
            brake_start_before_far_edge_m=args.front_brake_lead_m)
    output = args.output.resolve()
    model, scenario = front_run._prepare_model(
        output, crawl=True, road_friction=args.road_friction)
    if (not np.isfinite(args.vehicle_start_offset_m) or
            abs(args.vehicle_start_offset_m) > RIGHT_SIDE_MODEL.max_start_offset_m):
        parser.error("vehicle start offset exceeds the configured trial range")
    if args.vehicle_start_offset_m:
        _set_vehicle_start_offset(model, scenario, args.vehicle_start_offset_m)
    if (args.front_brake_lead_m is not None and
            args.front_brake_lead_m >= float(scenario["length_m"])):
        parser.error("front brake lead must be shorter than pit length")
    if args.pit_width_m is not None or args.pit_depth_m is not None:
        _set_pit_geometry(model, scenario,
                          width_m=(args.pit_width_m if args.pit_width_m is not None
                                   else float(scenario["width_m"])),
                          depth_m=(args.pit_depth_m if args.pit_depth_m is not None
                                   else float(scenario["depth_m"])))
    if not args.original_kinematics and RIGHT_SIDE_MODEL.clamp_ride_tables:
        _clamp_kinematics_at_original_bounds(model)
    if not args.original_kinematics:
        _scale_lateral_ride_movement(model, args.lateral_ride_scale)
    _enable_steering_import(model)
    path = model / "run_all.par"
    content, count = re.subn(r"(?m)^TSTOP\s+[-+0-9.eE]+\s*$",
                             "TSTOP 115" if args.identify_rr_swing else "TSTOP 180",
                             path.read_text(encoding="utf-8"))
    if count != 2:
        raise ValueError("expected two TruckSim stop times")
    path.write_text(content, encoding="utf-8")
    gain = np.array(json.loads((front_run.EVIDENCE / "gain_matrix.json").read_text(
        encoding="utf-8"))["gains"]["Fz_n"], dtype=float)
    contact_gains, gain_provenance = None, None
    if args.contact_gain_bundle is not None:
        bundle_path = args.contact_gain_bundle.resolve()
        bundle = json.loads(bundle_path.read_text(encoding='utf-8'))
        model_hash = hashlib.sha256(path.read_bytes()).hexdigest()
        contact_gains = validated_contact_gains(bundle, model_sha256=model_hash)
        gain = contact_gains['FOUR_CONTACT']
        gain_provenance = {'bundle': str(bundle_path),
                           'bundle_sha256': hashlib.sha256(bundle_path.read_bytes()).hexdigest(),
                           'model_sha256': model_hash,
                           'scope': 'four-contact/FR local load feedback; RR swing gain reserved for allocation'}
    rear_config = (REAR_CYCLE_RUN if args.original_kinematics else
                   ROBUST_REAR_RUN if args.rear_control_profile == "robust" else
                   FAST_CYCLE_REAR if args.efficiency_profile == "fast" else
                   TUNED_REAR_RUN)
    if args.efficiency_profile == "compact":
        rear_config = replace(rear_config, return_ramp_s=COMPACT_REAR_RETURN_RAMP_S)
    if args.rear_adaptive_preload:
        rear_config = replace(rear_config, adaptive_preload=True,
                              preload_ready_timeout_s=rear_config.adaptive_preload_timeout_s)
    if args.rear_sequential_preload:
        rear_config = replace(rear_config, sequential_preload=True)
    if args.rear_static_steering:
        rear_config = replace(rear_config, static_steering_feedback=True)
    if args.rear_parking_damping_nm_per_rpm is not None:
        if (not np.isfinite(args.rear_parking_damping_nm_per_rpm) or
                not 0. <= args.rear_parking_damping_nm_per_rpm <=
                rear_config.parking_limit_nm):
            parser.error("invalid rear parking damping trial")
        rear_config = replace(rear_config,
                              parking_damping_nm_per_rpm=args.rear_parking_damping_nm_per_rpm)
    if args.rear_preload_rl_force_n is not None:
        if (not np.isfinite(args.rear_preload_rl_force_n) or
                not 0. <= args.rear_preload_rl_force_n <= rear_config.sim_force_limit_n):
            parser.error("rear RL preload must be nonnegative and within simulation force limits")
        rear_config = replace(rear_config, preload_rl_force_n=args.rear_preload_rl_force_n)
    if args.rear_accel_ramp_s is not None and args.rear_target_speed_kph is None:
        parser.error("rear acceleration ramp requires a rear speed trial")
    if args.rear_target_speed_kph is not None:
        rear_config = rear_speed_trial_config(
            args.rear_target_speed_kph, args.rear_min_pit_speed_kph,
            rear_config, args.rear_accel_ramp_s)
    for value, label, sign in (
            (args.rear_preload_fl_force_n, "FL", -1),
            (args.rear_preload_fr_force_n, "FR", 1)):
        if value is not None and (
                not np.isfinite(value) or sign * value <= 0.0 or
                abs(value) > rear_config.sim_force_limit_n):
            parser.error(f"rear preload {label} force has invalid sign or exceeds the simulation limit")
    rear_config = replace(
        rear_config,
        preload_fl_force_n=(rear_config.preload_fl_force_n if
                            args.rear_preload_fl_force_n is None else
                            args.rear_preload_fl_force_n),
        preload_fr_force_n=(rear_config.preload_fr_force_n if
                            args.rear_preload_fr_force_n is None else
                            args.rear_preload_fr_force_n))
    if args.rear_lift_force_n is not None:
        if (not np.isfinite(args.rear_lift_force_n) or
                not -rear_config.max_lift_trial_force_n <=
                args.rear_lift_force_n < 0.0):
            parser.error("rear lift force must be negative and within the configured trial range")
        rear_config = replace(rear_config, lift_force_n=args.rear_lift_force_n)
    posture_force = (rear_config.posture_fl_force_n if
                     args.rear_posture_fl_force_n is None else
                     args.rear_posture_fl_force_n)
    posture_hold = (rear_config.posture_hold_fl_force_n if
                    args.rear_posture_hold_fl_force_n is None else
                    args.rear_posture_hold_fl_force_n)
    if (not np.isfinite(posture_force) or not np.isfinite(posture_hold) or
            not -rear_config.max_posture_trial_force_n <= posture_force < 0.0 or
            not posture_force <= posture_hold < 0.0):
        parser.error("rear posture force must be negative and hold force no stronger than its ramp")
    rear_config = replace(rear_config, posture_fl_force_n=posture_force,
                          posture_hold_fl_force_n=posture_hold)
    allocation, allocation_provenance = None, None
    if args.support_allocation_gains is not None:
        allocation_path = args.support_allocation_gains.resolve()
        bundle = json.loads(allocation_path.read_text(encoding='utf-8'))
        model_hash = hashlib.sha256(path.read_bytes()).hexdigest()
        models = validated_support_models(bundle, model_sha256=model_hash)
        allocation = SupportAllocationFeedback(mode_models=models,
            active=args.support_allocation == 'active', period_s=front_run.CFG.control_period_s,
            front_limits=front_run.CFG, rear_limits=rear_config,
            transitions=args.support_allocation_transitions)
        allocation_provenance = {'bundle_sha256': hashlib.sha256(allocation_path.read_bytes()).hexdigest(),
                                 'model_sha256': model_hash, 'mode': args.support_allocation}
    controller = FullRightSideController(
        scenario=scenario,
        rr_gain_per_coupled_force=float(
            gain[3, 1] + REAR_CYCLE_RUN.preload_feedback_fl_per_fr * gain[3, 0]),
        identify_rr_swing=args.identify_rr_swing,
        steer_probe_deg=args.steer_probe_deg,
        front_steering_feedback=args.front_steering_feedback,
        contact_gains=contact_gains,
        allocation=allocation,
        path_reference=StaticPathReference() if args.average_path_reference else None,
        rear_config=rear_config)
    noise_config = ObservationNoiseConfig(seed=args.noise_seed,
                                         period_s=front_run.CFG.control_period_s,
                                         feedback_delay_s=args.feedback_delay_s)
    trial = (NoisyFeedbackTrial(controller, front_run.EXPORTS, front_run.CFG,
                               noise_config) if args.measurement_noise else None)
    native = run_stepwise(model / "simfile.sim", trial or controller,
                          output / "native_5ms.csv",
                          (*front_run.IMPORT_NAMES, STEERING_IMPORT),
                          front_run.EXPORTS, log_decimation=10,
                          stop_at_s=args.stop_at,
                          stop_when=(lambda now_s: (
                              (controller.front.mode == "COMPLETE" and
                               controller.front.abort_reason is not None and
                               now_s - controller.front.lower_start_s >=
                               front_run.CFG.lower_ramp_s + front_run.CFG.return_ramp_s +
                               rear_config.post_complete_observe_s) or (
                              controller.rear.mode == "RR_COMPLETE" and
                              controller.rear.mode_start_s is not None and
                              now_s - controller.rear.mode_start_s >=
                              rear_config.post_complete_observe_s))))
    front_rows = trial.front_truth.rows if trial else controller.front.rows
    rear_rows = trial.rear_truth.rows if trial else controller.rear.rows
    if trial:
        _write_rows(output / "front_observed_20ms.csv", controller.front.rows)
        _write_rows(output / "rear_observed_20ms.csv", controller.rear.rows)
    _write_rows(output / "front_control_20ms.csv", front_rows)
    _write_rows(output / "rear_control_20ms.csv", rear_rows)
    path_yo = trial.path_reference_yo_m if trial else controller.front.initial_yo_m
    path_yaw = trial.path_reference_yaw_deg if trial else controller.front.initial_yaw_deg
    front_result = front_run.evaluate_control_rows(
        front_rows, crawl=True, scenario=scenario,
        native_completed=native["status"] == "COMPLETED",
        abort_reason=controller.front.abort_reason,
        path_reference_yo_m=path_yo,
        path_reference_yaw_deg=path_yaw)
    result = evaluate_full_cycle(front_result, controller.rear, scenario,
                                 native["status"] == "COMPLETED",
                                 truth_rows=rear_rows,
                                 path_reference_yo_m=path_yo,
                                 path_reference_yaw_deg=path_yaw)
    result["fr_abort_reason"] = controller.front.abort_reason
    result["measurement_noise"] = {
        "enabled": args.measurement_noise, "config": asdict(noise_config),
        "acceptance_source": "plant_truth",
        "control_path_reference_yo_m": controller.front.initial_yo_m,
        "control_path_reference_yaw_deg": controller.front.initial_yaw_deg,
        "load_observation_floor_n": 0.0,
        "ideal_channels": "contact geometry, CoM, clearance, rates, travel, wheel speed",
        "delay_scope": "all exported observation channels; controller timestamp remains current",
        "delay_sampling": "control-period packets, causal hold, first packet during warmup",
        "maximum_logged_age_s": max((r.get("measurement_age_s", 0.)
                                       for r in front_rows + rear_rows), default=0.),
    }
    result["model_tuning"] = {
        "original_kinematics": args.original_kinematics,
        "clamp_ride_tables": (not args.original_kinematics and
                              RIGHT_SIDE_MODEL.clamp_ride_tables),
        "lateral_ride_scale": (1.0 if args.original_kinematics else
                               args.lateral_ride_scale),
        "steering_import": STEERING_IMPORT,
    }
    result["efficiency_profile"] = args.efficiency_profile
    result["rear_control_profile"] = args.rear_control_profile
    result["front_steering_feedback"] = args.front_steering_feedback
    result['path_reference_estimation'] = {
        'averaged': args.average_path_reference,
        'config': asdict(PATH_REFERENCE),
        'sample_count': controller.path_reference.sample_count if controller.path_reference else 1,
        'scope': 'INIT_SETTLE trailing window; freeze on existing settle-to-preload transition; shared FR/RR reference',
    }
    result["controller_config"] = {
        "front": asdict(front_run.CFG), "rear": asdict(rear_config),
    }
    result['identified_gain_provenance'] = gain_provenance
    if allocation is not None:
        result['support_allocation'] = {**allocation.summary(), 'config': asdict(allocation.config),
                                         'provenance': allocation_provenance}
    result["front_timing_config"] = {
        "init_settle_s": front_run.CFG.init_settle_s,
        "preload_max_reference_rate": front_run.CFG.preload_max_reference_rate,
        "lift_ramp_s": front_run.CFG.lift_ramp_s,
        "hold_s": front_run.CFG.three_wheel_hold_s,
        "lower_ramp_s": front_run.CFG.lower_ramp_s,
        "return_ramp_s": front_run.CFG.return_ramp_s,
    }
    result["scenario"] = {key: scenario[key] for key in
                          ("start_station_m", "length_m", "width_m", "depth_m",
                           "friction")}
    result["scenario"]["vehicle_start_offset_m"] = args.vehicle_start_offset_m
    result["scenario"]["approach_distance_m"] = scenario["approach_distance_m"]
    result["rear_speed_config"] = {
        "target_kph": rear_config.crawl_speed_kph,
        "minimum_pit_kph": rear_config.min_pit_speed_kph,
        "accel_ramp_s": rear_config.crawl_accel_ramp_s,
        "preload_fl_force_n": rear_config.preload_fl_force_n,
        "preload_fr_force_n": rear_config.preload_fr_force_n,
        "preload_rl_force_n": rear_config.preload_rl_force_n,
        "lift_force_n": rear_config.lift_force_n,
        "posture_fl_force_n": rear_config.posture_fl_force_n,
        "posture_hold_fl_force_n": rear_config.posture_hold_fl_force_n,
    }
    result["front_speed_config"] = {
        "target_kph": front_run.CFG.crawl_speed_kph,
        "minimum_pit_kph": front_run.CFG.crawl_min_speed_kph,
        "accel_ramp_s": front_run.CFG.crawl_accel_ramp_s,
        "lift_force_n": front_run.CFG.fr_lift_force_n,
        "support_feedback_limit_n": front_run.CFG.support_feedback_limit_n,
        "brake_lead_m": front_run.CFG.brake_start_before_far_edge_m,
    }
    result["front"] = front_result
    result["native"] = native
    (output / "result.json").write_text(json.dumps(result, indent=2),
                                        encoding="utf-8")
    print(json.dumps({k: v for k, v in result.items() if k != "native"},
                     indent=2))


if __name__ == "__main__":
    main()
