"""Native TruckSim FR crossing followed by RR crossing and four-wheel return."""

from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path

import numpy as np

import run_static_fr_closed_loop as front_run

from ddevsim.cosim import run_stepwise
from ddevsim.static_wheel_lift.config import (REAR_CYCLE_RUN,
                                             RIGHT_SIDE_MODEL, TUNED_REAR_RUN)
from ddevsim.static_wheel_lift.rear_cycle import RearCycleController

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

def _write_rows(path: Path, rows: list[dict]) -> None:
    if not rows:
        raise ValueError("native controller produced no samples")
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
                 rear_config=REAR_CYCLE_RUN):
        self.front = front_run.LiftCrawlController(crawl=True, scenario=scenario)
        self.rear = RearCycleController(
            scenario=scenario,
            rr_gain_per_coupled_force=rr_gain_per_coupled_force,
            config=rear_config)
        self.rear_started = False
        self.identify_rr_swing = identify_rr_swing
        self.probe_start_s: float | None = None
        self.steer_probe_deg = steer_probe_deg

    def __call__(self, now_s: float, exports) -> tuple[float, ...]:
        if not self.rear_started:
            command = self.front(now_s, exports)
            if self.front.mode == "COMPLETE":
                self.rear_started = True
            return (*command, 0.0)
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
        return (*command, steer_deg)


def evaluate_full_cycle(front_result: dict, rear: RearCycleController,
                        scenario: dict, native_completed: bool) -> dict:
    rows = rear.rows
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
    max_lateral = (max(abs(row["yo_m"] - rows[0]["yo_m"]) for row in moving)
                   if moving else None)
    max_yaw = (max(abs(row["yaw_deg"] - rows[0]["yaw_deg"]) for row in moving)
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
    parser.add_argument("--identify-rr-swing", action="store_true")
    parser.add_argument("--steer-probe-deg", type=float)
    parser.add_argument("--stop-at", type=float)
    parser.add_argument("--original-kinematics", action="store_true",
                        help="diagnostic: preserve the source I_I ride tables")
    args = parser.parse_args()
    output = args.output.resolve()
    model, scenario = front_run._prepare_model(output, crawl=True)
    if not args.original_kinematics and RIGHT_SIDE_MODEL.clamp_ride_tables:
        _clamp_kinematics_at_original_bounds(model)
    if not args.original_kinematics:
        _scale_lateral_ride_movement(model, RIGHT_SIDE_MODEL.lateral_ride_scale)
    _enable_steering_import(model)
    path = model / "run_all.par"
    content, count = re.subn(r"(?m)^TSTOP 125\s*$",
                             "TSTOP 115" if args.identify_rr_swing else "TSTOP 180",
                             path.read_text(encoding="utf-8"))
    if count != 2:
        raise ValueError("expected two TruckSim stop times")
    path.write_text(content, encoding="utf-8")
    gain = np.array(json.loads((front_run.EVIDENCE / "gain_matrix.json").read_text(
        encoding="utf-8"))["gains"]["Fz_n"], dtype=float)
    controller = FullRightSideController(
        scenario=scenario,
        rr_gain_per_coupled_force=float(
            gain[3, 1] + REAR_CYCLE_RUN.preload_feedback_fl_per_fr * gain[3, 0]),
        identify_rr_swing=args.identify_rr_swing,
        steer_probe_deg=args.steer_probe_deg,
        rear_config=(REAR_CYCLE_RUN if args.original_kinematics
                     else TUNED_REAR_RUN))
    native = run_stepwise(model / "simfile.sim", controller,
                          output / "native_5ms.csv",
                          (*front_run.IMPORT_NAMES, STEERING_IMPORT),
                          front_run.EXPORTS, log_decimation=10,
                          stop_at_s=args.stop_at)
    _write_rows(output / "front_control_20ms.csv", controller.front.rows)
    _write_rows(output / "rear_control_20ms.csv", controller.rear.rows)
    front_result = front_run.evaluate_control_rows(
        controller.front.rows, crawl=True, scenario=scenario,
        native_completed=native["status"] == "COMPLETED",
        abort_reason=controller.front.abort_reason)
    result = evaluate_full_cycle(front_result, controller.rear, scenario,
                                 native["reached_configured_stop"])
    result["model_tuning"] = {
        "original_kinematics": args.original_kinematics,
        "clamp_ride_tables": (not args.original_kinematics and
                              RIGHT_SIDE_MODEL.clamp_ride_tables),
        "lateral_ride_scale": (1.0 if args.original_kinematics else
                               RIGHT_SIDE_MODEL.lateral_ride_scale),
        "steering_import": STEERING_IMPORT,
    }
    result["front"] = front_result
    result["native"] = native
    (output / "result.json").write_text(json.dumps(result, indent=2),
                                        encoding="utf-8")
    print(json.dumps({k: v for k, v in result.items() if k != "native"},
                     indent=2))


if __name__ == "__main__":
    main()
