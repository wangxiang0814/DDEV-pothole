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

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from ddevsim.cosim import run_stepwise
from ddevsim.contact_geometry import GEOMETRY_EXPORTS
from ddevsim.interface_validation import EXPORT_NAMES, IMPORT_NAMES
from ddevsim.pothole_case import SCENARIO_EXPORTS
from ddevsim.static_wheel_lift.closed_loop import (
    CrawlTorqueFeedback, PreloadProgress, ScalarLoadFeedback,
    SupportForceFeedback, blend_contact_gain, front_clearance_guard_m,
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


def _reference(path: Path) -> tuple[np.ndarray, np.ndarray]:
    with gzip.open(path, "rt", encoding="utf-8", newline="") as source:
        rows = list(csv.DictReader(source))
    time = np.array([float(row["time_s"]) for row in rows])
    fz = np.array([[float(row["exp_Fz_" + wheel]) for wheel in WHEELS]
                   for row in rows])
    if len(time) < 2 or np.any(np.diff(time) <= 0) or not np.isfinite(fz).all():
        raise ValueError("invalid native lift reference")
    return time, fz


def _barycentric(triangle: np.ndarray, point: np.ndarray) -> np.ndarray:
    frame = np.vstack((triangle.T, np.ones(3)))
    return np.linalg.solve(frame, np.r_[point, 1.])


def _prepare_model(output: Path, *, crawl: bool,
                   road_friction: float | None = None) -> tuple[Path, dict]:
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
                                       f"{key} {CFG.tyre_low_speed_slip_m_s:g}",
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
        scenario["target_speed_kph"] = CFG.crawl_speed_kph
    scenario["controller_period_s"] = CFG.control_period_s
    content, count = re.subn(r"(?m)^TSTOP\s+[-+0-9.eE]+\s*$",
                             "TSTOP 125" if crawl else "TSTOP 82", content)
    if count != 2:
        raise ValueError("unexpected TruckSim stop-time declarations")
    (model / "run_all.par").write_text(content, encoding="utf-8")
    (model / "scenario.json").write_text(
        json.dumps(scenario, indent=2) + "\n", encoding="utf-8")
    return model, scenario


class LiftCrawlController:
    def __init__(self, *, crawl: bool, scenario: dict, contact_gains=None):
        model_hash = hashlib.sha256((EVIDENCE / "model" / "run_all.par").read_bytes()).hexdigest()
        gain = json.loads((EVIDENCE / "swing_support_gain.json").read_text(encoding="utf-8"))
        if gain["model_sha256"] != model_hash:
            raise ValueError("identified swing gain does not match I_I model")
        self.trace = load_command_trace(EVIDENCE / "preload_trace.csv.gz", IMPORT_NAMES)
        self.ref_time, self.ref_fz = _reference(EVIDENCE / "lift_reference.csv.gz")
        self.preload_clock = PreloadProgress(
            start_s=CFG.preload_feedback_start_s,
            end_s=float(self.trace.time_s[-1]),
            max_rate=CFG.preload_max_reference_rate,
            slow_error_n=CFG.preload_slow_tracking_error_n,
            pause_error_n=CFG.preload_pause_tracking_error_n)
        self.swing_gain = np.asarray(gain["gain_fz_per_command"], dtype=float)
        static_gain = np.asarray(json.loads((EVIDENCE / "gain_matrix.json").read_text(
            encoding="utf-8"))["gains"]["Fz_n"], dtype=float)
        if contact_gains is not None:
            static_gain = contact_gains['FOUR_CONTACT']
            self.swing_gain = contact_gains['FR'][np.ix_(SUPPORT, SUPPORT)]
        self.fr_feedback = ScalarLoadFeedback(
            gain_load_per_force=float(static_gain[1, 1]),
            control_period_s=CFG.control_period_s,
            correction_limit_n=CFG.fr_feedback_limit_n,
            slew_n_s=CFG.fr_feedback_slew_n_s,
            tracking_gain=CFG.fr_feedback_tracking_gain,
            deadband_n=CFG.fr_feedback_deadband_n)
        self.static_support_gain = static_gain[np.ix_(SUPPORT, SUPPORT)]
        self.support = SupportForceFeedback(self.static_support_gain,
                                            control_period_s=CFG.control_period_s,
                                            correction_limit_n=CFG.support_feedback_limit_n,
                                            rebound_guard_mm=(CFG.moving_rebound_guard_mm
                                                              if crawl else CFG.rebound_guard_mm))
        self.load_filter = WheelLoadFilter(
            cutoff_hz=CFG.wheel_load_filter_cutoff_hz, wheel_count=4)
        self.drive = CrawlTorqueFeedback(control_period_s=CFG.control_period_s,
                                         tyre_radius_m=CFG.tyre_radius_m,
                                         half_track_m=CFG.tyre_half_track_m,
                                         friction=float(scenario["friction"]))
        self.crawl = crawl
        self.scenario = scenario
        self.mode = "INIT_SETTLE"
        self.mode_start_s = 0.
        self.ready_since_s: float | None = None
        self.preload_end_since_s: float | None = None
        self.stop_since_s: float | None = None
        self.stop_enter_speed_kph = 0.
        self.lift_start_s: float | None = None
        self.lower_start_s: float | None = None
        self.hold_start_s: float | None = None
        self.crawl_start_s: float | None = None
        self.abort_reason: str | None = None
        self.last_tick_s: float | None = None
        self.fr_correction_n = 0.
        self.swing_gain_weight = 0.
        self.torques_nm = np.zeros(3)
        self.lower_start_torques_nm = np.zeros(3)
        self.initial_yo_m: float | None = None
        self.initial_yaw_deg: float | None = None
        self.rows: list[dict] = []

    def _enter(self, mode: str, now_s: float) -> None:
        self.mode = mode
        self.mode_start_s = now_s
        if mode == "LIFTING":
            self.lift_start_s = now_s
            self.support.gain = self.swing_gain
        elif mode == "THREE_WHEEL_HOLD":
            self.hold_start_s = now_s
        elif mode == "CRAWL":
            self.crawl_start_s = now_s
        elif mode == "LOWERING":
            self.lower_start_s = now_s
            self.lower_start_torques_nm = self.torques_nm.copy()

    def _ref_loads(self, now_s: float) -> np.ndarray:
        if self.lift_start_s is None:
            reference_time = self.preload_clock.progress_s
        else:
            reference_time = self.trace.time_s[-1] + now_s - self.lift_start_s
            if self.crawl:
                reference_time = min(reference_time, CFG.reference_hold_s)
        return np.array([np.interp(reference_time, self.ref_time, self.ref_fz[:, i])
                         for i in range(4)])

    def __call__(self, now_s: float, exports) -> tuple[float, ...]:
        x = dict(zip(EXPORTS, exports))
        if self.last_tick_s is None or now_s - self.last_tick_s >= CFG.control_period_s - 1e-8:
            self.last_tick_s = now_s
            self._tick(now_s, x)
        base = np.asarray(self.trace.at(self.preload_clock.progress_s), dtype=float)
        # The validated preload trace ends at 48 s and holds its final command.
        extra_fr = 0.
        extra_rl = 0.
        if self.lift_start_s is not None:
            lift_fraction = quintic_step(now_s, self.lift_start_s, CFG.lift_ramp_s)[0]
            extra_fr = CFG.fr_lift_force_n * lift_fraction
            extra_rl = CFG.rl_support_force_n * quintic_step(
                now_s, self.lift_start_s + CFG.lift_ramp_s,
                CFG.support_ramp_s)[0]
        if self.lower_start_s is not None:
            lower_fraction = quintic_step(now_s, self.lower_start_s,
                                          CFG.lower_ramp_s)[0]
            extra_fr *= 1. - lower_fraction
            extra_rl *= 1. - lower_fraction
            return_scale = 1. - quintic_step(now_s, self.lower_start_s +
                                             CFG.lower_ramp_s,
                                             CFG.return_ramp_s)[0]
            base *= return_scale
        else:
            return_scale = 1.
        base[4:8] += return_scale * np.array([
                               self.support.correction[0], self.fr_correction_n,
                               extra_rl + self.support.correction[1],
                               self.support.correction[2]])
        base[5] += return_scale * extra_fr
        if self.mode == "LOWERING":
            torque_fraction = 1. - quintic_step(
                now_s, self.lower_start_s, CFG.torque_release_s)[0]
            base[[0, 2, 3]] = self.lower_start_torques_nm * torque_fraction
        else:
            base[[0, 2, 3]] = self.torques_nm
        base[1] = 0.  # FR is never driven during lift or crawl.
        if self.rows and self.rows[-1]["time_s"] == now_s:
            self.rows[-1].update({
                "fact_fl_n": float(base[4]), "fact_fr_n": float(base[5]),
                "fact_rl_n": float(base[6]), "fact_rr_n": float(base[7]),
            })
        return tuple(float(v) for v in base)

    def _tick(self, now_s: float, x: dict[str, float]) -> None:
        fz = np.array([max(0., x["Fz_" + wheel]) for wheel in WHEELS])
        gate_fz = self.load_filter.update(now_s, fz)
        if self.mode in ("INIT_SETTLE", "PRELOAD_SHIFT"):
            self.support.gain = blend_contact_gain(
                self.static_support_gain, self.swing_gain, float(gate_fz[1]),
                swing_n=CFG.gain_blend_swing_fr_load_n,
                stance_n=CFG.gain_blend_stance_fr_load_n)
            self.swing_gain_weight = float(np.clip(
                (CFG.gain_blend_stance_fr_load_n - gate_fz[1]) /
                (CFG.gain_blend_stance_fr_load_n -
                 CFG.gain_blend_swing_fr_load_n), 0., 1.))
        if self.mode in ("INIT_SETTLE", "PRELOAD_SHIFT"):
            tracking_error = float(np.max(np.abs(
                gate_fz - self._ref_loads(now_s))))
            self.preload_clock.update(now_s, load_error_n=tracking_error)
        travel = np.array([x["Jnc_" + wheel] for wheel in WHEELS])
        contacts = {corner: (x["Xctc_" + wheel + "i"],
                             x["Yctc_" + wheel + "i"])
                    for corner, wheel in zip(("FL", "FR", "RL", "RR"), WHEELS)}
        com = np.array([x["XCG_TM"], x["YCG_TM"]])
        assessment = assess_support(contacts, dict(zip(("FL", "FR", "RL", "RR"), gate_fz)),
                                    tuple(com), CFG.lambda_target)
        triangle = np.array([contacts[c] for c in ("FL", "RL", "RR")])
        com_lambda = _barycentric(triangle, com)
        margin = min(assessment.lambda_min, float(np.min(com_lambda)))
        top_clearance = x["Z_R1"] - CFG.tyre_radius_m
        attitude = max(abs(x["Roll_E"]), abs(x["Pitch"]))
        support_load = float(min(gate_fz[SUPPORT]))
        if self.initial_yo_m is None and now_s >= CFG.init_settle_s:
            self.initial_yo_m = x["Yo"]
            self.initial_yaw_deg = x["Yaw"]
        ready = (gate_fz[1] <= CFG.fr_ready_load_n and support_load >=
                 CFG.support_floor_n and margin >= CFG.lambda_target and
                 attitude <= CFG.ready_attitude_limit_deg and
                 abs(x["Vx"]) <= CFG.init_max_vx_kph)
        if ready:
            if self.ready_since_s is None:
                self.ready_since_s = now_s
        else:
            self.ready_since_s = None
        if self.mode == "INIT_SETTLE" and now_s >= CFG.init_settle_s:
            if (np.min(gate_fz) > CFG.support_floor_n and
                    abs(x["Vx"]) <= CFG.init_max_vx_kph):
                if self.preload_clock.progress_s < CFG.preload_feedback_start_s:
                    self.preload_clock.align_reference(
                        now_s, CFG.preload_feedback_start_s)
                self._enter("PRELOAD_SHIFT", now_s)
        if (self.mode == "PRELOAD_SHIFT" and
                self.preload_clock.progress_s >= self.trace.time_s[-1]):
            if self.preload_end_since_s is None:
                self.preload_end_since_s = now_s
            if (self.ready_since_s is not None and
                    now_s - self.ready_since_s >= CFG.ready_dwell_s):
                self._enter("LIFTING", now_s)
            elif now_s - self.preload_end_since_s >= CFG.preload_end_wait_limit_s:
                self.abort_reason = "preload ready gate not reached"
                self._enter("LOWERING", now_s)
        if (self.mode == "PRELOAD_SHIFT" and
                now_s - self.mode_start_s >= CFG.preload_total_limit_s):
            self.abort_reason = "preload tracking deadline exceeded"
            self._enter("LOWERING", now_s)
        if (self.mode == "LIFTING" and now_s - self.lift_start_s >=
                CFG.lift_ramp_s + CFG.support_ramp_s and
                top_clearance >= CFG.lift_clearance_m and
                gate_fz[1] <= CFG.fr_swing_load_n):
            self._enter("THREE_WHEEL_HOLD", now_s)
        if (self.mode == "THREE_WHEEL_HOLD" and
                now_s - self.hold_start_s >= CFG.three_wheel_hold_s):
            self._enter("CRAWL" if self.crawl else "LOWERING", now_s)
        if self.mode == "CRAWL":
            if x["X_R1"] >= (float(self.scenario["start_station_m"]) +
                               float(self.scenario["length_m"]) -
                               CFG.brake_start_before_far_edge_m):
                self.stop_enter_speed_kph = max(0., x["Vx"])
                self._enter("STOP", now_s)
            elif now_s - self.crawl_start_s > CFG.max_crawl_s:
                self.abort_reason = "crawl did not cross before timeout"
                self._enter("ABORT_STOP", now_s)
        if self.mode == "STOP":
            fr_clear = x["X_R1"] >= (float(self.scenario["start_station_m"]) +
                                      float(self.scenario["length_m"]) +
                                      CFG.crossing_clearance_m)
            rear_clear = x["X_R2"] <= (float(self.scenario["start_station_m"]) -
                                       CFG.rear_stop_clearance_m)
            if fr_clear and rear_clear and abs(x["Vx"]) <= CFG.stop_speed_kph:
                if self.stop_since_s is None:
                    self.stop_since_s = now_s
                elif now_s - self.stop_since_s >= CFG.stop_dwell_s:
                    self._enter("LOWERING", now_s)
            else:
                self.stop_since_s = None
        if (self.mode == "LOWERING" and now_s - self.lower_start_s >=
                CFG.lower_ramp_s and fz[1] > CFG.support_floor_n):
            self._enter("RETURN_TO_FOUR_WHEEL", now_s)
        if (self.mode == "RETURN_TO_FOUR_WHEEL" and
                now_s - self.lower_start_s >= CFG.lower_ramp_s +
                CFG.return_ramp_s and np.min(fz) > CFG.support_floor_n):
            self._enter("COMPLETE", now_s)
        if self.mode in ("LIFTING", "THREE_WHEEL_HOLD", "CRAWL", "STOP"):
            if (support_load < CFG.support_floor_n or margin < CFG.lambda_abort or
                    attitude > CFG.attitude_limit_deg or
                    min(travel) < (CFG.moving_rebound_abort_mm if self.crawl
                                   else CFG.rebound_abort_mm) or
                    max(travel) > CFG.jounce_abort_mm or
                    (self.mode in ("CRAWL", "STOP") and
                     top_clearance < front_clearance_guard_m(
                         x["X_R1"],
                         float(self.scenario["start_station_m"]) +
                         float(self.scenario["length_m"]),
                         CFG.crossing_clearance_m, CFG.lip_clearance_m,
                         CFG.post_pit_positive_clearance_m))):
                self.abort_reason = "three-wheel support or clearance limit"
                self._enter("ABORT_STOP", now_s)
        ref = self._ref_loads(now_s)
        if self.mode in ("PRELOAD_SHIFT", "LIFTING", "THREE_WHEEL_HOLD",
                         "CRAWL", "STOP"):
            if (self.mode in ("CRAWL", "STOP") and
                    (assessment.lambda_min < CFG.lambda_target or
                     min(com_lambda) < CFG.lambda_target)):
                target_lambda = np.maximum(com_lambda, CFG.lambda_target)
                target_lambda /= target_lambda.sum()
                ref[SUPPORT] = fz.sum() * target_lambda
            self.support.update(now_s, fz[SUPPORT], ref[SUPPORT],
                                travel_mm=travel[SUPPORT])
        if self.mode == "PRELOAD_SHIFT":
            self.fr_correction_n = self.fr_feedback.update(
                now_s, measured_n=float(gate_fz[1]),
                reference_n=float(ref[1]))
        if self.mode in ("CRAWL", "STOP", "ABORT_STOP"):
            if self.mode == "CRAWL":
                target_speed = (CFG.crawl_speed_kph *
                                quintic_step(now_s, self.crawl_start_s,
                                             CFG.crawl_accel_ramp_s)[0]
                                if CFG.crawl_accel_ramp_s > 0.
                                else CFG.crawl_speed_kph)
            elif self.mode == "STOP":
                target_speed = self.stop_enter_speed_kph * (
                    1. - quintic_step(now_s, self.mode_start_s,
                                       CFG.stop_ramp_s)[0])
            else:
                target_speed = 0.
            self.torques_nm = self.drive.update(
                now_s, vx_kph=x["Vx"],
                yaw_deg=x["Yaw"] - (self.initial_yaw_deg or 0.),
                yaw_rate_deg_s=x["AVz"],
                lateral_m=x["Yo"] - (self.initial_yo_m or 0.),
                loads_n=fz[SUPPORT], target_kph=target_speed)
            if self.mode in ("STOP", "ABORT_STOP"):
                self.torques_nm = np.maximum(
                    self.torques_nm, -CFG.stop_max_brake_nm)
                self.drive.torque_nm = self.torques_nm.copy()
        else:
            self.torques_nm[:] = 0.
            target_speed = 0.
        if self.lower_start_s is None:
            applied_scale = 1.
        else:
            applied_scale = 1. - quintic_step(
                now_s, self.lower_start_s + CFG.lower_ramp_s,
                CFG.return_ramp_s)[0]
        if self.mode == "LOWERING":
            applied_torque = self.lower_start_torques_nm * (
                1. - quintic_step(now_s, self.lower_start_s,
                                   CFG.torque_release_s)[0])
        else:
            applied_torque = self.torques_nm
        self.rows.append({
            "time_s": now_s, "mode": self.mode, "vx_kph": x["Vx"],
            "speed_target_kph": target_speed,
            "preload_ref_time_s": self.preload_clock.progress_s,
            "preload_ref_rate": self.preload_clock.rate,
            "support_swing_gain_weight": self.swing_gain_weight,
            "x_fr_m": x["X_R1"], "x_rr_m": x["X_R2"],
            "yo_m": x["Yo"], "yaw_deg": x["Yaw"],
            "roll_deg": x["Roll_E"], "pitch_deg": x["Pitch"],
            "travel_fl_mm": travel[0], "travel_fr_mm": travel[1],
            "travel_rl_mm": travel[2], "travel_rr_mm": travel[3],
            "fr_top_clearance_m": top_clearance,
            "fz_fl_n": fz[0], "fz_fr_n": fz[1], "fz_rl_n": fz[2],
            "fz_rr_n": fz[3], "ref_fl_n": ref[0], "ref_fr_n": ref[1],
            "ref_rl_n": ref[2], "ref_rr_n": ref[3],
            "zmp_lambda_min": assessment.lambda_min,
            "zmp_x_m": assessment.zmp_xy[0],
            "zmp_y_m": assessment.zmp_xy[1],
            "zmp_edge_distance_m": assessment.edge_distance_m,
            "com_x_m": com[0], "com_y_m": com[1],
            "com_lambda_min": float(np.min(com_lambda)),
            "support_feedback_saturated": bool(np.any(
                np.abs(self.support.correction) >=
                CFG.support_feedback_limit_n - 1e-8)),
            "force_corr_fl_n": applied_scale * self.support.correction[0],
            "force_corr_fr_n": applied_scale * self.fr_correction_n,
            "force_corr_rl_n": applied_scale * self.support.correction[1],
            "force_corr_rr_n": applied_scale * self.support.correction[2],
            "fz_fl_filtered_n": gate_fz[0],
            "fz_fr_filtered_n": gate_fz[1],
            "fz_rl_filtered_n": gate_fz[2],
            "fz_rr_filtered_n": gate_fz[3],
            "torque_fl_nm": applied_torque[0],
            "torque_rl_nm": applied_torque[1],
            "torque_rr_nm": applied_torque[2],
        })


def evaluate_control_rows(rows: list[dict], *, crawl: bool, scenario: dict,
                          native_completed: bool,
                          abort_reason: str | None,
                          path_reference_yo_m: float | None = None,
                          path_reference_yaw_deg: float | None = None) -> dict:
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
                    instant - last_saturation_time > 1.5 * CFG.control_period_s):
                saturation_start = instant
            last_saturation_time = instant
            longest_saturation_s = max(
                longest_saturation_s,
                instant - saturation_start + CFG.control_period_s)
        else:
            saturation_start = None
            last_saturation_time = None
    criteria = {
        "native_completed": native_completed,
        "no_abort": abort_reason is None,
        "fsm_complete": end["mode"] == "COMPLETE",
        "hold_at_least_5_s": hold_duration >= CFG.three_wheel_hold_s - 1e-6,
        "hold_fr_clearance": bool(hold) and min_value(hold, "fr_top_clearance_m") >= CFG.lift_clearance_m,
        "hold_fr_unloaded": bool(hold) and max_value(hold, "fz_fr_n") <= CFG.fr_swing_load_n,
        "support_wheels_loaded": min_support is not None and min_support >= CFG.support_floor_n,
        "support_triangle_zmp": bool(hold) and min_value(hold + moving, "zmp_lambda_min") >= CFG.lambda_target,
        "support_triangle_com": bool(hold) and min_value(hold + moving, "com_lambda_min") >= CFG.lambda_target,
        "four_wheel_recovered": all(float(end[k]) >= CFG.support_floor_n
                                    for k in ("fz_fl_n", "fz_fr_n", "fz_rl_n", "fz_rr_n")),
        "no_sustained_support_saturation":
            longest_saturation_s <= CFG.max_continuous_support_saturation_s,
    }
    if crawl:
        criteria.update({
            "pit_crossed": float(end["x_fr_m"]) >= far_edge + CFG.crossing_clearance_m,
            "pit_speed_in_requested_range": bool(pit) and
            min_value(pit, "vx_kph") >= CFG.crawl_min_speed_kph and
            max_value(pit, "vx_kph") <= CFG.crawl_max_speed_kph,
            "lifted_fr_over_pit": bool(pit) and
            min_value(pit, "fr_top_clearance_m") >= CFG.lift_clearance_m and
            max_value(pit, "fz_fr_n") <= CFG.fr_swing_load_n,
            "straight_path": lateral_max is not None and
            lateral_max <= CFG.crawl_max_lateral_error_m and
            yaw_max <= CFG.crawl_max_yaw_error_deg,
            "stopped_before_lowering": bool(lowering) and
            abs(float(lowering[0]["vx_kph"])) <= CFG.stop_speed_kph,
            "rear_stopped_before_pit": bool(lowering) and
            float(lowering[0]["x_rr_m"]) <=
            edge - CFG.rear_stop_clearance_m,
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


def main() -> None:
    global CFG
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--crawl", action="store_true")
    parser.add_argument("--road-friction", type=float,
                        help="physical road mu in an isolated crawl copy")
    parser.add_argument("--target-speed-kph", type=float)
    parser.add_argument("--min-pit-speed-kph", type=float)
    args = parser.parse_args()
    if (args.target_speed_kph is None) != (args.min_pit_speed_kph is None):
        parser.error("target and minimum pit speed must be supplied together")
    if args.target_speed_kph is not None:
        CFG = speed_trial_config(args.target_speed_kph, args.min_pit_speed_kph)
    output = args.output.resolve()
    model, scenario = _prepare_model(output, crawl=args.crawl,
                                     road_friction=args.road_friction)
    controller = LiftCrawlController(crawl=args.crawl, scenario=scenario)
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
        abort_reason=controller.abort_reason))
    (output / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps({k: v for k, v in result.items() if k != "native"}, indent=2))


if __name__ == "__main__":
    main()
