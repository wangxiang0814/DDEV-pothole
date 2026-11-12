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

# Compatibility collaborators shared by legacy full-cycle diagnostics.
from .model_builder import prepare_model as _prepare_model
from .acceptance import evaluate_control_rows


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


class LiftCrawlController:
    def __init__(self, *, crawl: bool, scenario: dict, contact_gains=None, config=CFG, evidence_dir=None):
        self.config = config
        evidence = Path(evidence_dir) if evidence_dir is not None else EVIDENCE
        model_hash = hashlib.sha256((evidence / "model" / "run_all.par").read_bytes()).hexdigest()
        gain = json.loads((evidence / "swing_support_gain.json").read_text(encoding="utf-8"))
        if gain["model_sha256"] != model_hash:
            raise ValueError("identified swing gain does not match I_I model")
        self.trace = load_command_trace(evidence / "preload_trace.csv.gz", IMPORT_NAMES)
        self.ref_time, self.ref_fz = _reference(evidence / "lift_reference.csv.gz")
        self.preload_clock = PreloadProgress(
            start_s=self.config.preload_feedback_start_s,
            end_s=float(self.trace.time_s[-1]),
            max_rate=self.config.preload_max_reference_rate,
            slow_error_n=self.config.preload_slow_tracking_error_n,
            pause_error_n=self.config.preload_pause_tracking_error_n)
        self.preload_initial_bias_n = None
        self.swing_gain = np.asarray(gain["gain_fz_per_command"], dtype=float)
        static_gain = np.asarray(json.loads((evidence / "gain_matrix.json").read_text(
            encoding="utf-8"))["gains"]["Fz_n"], dtype=float)
        if contact_gains is not None:
            static_gain = contact_gains['FOUR_CONTACT']
            self.swing_gain = contact_gains['FR'][np.ix_(SUPPORT, SUPPORT)]
        self.fr_feedback = ScalarLoadFeedback(
            gain_load_per_force=float(static_gain[1, 1]),
            control_period_s=self.config.control_period_s,
            correction_limit_n=self.config.fr_feedback_limit_n,
            slew_n_s=self.config.fr_feedback_slew_n_s,
            tracking_gain=self.config.fr_feedback_tracking_gain,
            deadband_n=self.config.fr_feedback_deadband_n)
        self.static_support_gain = static_gain[np.ix_(SUPPORT, SUPPORT)]
        self.support = SupportForceFeedback(self.static_support_gain,
                                            control_period_s=self.config.control_period_s,
                                            correction_limit_n=self.config.support_feedback_limit_n,
                                            rebound_guard_mm=(self.config.moving_rebound_guard_mm
                                                              if crawl else self.config.rebound_guard_mm))
        self.load_filter = WheelLoadFilter(
            cutoff_hz=self.config.wheel_load_filter_cutoff_hz, wheel_count=4)
        self.drive = CrawlTorqueFeedback(control_period_s=self.config.control_period_s,
                                         tyre_radius_m=self.config.tyre_radius_m,
                                         half_track_m=self.config.tyre_half_track_m,
                                         friction=float(scenario["friction"]))
        self.crawl = crawl
        self.scenario = scenario
        self.mode = "INIT_SETTLE"
        self.mode_start_s = 0.
        self.settle_start_s = 0.
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

    def begin_settle(self, now_s: float) -> None:
        self.settle_start_s = now_s
        self._enter('INIT_SETTLE', now_s)

    def _enter(self, mode: str, now_s: float) -> None:
        self.mode = mode
        self.mode_start_s = now_s
        if mode == "ABORT_STOP":
            self.stop_since_s = None
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
                reference_time = min(reference_time, self.config.reference_hold_s)
        loads = np.array([np.interp(reference_time, self.ref_time, self.ref_fz[:, i])
                          for i in range(4)])
        if self.lift_start_s is None and self.preload_initial_bias_n is not None:
            start = self.config.preload_feedback_start_s
            remaining = np.clip((self.trace.time_s[-1] - reference_time) /
                                (self.trace.time_s[-1] - start), 0., 1.)
            loads = np.maximum(0., loads + remaining * self.preload_initial_bias_n)
        return loads

    def __call__(self, now_s: float, exports) -> tuple[float, ...]:
        x = dict(zip(EXPORTS, exports))
        if self.last_tick_s is None or now_s - self.last_tick_s >= self.config.control_period_s - 1e-8:
            self.last_tick_s = now_s
            self._tick(now_s, x)
        base = np.asarray(self.trace.at(self.preload_clock.progress_s), dtype=float)
        # The validated preload trace ends at 48 s and holds its final command.
        extra_fr = 0.
        extra_rl = 0.
        if self.lift_start_s is not None:
            lift_fraction = quintic_step(now_s, self.lift_start_s, self.config.lift_ramp_s)[0]
            extra_fr = self.config.fr_lift_force_n * lift_fraction
            extra_rl = self.config.rl_support_force_n * quintic_step(
                now_s, self.lift_start_s + self.config.lift_ramp_s,
                self.config.support_ramp_s)[0]
        if self.lower_start_s is not None:
            lower_fraction = quintic_step(now_s, self.lower_start_s,
                                          self.config.lower_ramp_s)[0]
            extra_fr *= 1. - lower_fraction
            extra_rl *= 1. - lower_fraction
            return_scale = 1. - quintic_step(now_s, self.lower_start_s +
                                             self.config.lower_ramp_s,
                                             self.config.return_ramp_s)[0]
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
                now_s, self.lower_start_s, self.config.torque_release_s)[0]
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
        since_settle_s = now_s - self.settle_start_s
        fz = np.array([max(0., x["Fz_" + wheel]) for wheel in WHEELS])
        gate_fz = self.load_filter.update(now_s, fz)
        if self.mode in ("INIT_SETTLE", "PRELOAD_SHIFT"):
            self.support.gain = blend_contact_gain(
                self.static_support_gain, self.swing_gain, float(gate_fz[1]),
                swing_n=self.config.gain_blend_swing_fr_load_n,
                stance_n=self.config.gain_blend_stance_fr_load_n)
            self.swing_gain_weight = float(np.clip(
                (self.config.gain_blend_stance_fr_load_n - gate_fz[1]) /
                (self.config.gain_blend_stance_fr_load_n -
                 self.config.gain_blend_swing_fr_load_n), 0., 1.))
        if self.mode in ("INIT_SETTLE", "PRELOAD_SHIFT"):
            tracking_error = float(np.max(np.abs(
                gate_fz - self._ref_loads(now_s))))
            self.preload_clock.update(since_settle_s, load_error_n=tracking_error)
        travel = np.array([x["Jnc_" + wheel] for wheel in WHEELS])
        contacts = {corner: (x["Xctc_" + wheel + "i"],
                             x["Yctc_" + wheel + "i"])
                    for corner, wheel in zip(("FL", "FR", "RL", "RR"), WHEELS)}
        com = np.array([x["XCG_TM"], x["YCG_TM"]])
        assessment = assess_support(contacts, dict(zip(("FL", "FR", "RL", "RR"), gate_fz)),
                                    tuple(com), self.config.lambda_target)
        triangle = np.array([contacts[c] for c in ("FL", "RL", "RR")])
        com_lambda = _barycentric(triangle, com)
        margin = min(assessment.lambda_min, float(np.min(com_lambda)))
        top_clearance = x["Z_R1"] - self.config.tyre_radius_m
        attitude = max(abs(x["Roll_E"]), abs(x["Pitch"]))
        support_load = float(min(gate_fz[SUPPORT]))
        if self.initial_yo_m is None and since_settle_s >= self.config.init_settle_s:
            self.initial_yo_m = x["Yo"]
            self.initial_yaw_deg = x["Yaw"]
        ready = (gate_fz[1] <= self.config.fr_ready_load_n and support_load >=
                 self.config.support_floor_n and margin >= self.config.lambda_target and
                 attitude <= self.config.ready_attitude_limit_deg and
                 abs(x["Vx"]) <= self.config.init_max_vx_kph)
        if ready:
            if self.ready_since_s is None:
                self.ready_since_s = now_s
        else:
            self.ready_since_s = None
        if self.mode == "INIT_SETTLE" and since_settle_s >= self.config.init_settle_s:
            if (np.min(gate_fz) > self.config.support_floor_n and
                    abs(x["Vx"]) <= self.config.init_max_vx_kph):
                if self.preload_clock.progress_s < self.config.preload_feedback_start_s:
                    self.preload_clock.align_reference(
                        since_settle_s, self.config.preload_feedback_start_s)
                if self.scenario.get('side') == 'left':
                    # The plant's fixed lateral payload bias survives reflection.
                    # Start tracking at measured static loads and fade the offset
                    # out before the original unload/stability gate.
                    self.preload_initial_bias_n = gate_fz - self._ref_loads(now_s)
                self._enter("PRELOAD_SHIFT", now_s)
        if (self.mode == "PRELOAD_SHIFT" and
                self.preload_clock.progress_s >= self.trace.time_s[-1]):
            if self.preload_end_since_s is None:
                self.preload_end_since_s = now_s
            if (self.ready_since_s is not None and
                    now_s - self.ready_since_s >= self.config.ready_dwell_s):
                self._enter("LIFTING", now_s)
            elif now_s - self.preload_end_since_s >= self.config.preload_end_wait_limit_s:
                self.abort_reason = "preload ready gate not reached"
                self._enter("LOWERING", now_s)
        if (self.mode == "PRELOAD_SHIFT" and
                now_s - self.mode_start_s >= self.config.preload_total_limit_s):
            self.abort_reason = "preload tracking deadline exceeded"
            self._enter("LOWERING", now_s)
        if (self.mode == "LIFTING" and now_s - self.lift_start_s >=
                self.config.lift_ramp_s + self.config.support_ramp_s and
                top_clearance >= self.config.lift_clearance_m and
                gate_fz[1] <= self.config.fr_swing_load_n):
            self._enter("THREE_WHEEL_HOLD", now_s)
        if (self.mode == "THREE_WHEEL_HOLD" and
                now_s - self.hold_start_s >= self.config.three_wheel_hold_s):
            self._enter("CRAWL" if self.crawl else "LOWERING", now_s)
        if self.mode == "CRAWL":
            if x["X_R1"] >= (float(self.scenario["start_station_m"]) +
                               float(self.scenario["length_m"]) -
                               self.config.brake_start_before_far_edge_m):
                self.stop_enter_speed_kph = max(0., x["Vx"])
                self._enter("STOP", now_s)
            elif now_s - self.crawl_start_s > self.config.max_crawl_s:
                self.abort_reason = "crawl did not cross before timeout"
                self._enter("ABORT_STOP", now_s)
        if self.mode == "STOP":
            fr_clear = x["X_R1"] >= (float(self.scenario["start_station_m"]) +
                                      float(self.scenario["length_m"]) +
                                      self.config.crossing_clearance_m)
            rear_clear = x["X_R2"] <= (float(self.scenario["start_station_m"]) -
                                       self.config.rear_stop_clearance_m)
            if fr_clear and rear_clear and abs(x["Vx"]) <= self.config.stop_speed_kph:
                if self.stop_since_s is None:
                    self.stop_since_s = now_s
                elif now_s - self.stop_since_s >= self.config.stop_dwell_s:
                    self._enter("LOWERING", now_s)
            else:
                self.stop_since_s = None
        if self.mode == "ABORT_STOP":
            # Keep FR lifted over the pit. Recover only after braking on solid
            # ground, with RR also on solid ground and three supports intact.
            can_recover = front_abort_recovery_ready(x, support_load_n=support_load,
                margin=margin, config=self.config, scenario=self.scenario)
            if can_recover:
                if self.stop_since_s is None:
                    self.stop_since_s = now_s
                elif now_s - self.stop_since_s >= self.config.stop_dwell_s:
                    self._enter("LOWERING", now_s)
            else:
                self.stop_since_s = None
        if (self.mode == "LOWERING" and now_s - self.lower_start_s >=
                self.config.lower_ramp_s and fz[1] > self.config.support_floor_n):
            self._enter("RETURN_TO_FOUR_WHEEL", now_s)
        if (self.mode == "RETURN_TO_FOUR_WHEEL" and
                now_s - self.lower_start_s >= self.config.lower_ramp_s +
                self.config.return_ramp_s and np.min(fz) > self.config.support_floor_n):
            self._enter("COMPLETE", now_s)
        if self.mode in ("LIFTING", "THREE_WHEEL_HOLD", "CRAWL", "STOP"):
            if (support_load < self.config.support_floor_n or margin < self.config.lambda_abort or
                    attitude > self.config.attitude_limit_deg or
                    min(travel) < (self.config.moving_rebound_abort_mm if self.crawl
                                   else self.config.rebound_abort_mm) or
                    max(travel) > self.config.jounce_abort_mm or
                    (self.mode in ("CRAWL", "STOP") and
                     top_clearance < front_clearance_guard_m(
                         x["X_R1"],
                         float(self.scenario["start_station_m"]) +
                         float(self.scenario["length_m"]),
                         self.config.crossing_clearance_m, self.config.lip_clearance_m,
                         self.config.post_pit_positive_clearance_m))):
                self.abort_reason = "three-wheel support or clearance limit"
                self._enter("ABORT_STOP", now_s)
        ref = self._ref_loads(now_s)
        if self.mode in ("PRELOAD_SHIFT", "LIFTING", "THREE_WHEEL_HOLD",
                         "CRAWL", "STOP"):
            if (self.mode in ("CRAWL", "STOP") and
                    (assessment.lambda_min < self.config.lambda_target or
                     min(com_lambda) < self.config.lambda_target)):
                target_lambda = np.maximum(com_lambda, self.config.lambda_target)
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
                target_speed = (self.config.crawl_speed_kph *
                                quintic_step(now_s, self.crawl_start_s,
                                             self.config.crawl_accel_ramp_s)[0]
                                if self.config.crawl_accel_ramp_s > 0.
                                else self.config.crawl_speed_kph)
            elif self.mode == "STOP":
                target_speed = self.stop_enter_speed_kph * (
                    1. - quintic_step(now_s, self.mode_start_s,
                                       self.config.stop_ramp_s)[0])
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
                    self.torques_nm, -self.config.stop_max_brake_nm)
                self.drive.torque_nm = self.torques_nm.copy()
        else:
            self.torques_nm[:] = 0.
            target_speed = 0.
        if self.lower_start_s is None:
            applied_scale = 1.
        else:
            applied_scale = 1. - quintic_step(
                now_s, self.lower_start_s + self.config.lower_ramp_s,
                self.config.return_ramp_s)[0]
        if self.mode == "LOWERING":
            applied_torque = self.lower_start_torques_nm * (
                1. - quintic_step(now_s, self.lower_start_s,
                                   self.config.torque_release_s)[0])
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
                self.config.support_feedback_limit_n - 1e-8)),
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


