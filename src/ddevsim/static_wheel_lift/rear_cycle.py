"""Measured RR lift and second right-track crossing after FR recovery."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .closed_loop import (CrawlFeedbackConfig, CrawlTorqueFeedback,
                          ScalarLoadFeedback, WheelLoadFilter)
from .config import REAR_CYCLE_RUN, RearCycleConfig
from .quintic_trajectory import quintic_step
from .support_geometry import assess_support, _lambda


WHEEL_NAMES = ("FL", "FR", "RL", "RR")
TRUCKSIM_WHEELS = ("L1", "R1", "L2", "R2")
RR_SUPPORT = np.array([0, 1, 2])


@dataclass(frozen=True)
class RearGate:
    ready: bool
    safe: bool
    zmp_lambda_min: float
    com_lambda_min: float
    min_support_n: float
    clearance_m: float


def assess_rear_gate(x: dict[str, float], filtered_loads_n: np.ndarray,
                     config: RearCycleConfig = REAR_CYCLE_RUN) -> RearGate:
    """Use RR's own load/clearance and the FL-FR-RL support triangle."""
    contacts = {name: (x[f"Xctc_{wheel}i"], x[f"Yctc_{wheel}i"])
                for name, wheel in zip(WHEEL_NAMES, TRUCKSIM_WHEELS)}
    com = (x["XCG_TM"], x["YCG_TM"])
    assessment = assess_support(
        contacts, dict(zip(WHEEL_NAMES, filtered_loads_n)), com,
        config.lambda_safe, lifted_corner="RR")
    triangle = tuple(contacts[c] for c in assessment.support_corners)
    com_min = min(_lambda(triangle, com))
    minimum = float(np.min(filtered_loads_n[RR_SUPPORT]))
    clearance = x["Z_R2"] - config.tyre_radius_m
    attitude = max(abs(x["Roll_E"]), abs(x["Pitch"]))
    travel = np.array([x[f"Jnc_{wheel}"] for wheel in TRUCKSIM_WHEELS])
    actuator_safe = (float(np.min(travel)) > config.travel_rebound_abort_mm and
                     float(np.max(travel)) < config.travel_jounce_abort_mm)
    safe = (minimum >= config.support_floor_n and
            assessment.lambda_min >= config.lambda_abort and
            com_min >= config.lambda_abort and
            attitude <= config.attitude_limit_deg and actuator_safe)
    ready = (safe and filtered_loads_n[3] <= config.wheel_unloaded_n and
             assessment.lambda_min >= config.lambda_safe and
             com_min >= config.lambda_safe and
             abs(x["Vx"]) <= config.stationary_kph)
    return RearGate(ready, safe, assessment.lambda_min, com_min,
                    minimum, clearance)


class RearCycleController:
    """RR contact switch, continuous suspension commands, and three-wheel crawl."""

    def __init__(self, *, scenario: dict, rr_gain_per_coupled_force: float,
                 config: RearCycleConfig = REAR_CYCLE_RUN):
        if (not math.isfinite(rr_gain_per_coupled_force) or
                abs(rr_gain_per_coupled_force) < 1e-6):
            raise ValueError("identified RR unload gain required")
        self.scenario = scenario
        self.config = config
        self.mode = "RR_SETTLE"
        self.mode_start_s: float | None = None
        self.last_tick_s: float | None = None
        self.ready_since_s: float | None = None
        self.stop_since_s: float | None = None
        self.posture_start_s: float | None = None
        self.posture_release_start_s: float | None = None
        self.posture_stop_release_start_s: float | None = None
        self.stop_roll_counter_start_s: float | None = None
        self.preload_start_s: float | None = None
        self.lift_start_s: float | None = None
        self.last_suspension_force_n = np.zeros(4)
        self.abort_suspension_force_n: np.ndarray | None = None
        self.preload_reference_s = 0.0
        self.preload_clock_update_s: float | None = None
        self.preload_reference_rate = 1.0
        self.start_rr_load_n: float | None = None
        self.abort_reason: str | None = None
        self.filter = WheelLoadFilter(cutoff_hz=5.0, wheel_count=4)
        self.unload_feedback = ScalarLoadFeedback(
            gain_load_per_force=rr_gain_per_coupled_force,
            control_period_s=config.control_period_s,
            correction_limit_n=config.preload_feedback_limit_n,
            slew_n_s=config.preload_feedback_slew_n_s,
            tracking_gain=config.preload_feedback_gain,
            deadband_n=config.preload_feedback_deadband_n)
        self.drive = CrawlTorqueFeedback(
            control_period_s=config.control_period_s,
            tyre_radius_m=config.tyre_radius_m,
            half_track_m=config.tyre_half_track_m,
            friction=float(scenario["friction"]),
            config=CrawlFeedbackConfig(
                yaw_kp_nm_per_deg=config.crawl_yaw_kp_nm_per_deg,
                yaw_kd_nm_per_deg_s=config.crawl_yaw_kd_nm_per_deg_s,
                lateral_kp_n_per_m=config.crawl_lateral_kp_n_per_m))
        self.torque = np.zeros(3)
        self.parking_torque = np.zeros(4)
        self.steer_deg = 0.0
        self.start_stop_speed_kph = 0.0
        self.stop_travel_relief_n = 0.0
        self.initial_yaw_deg: float | None = None
        self.initial_yo_m: float | None = None
        self.rows: list[dict] = []

    def _enter(self, mode: str, now_s: float) -> None:
        if mode == 'RR_ABORT_STOP' and self.abort_suspension_force_n is None:
            self.abort_suspension_force_n = self.last_suspension_force_n.copy()
        if mode == "RR_CRAWL" and self.config.parking_damping_nm_per_rpm > 0.:
            self.drive.torque_nm = self.parking_torque[[2, 0, 1]].copy()
        self.mode = mode
        self.mode_start_s = now_s
        self.ready_since_s = None
        self.stop_since_s = None

    def _preload_fractions(self, now_s: float) -> tuple[float, float]:
        cfg = self.config
        elapsed = (self.preload_reference_s if cfg.adaptive_preload else
                   now_s - self.preload_start_s)
        if cfg.sequential_preload:
            split = cfg.preload_ramp_s * cfg.preload_diagonal_fraction
            return (quintic_step(elapsed, 0., split)[0],
                    quintic_step(elapsed, split, cfg.preload_ramp_s - split)[0])
        fraction = quintic_step(elapsed, 0., cfg.preload_ramp_s)[0]
        return fraction, fraction

    def __call__(self, now_s: float, exports: dict[str, float]) -> tuple[float, ...]:
        x = exports
        if self.mode_start_s is None:
            self.mode_start_s = now_s
        if (self.last_tick_s is None or
                now_s - self.last_tick_s >= self.config.control_period_s - 1e-8):
            self.last_tick_s = now_s
            self._tick(now_s, x)
        cfg = self.config
        command = np.zeros(8)
        if self.preload_start_s is not None and self.mode != "RR_COMPLETE":
            diagonal_preload, preload = self._preload_fractions(now_s)
            if self.mode in ("RR_LOWERING", "RR_RETURN"):
                release = 1. - quintic_step(now_s, self.lower_start_s,
                                            cfg.lower_ramp_s + cfg.return_ramp_s)[0]
            else:
                release = 1.
            command[4] += (cfg.preload_fl_force_n * diagonal_preload +
                           cfg.preload_feedback_fl_per_fr *
                           self.unload_feedback.correction_n) * release
            if self.posture_start_s is not None:
                posture = quintic_step(now_s, self.posture_start_s,
                                       cfg.posture_ramp_s)[0]
                if self.posture_release_start_s is not None:
                    blend = quintic_step(now_s, self.posture_release_start_s,
                                         cfg.posture_release_s)[0]
                    hold_fraction = (cfg.posture_hold_fl_force_n /
                                     cfg.posture_fl_force_n)
                    posture = (1. - blend) * posture + blend * hold_fraction
                if self.posture_stop_release_start_s is not None:
                    posture *= 1. - quintic_step(
                        now_s, self.posture_stop_release_start_s,
                        cfg.posture_stop_release_s)[0]
                command[4] += cfg.posture_fl_force_n * release * posture
            if self.stop_roll_counter_start_s is not None:
                command[4] += (cfg.stop_roll_counter_fl_n * release *
                               quintic_step(now_s, self.stop_roll_counter_start_s,
                                            cfg.stop_roll_counter_ramp_s)[0])
            command[5] += (cfg.preload_fr_force_n * preload +
                           self.unload_feedback.correction_n +
                           self.stop_travel_relief_n) * release
            command[6] = cfg.preload_rl_force_n * preload * release
            if self.lift_start_s is not None and self.mode != 'RR_PRELOAD':
                lift = quintic_step(now_s, self.lift_start_s,
                                    cfg.lift_ramp_s)[0]
                if self.mode in ("RR_LOWERING", "RR_RETURN"):
                    lift *= 1. - quintic_step(now_s, self.lower_start_s,
                                              cfg.lower_ramp_s)[0]
                command[7] = cfg.lift_force_n * lift
        if self.abort_suspension_force_n is not None and self.mode in (
                'RR_ABORT_STOP', 'RR_LOWERING', 'RR_RETURN'):
            # Freeze actual inputs at the fault; do not finish a preload script.
            release, lift_release = 1., 1.
            if self.mode in ('RR_LOWERING', 'RR_RETURN'):
                release -= quintic_step(now_s, self.lower_start_s,
                                        cfg.lower_ramp_s + cfg.return_ramp_s)[0]
                lift_release -= quintic_step(now_s, self.lower_start_s,
                                             cfg.lower_ramp_s)[0]
            command[4:7] = self.abort_suspension_force_n[:3] * release
            command[7] = self.abort_suspension_force_n[3] * lift_release
        self.last_suspension_force_n = command[4:].copy()
        if self.mode in ("RR_CRAWL", "RR_STOP", "RR_ABORT_STOP"):
            # CrawlTorqueFeedback's three entries are left, left, right.
            # Here they map to RL, FL, FR; RR remains undriven in swing.
            command[[2, 0, 1]] = self.torque
        elif self.mode in ("RR_SETTLE", "RR_PRELOAD", "RR_LIFTING",
                            "RR_POSTURE", "RR_HOLD", "RR_LOWERING", "RR_RETURN"):
            command[:4] = self.parking_torque
        if self.rows and self.rows[-1]["time_s"] == now_s:
            for i, wheel in enumerate(WHEEL_NAMES):
                self.rows[-1][f"torque_{wheel.lower()}_nm"] = float(command[i])
                self.rows[-1][f"fact_{wheel.lower()}_n"] = float(command[4 + i])
        return tuple(map(float, command))

    def _tick(self, now_s: float, x: dict[str, float]) -> None:
        cfg = self.config
        fz = np.array([max(0., x[f"Fz_{wheel}"]) for wheel in TRUCKSIM_WHEELS])
        filtered = self.filter.update(now_s, fz)
        gate = assess_rear_gate(x, filtered, cfg)
        rpm = np.array([x.get(f"AVy_{wheel}", 0.) for wheel in TRUCKSIM_WHEELS])
        parking_target = np.clip(-cfg.parking_damping_nm_per_rpm * rpm,
                                 -cfg.parking_limit_nm, cfg.parking_limit_nm)
        if self.mode not in ("RR_SETTLE", "RR_PRELOAD", "RR_LIFTING",
                              "RR_POSTURE", "RR_HOLD"):
            parking_target[:] = 0.
        parking_step = cfg.parking_slew_nm_s * cfg.control_period_s
        self.parking_torque += np.clip(parking_target - self.parking_torque,
                                      -parking_step, parking_step)
        # Resist wheel motion only, and never torque the lifted RR.
        self.parking_torque[fz <= cfg.wheel_unloaded_n] = 0.
        self.parking_torque[self.parking_torque * rpm > 0.] = 0.
        if self.initial_yaw_deg is None:
            self.initial_yaw_deg, self.initial_yo_m = x["Yaw"], x["Yo"]
        if self.mode == "RR_SETTLE":
            if (now_s - self.mode_start_s >= cfg.settle_dwell_s and
                    abs(x["Vx"]) <= cfg.stationary_kph and
                    np.min(filtered) >= cfg.support_floor_n):
                self.preload_start_s = now_s
                self.start_rr_load_n = float(filtered[3])
                self._enter("RR_PRELOAD", now_s)
        if (cfg.adaptive_preload and self.preload_start_s is not None and
                self.mode in ("RR_PRELOAD", "RR_LIFTING", "RR_POSTURE", "RR_HOLD")):
            dt = (0.0 if self.preload_clock_update_s is None else
                  now_s - self.preload_clock_update_s)
            self.preload_clock_update_s = now_s
            self.preload_reference_rate = float(np.clip(
                (cfg.preload_yaw_pause_deg_s - abs(x["AVz"])) /
                (cfg.preload_yaw_pause_deg_s - cfg.preload_yaw_slow_deg_s),
                0.0, 1.0))
            self.preload_reference_s = min(
                cfg.preload_ramp_s,
                self.preload_reference_s + dt * self.preload_reference_rate)
        if self.mode == "RR_PRELOAD":
            elapsed = now_s - self.preload_start_s
            diagonal, other = self._preload_fractions(now_s)
            ref = (self.start_rr_load_n or 0.) * (
                1. - cfg.preload_fl_unload_share * diagonal -
                (1. - cfg.preload_fl_unload_share) * other)
            self.unload_feedback.update(now_s, measured_n=float(filtered[3]),
                                        reference_n=ref)
            if gate.ready:
                if self.ready_since_s is None:
                    self.ready_since_s = now_s
                elif now_s - self.ready_since_s >= cfg.ready_dwell_s:
                    self.lift_start_s = now_s
                    self._enter("RR_LIFTING", now_s)
            else:
                self.ready_since_s = None
            if (self.mode == "RR_PRELOAD" and
                    elapsed >= cfg.preload_ready_timeout_s):
                self.abort_reason = "RR unload or support gate not reached"
                self.lower_start_s = now_s
                self.lift_start_s = now_s
                self._enter("RR_LOWERING", now_s)
        if self.mode == "RR_LIFTING":
            if (now_s - self.lift_start_s >= cfg.lift_ramp_s and
                    gate.clearance_m >= cfg.lift_entry_clearance_m and
                    filtered[3] <= cfg.wheel_unloaded_n):
                if self.posture_start_s is None:
                    self.posture_start_s = now_s
                self._enter("RR_POSTURE", now_s)
            elif (now_s - self.lift_start_s >= cfg.lift_ramp_s and
                  gate.ready and self.posture_start_s is None):
                # An unloaded RR can remain almost touching with the scalar
                # lift force alone. Apply the existing smooth body adjustment
                # while stationary, retaining the clearance/contact entry gate.
                self.posture_start_s = now_s
            elif now_s - self.lift_start_s >= cfg.lift_timeout_s:
                self.abort_reason = "RR lift clearance gate not reached"
                self.lower_start_s = now_s
                self._enter("RR_LOWERING", now_s)
        if self.mode == "RR_POSTURE":
            if (self.posture_release_start_s is None and
                    x["Roll_E"] <= cfg.posture_target_roll_deg and
                    gate.clearance_m >= cfg.posture_target_clearance_m):
                self.posture_release_start_s = now_s
            if (self.posture_release_start_s is not None and
                    now_s - self.posture_release_start_s >=
                    cfg.posture_release_s + cfg.posture_settle_s):
                self._enter("RR_HOLD", now_s)
            elif now_s - self.posture_start_s >= cfg.posture_timeout_s:
                self.abort_reason = "RR posture clearance gate not reached"
                self.lower_start_s = now_s
                self._enter("RR_LOWERING", now_s)
        if self.mode == "RR_HOLD" and now_s - self.mode_start_s >= cfg.hold_s:
            self._enter("RR_CRAWL", now_s)
        far_edge = (float(self.scenario["start_station_m"]) +
                    float(self.scenario["length_m"]))
        if self.mode == "RR_CRAWL":
            if x["X_R2"] >= far_edge - cfg.brake_start_before_far_edge_m:
                self.start_stop_speed_kph = max(0., x["Vx"])
                self._enter("RR_STOP", now_s)
            elif now_s - self.mode_start_s >= cfg.max_crawl_s:
                self.abort_reason = "RR crossing deadline exceeded"
                self._enter("RR_ABORT_STOP", now_s)
        if self.mode == "RR_STOP":
            # Positive FR spring-seat force extends FR in the measured model.
            # Reduce it only once RR has cleared the far edge, with bounded slew.
            desired_relief = 0.0
            if x["X_R2"] >= far_edge + cfg.crossing_clearance_m:
                desired_relief = -float(np.clip(
                    (cfg.stop_travel_relief_start_mm - x["Jnc_R1"]) *
                    cfg.stop_travel_relief_gain_n_per_mm,
                    0., cfg.stop_travel_relief_limit_n))
            relief_step = cfg.stop_travel_relief_slew_n_s * cfg.control_period_s
            self.stop_travel_relief_n += float(np.clip(
                desired_relief - self.stop_travel_relief_n,
                -relief_step, relief_step))
            if self.posture_stop_release_start_s is None:
                self.posture_stop_release_start_s = now_s
            if (self.stop_roll_counter_start_s is None and
                    x["X_R2"] >= far_edge):
                self.stop_roll_counter_start_s = now_s
            if (x["X_R2"] >= far_edge + cfg.crossing_clearance_m and
                    abs(x["Vx"]) <= cfg.stationary_kph):
                if self.stop_since_s is None:
                    self.stop_since_s = now_s
                elif now_s - self.stop_since_s >= cfg.stop_dwell_s:
                    self.lower_start_s = now_s
                    self._enter("RR_LOWERING", now_s)
            else:
                self.stop_since_s = None
        if self.mode == "RR_ABORT_STOP":
            # Recovery is allowed on solid ground after stopping; never lower
            # RR into the known pit. Preserve abort_reason so this remains FAIL.
            can_recover = (
                (x["X_R2"] >= far_edge + cfg.crossing_clearance_m or
                 x["X_R2"] <= float(self.scenario["start_station_m"]) - cfg.crossing_clearance_m) and
                abs(x["Vx"]) <= cfg.stationary_kph and
                gate.min_support_n >= cfg.support_floor_n and
                gate.zmp_lambda_min >= cfg.lambda_abort and
                gate.com_lambda_min >= cfg.lambda_abort)
            if can_recover:
                if self.stop_since_s is None:
                    self.stop_since_s = now_s
                elif now_s - self.stop_since_s >= cfg.stop_dwell_s:
                    self.lower_start_s = now_s
                    self._enter("RR_LOWERING", now_s)
            else:
                self.stop_since_s = None
        if self.mode == "RR_LOWERING" and now_s - self.lower_start_s >= cfg.lower_ramp_s:
            if filtered[3] >= cfg.support_floor_n:
                self._enter("RR_RETURN", now_s)
        if self.mode == "RR_RETURN" and now_s - self.lower_start_s >= (
                cfg.lower_ramp_s + cfg.return_ramp_s):
            if np.min(filtered) >= cfg.support_floor_n:
                self._enter("RR_COMPLETE", now_s)
        if self.mode in ("RR_LIFTING", "RR_POSTURE",
                         "RR_HOLD", "RR_CRAWL", "RR_STOP"):
            if (not gate.safe or
                    self.mode in ("RR_CRAWL", "RR_STOP") and
                    gate.clearance_m < cfg.lip_clearance_m):
                travel = [x[f"Jnc_{wheel}"] for wheel in TRUCKSIM_WHEELS]
                if min(travel) <= cfg.travel_rebound_abort_mm:
                    self.abort_reason = "RR support suspension rebound limit"
                elif max(travel) >= cfg.travel_jounce_abort_mm:
                    self.abort_reason = "RR support suspension jounce limit"
                elif max(abs(x["Roll_E"]), abs(x["Pitch"])) > cfg.attitude_limit_deg:
                    self.abort_reason = "RR attitude limit"
                elif gate.min_support_n < cfg.support_floor_n:
                    self.abort_reason = "RR support wheel load limit"
                elif not gate.safe:
                    self.abort_reason = "RR support triangle limit"
                else:
                    self.abort_reason = "RR clearance limit"
                self._enter("RR_ABORT_STOP", now_s)
        if self.mode in ("RR_CRAWL", "RR_STOP", "RR_ABORT_STOP"):
            if self.mode == "RR_CRAWL":
                target_speed = (cfg.crawl_speed_kph *
                                quintic_step(now_s, self.mode_start_s,
                                             cfg.crawl_accel_ramp_s)[0])
            elif self.mode == "RR_STOP":
                target_speed = self.start_stop_speed_kph * (
                    1. - quintic_step(now_s, self.mode_start_s,
                                       cfg.stop_ramp_s)[0])
            else:
                target_speed = 0.
            self.torque = self.drive.update(
                now_s, vx_kph=x["Vx"],
                yaw_deg=x["Yaw"] - self.initial_yaw_deg,
                yaw_rate_deg_s=x["AVz"],
                lateral_m=x["Yo"] - self.initial_yo_m,
                loads_n=fz[[2, 0, 1]], target_kph=target_speed)
            if self.mode in ("RR_STOP", "RR_ABORT_STOP"):
                self.torque = np.maximum(self.torque, -cfg.stop_max_brake_nm)
                self.drive.torque_nm = self.torque.copy()
        else:
            self.torque[:] = 0.
        if (self.mode in ("RR_CRAWL", "RR_STOP") or
                cfg.static_steering_feedback and self.mode in
                ("RR_PRELOAD", "RR_LIFTING", "RR_POSTURE", "RR_HOLD")):
            desired_steer = float(np.clip(
                -cfg.steer_lateral_deg_per_m * (x["Yo"] - self.initial_yo_m)
                -cfg.steer_yaw_deg_per_deg * (x["Yaw"] - self.initial_yaw_deg),
                -cfg.steer_limit_deg, cfg.steer_limit_deg))
        else:
            desired_steer = 0.0
        steer_step = cfg.steer_slew_deg_s * cfg.control_period_s
        self.steer_deg += float(np.clip(desired_steer - self.steer_deg,
                                        -steer_step, steer_step))
        self.rows.append({
            "time_s": now_s, "mode": self.mode,
            "x_rr_m": x["X_R2"], "vx_kph": x["Vx"],
            "yo_m": x["Yo"], "yaw_deg": x["Yaw"],
            "roll_deg": x["Roll_E"], "pitch_deg": x["Pitch"],
            "zmp_lambda_min": gate.zmp_lambda_min,
            "com_lambda_min": gate.com_lambda_min,
            "min_support_n": gate.min_support_n,
            "rr_clearance_m": gate.clearance_m,
            "rr_ready": gate.ready, "rr_safe": gate.safe,
            "preload_feedback_n": self.unload_feedback.correction_n,
            "preload_reference_s": self.preload_reference_s,
            "preload_reference_rate": self.preload_reference_rate,
            "stop_travel_relief_n": self.stop_travel_relief_n,
            **{f"wheel_{wheel.lower()}_rpm": float(rpm[i])
               for i, wheel in enumerate(WHEEL_NAMES)},
            "steer_sw_deg": self.steer_deg,
            **{f"fz_{wheel.lower()}_n": float(fz[i])
               for i, wheel in enumerate(WHEEL_NAMES)},
            **{f"travel_{wheel.lower()}_mm": x[f"Jnc_{truck_wheel}"]
               for wheel, truck_wheel in zip(WHEEL_NAMES, TRUCKSIM_WHEELS)},
        })
