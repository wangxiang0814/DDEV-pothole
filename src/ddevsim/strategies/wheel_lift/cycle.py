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

from . import front as front_run

from ddevsim.cosim import run_stepwise
from ddevsim.pothole_case import corner_module_scenario
from ddevsim.visual_mesh import write_visual_mesh
from ddevsim.static_wheel_lift.config import (REAR_CYCLE_RUN,
                                             RIGHT_SIDE_MODEL, TUNED_REAR_RUN,
                                             FAST_CYCLE_FRONT, FAST_CYCLE_REAR,
                                             BALANCED_CYCLE_FRONT, CLOSED_LOOP_RUN,
                                             COMPACT_CYCLE_FRONT, EFFICIENT_CYCLE_FRONT, COMPACT_REAR_RETURN_RAMP_S,
                                             ROBUST_REAR_RUN,
                                             rear_speed_trial_config)
from ddevsim.static_wheel_lift.rear_cycle import RearCycleController
from ddevsim.static_wheel_lift.observation_noise import (
    ObservationNoiseConfig, NoisyFeedbackTrial)
from ddevsim.static_wheel_lift.system_identification import validated_contact_gains, validated_support_models
from ddevsim.static_wheel_lift.support_allocation_feedback import SupportAllocationFeedback
from ddevsim.static_wheel_lift.path_reference import StaticPathReference
from ddevsim.static_wheel_lift.config import PATH_REFERENCE
from ddevsim.static_wheel_lift.config import FEEDBACK_HEALTH
from ddevsim.static_wheel_lift.force_disturbance import SupportForcePulse
from ddevsim.static_wheel_lift.config import SupportForcePulseConfig
from ddevsim.static_wheel_lift.pit_scene_transfer import (
    replace_pit_geometry_parameters, validated_pit_scene_models)
from ddevsim.static_wheel_lift.environment_gain_transfer import validated_environment_models

STEERING_IMPORT = "IMP_STEER_SW"


class FullRightSideController:
    """Keep the validated FR controller intact; hand over after recovery."""

    def __init__(self, *, scenario: dict, rr_gain_per_coupled_force: float,
                 identify_rr_swing: bool = False,
                 steer_probe_deg: float | None = None,
                 front_steering_feedback: bool = False,
                 contact_gains=None,
                 allocation=None,
                 path_reference=None,
                 disturbance=None,
                 rear_config=REAR_CYCLE_RUN, front_config=CLOSED_LOOP_RUN):
        self.front = front_run.LiftCrawlController(crawl=True, scenario=scenario,
                                                   contact_gains=contact_gains, config=front_config)
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
        self.disturbance = disturbance
        self.feedback_fresh = True
        self.last_command = (0.,) * 9
        self.last_command_time_s = None
        self.last_stale_log_s = None

    def set_feedback_timestamp(self, now_s, measurement_time_s):
        if measurement_time_s is None:
            self.feedback_fresh = False
            return
        age = now_s - measurement_time_s
        self.feedback_fresh = bool(np.isfinite(age) and 0. <= age <= FEEDBACK_HEALTH.max_packet_age_s)

    def __call__(self, now_s, exports):
        if not getattr(self, 'feedback_fresh', True):
            self.feedback_was_stale=True
            selected = self.rear if self.rear_started else self.front
            if selected.abort_reason is None:
                selected.abort_reason = 'feedback packet stale'
            abort_mode = 'RR_ABORT_STOP' if self.rear_started else 'ABORT_STOP'
            if selected.mode != abort_mode:
                selected._enter(abort_mode, now_s)
            # No optimization, progress, exit or lowering from an old packet.
            # Keep actual suspension/steering; relinquish drive with bounded slew.
            output = np.asarray(self.last_command, dtype=float).copy()
            dt = 0. if self.last_command_time_s is None else max(0., now_s-self.last_command_time_s)
            step = FEEDBACK_HEALTH.torque_release_slew_nm_s * dt
            output[:4] += np.clip(-output[:4], -step, step)
            if hasattr(selected, 'drive'):
                indices = [2,0,1] if self.rear_started else [0,2,3]
                selected.drive.torque_nm = output[indices].copy()
                selected.drive.last_update_s = now_s
                if self.rear_started:
                    selected.torque = selected.drive.torque_nm.copy()
                else:
                    selected.torques_nm = selected.drive.torque_nm.copy()
            if getattr(self, 'allocation', None) is not None:
                # Keep actuator history current even while no QP is run, so
                # fresh feedback cannot spend the outage as one large rate step.
                self.allocation.last_applied = output[4:8].copy()
                self.allocation.last_application_s = now_s
            if selected.rows and (self.last_stale_log_s is None or
                    now_s-self.last_stale_log_s >= FEEDBACK_HEALTH.log_period_s-1e-8):
                row = {**selected.rows[-1], 'time_s': now_s, 'mode': abort_mode,
                       'feedback_fresh': False, 'speed_ref_kph': 0.,
                       'support_qp_status': 'STALE_PACKET', 'support_qp_failure_s': 0.}
                for i,c in enumerate(('fl','fr','rl','rr')):
                    row[f'torque_{c}_nm'],row[f'fact_{c}_n']=float(output[i]),float(output[4+i])
                selected.rows.append(row)
                self.last_stale_log_s=now_s
            command = tuple(map(float, output))
        else:
            command = self._call_fresh(now_s, exports)
            selected = self.rear if self.rear_started else self.front
            if getattr(selected, 'rows', None) and selected.rows[-1]['time_s'] == now_s:
                selected.rows[-1]['feedback_fresh'] = True
        self.last_command, self.last_command_time_s = command, now_s
        return command

    def _disturb(self, now_s, command, stage):
        if self.disturbance is None:
            return command
        selected = self.front if stage == 'FR' else self.rear
        actual = self.disturbance.apply(now_s, command, stage=stage, phase=selected.mode)
        if stage == 'RR':
            # Recovery must snapshot actual inputs, including the unknown bias.
            self.rear.last_suspension_force_n = np.asarray(actual[4:8]).copy()
        if selected.rows and selected.rows[-1]['time_s'] == now_s:
            row = selected.rows[-1]
            row.update(self.disturbance.telemetry)
            for i, c in enumerate(('fl', 'fr', 'rl', 'rr')):
                row[f'fact_{c}_n'] = float(actual[4 + i])
        return actual

    def _allocate(self, now_s, x, command, stage):
        if self.allocation is None:
            return command
        selected = self.front if stage == 'FR' else self.rear
        output = self.allocation.apply(now_s, x, command, stage=stage, phase=selected.mode)
        if stage == 'RR':
            self.rear.last_suspension_force_n = np.asarray(output[4:8]).copy()
            self.rear.recovery_feedback_healthy = (self.allocation.active and
                self.allocation.telemetry.get('support_qp_status') == 'OPTIMAL')
        if selected.rows and selected.rows[-1]['time_s'] == now_s:
            row = selected.rows[-1]
            row.update(self.allocation.telemetry)
            for i, c in enumerate(('fl', 'fr', 'rl', 'rr')):
                row[f'fact_{c}_n'] = float(output[4 + i])
        if (selected.mode not in ('ABORT_STOP', 'RR_ABORT_STOP') and
                self.allocation.active and self.allocation.telemetry['support_qp_failure_s'] >=
                self.allocation.config.max_continuous_failure_s):
            if selected.abort_reason is None:
                selected.abort_reason = 'support allocation persistently infeasible'
            selected._enter('ABORT_STOP' if stage == 'FR' else 'RR_ABORT_STOP', now_s)
        return output

    def _call_fresh(self, now_s: float, exports) -> tuple[float, ...]:
        if getattr(self,'feedback_was_stale',False):
            if self.rear_started:
                x=dict(zip(front_run.EXPORTS,exports))
                self.rear.resume_abort_braking(now_s,x['Vx'])
            self.feedback_was_stale=False
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
            return self._disturb(now_s, self._allocate(now_s, dict(zip(front_run.EXPORTS, exports)),
                                  (*command, self.front_steer_deg), 'FR'), 'FR')
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
        return self._disturb(now_s, self._allocate(now_s, x, (*command, steer_deg), 'RR'), 'RR')


