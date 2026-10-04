"""Finite actuator-force bias: plant disturbance, never an observation change."""
from dataclasses import asdict
import math
import numpy as np
from .config import SupportForcePulseConfig
from .quintic_trajectory import quintic_step


class SupportForcePulse:
    def __init__(self, config=SupportForcePulseConfig()):
        self.config = config
        self.start_s = None
        self.release_start_s = None
        self.previous_bias_n = 0.
        self.frozen_bias_n = 0.
        self.interrupted = False
        self.completed = False
        self.clipped = False
        self.peak_applied_n = 0.
        self.telemetry = {}

    def apply(self, now_s, command, *, stage, phase):
        output = np.asarray(command, dtype=float).copy()
        if (output.shape != (9,) or not np.isfinite(output).all() or
                not math.isfinite(now_s) or stage not in ('FR', 'RR')):
            raise ValueError('invalid disturbance input')
        cfg = self.config
        target_stage = 'FR' if cfg.phase == 'THREE_WHEEL_HOLD' else 'RR'
        if self.start_s is None and stage == target_stage and phase == cfg.phase:
            self.start_s = now_s + cfg.start_delay_s
        requested = 0.
        if self.start_s is not None and stage == target_stage:
            if not self.interrupted and phase != cfg.phase and not self.completed:
                self.interrupted = True
                self.frozen_bias_n = self.previous_bias_n
            if self.interrupted:
                if stage == 'FR':
                    # FR recovery has no force snapshot; hold bias during braking,
                    # then release smoothly with its established lowering path.
                    requested = self.frozen_bias_n
                    if phase in ('LOWERING', 'RETURN_TO_FOUR_WHEEL', 'COMPLETE'):
                        if self.release_start_s is None:
                            self.release_start_s = now_s
                        requested *= 1. - quintic_step(now_s, self.release_start_s,
                                                      cfg.recovery_release_s)[0]
                # RR's abort snapshot contains the previous actual applied bias.
                # Adding it again here would double the physical disturbance.
            elif phase == cfg.phase and not self.completed:
                rise = quintic_step(now_s, self.start_s, cfg.ramp_s)[0]
                fall = quintic_step(now_s, self.start_s + cfg.ramp_s + cfg.hold_s,
                                    cfg.ramp_s)[0]
                requested = cfg.amplitude_n * rise * (1. - fall)
                self.completed = now_s >= self.start_s + 2. * cfg.ramp_s + cfg.hold_s
        index = 4 + ('FL', 'FR', 'RL', 'RR').index(cfg.corner)
        original = output.copy()
        if requested != 0.:
            output[index] = np.clip(output[index] + requested, -cfg.force_limit_n, cfg.force_limit_n)
        applied = float(output[index] - original[index])
        self.previous_bias_n = applied
        self.peak_applied_n = max(self.peak_applied_n, abs(applied))
        self.clipped |= not math.isclose(applied, requested, abs_tol=1e-8)
        self.telemetry = dict(disturbance_requested_n=requested, disturbance_applied_n=applied,
                              disturbance_corner=cfg.corner, disturbance_interrupted=self.interrupted)
        for i, corner in enumerate(('fl', 'fr', 'rl', 'rr')):
            self.telemetry[f'command_fact_{corner}_n'] = float(original[4 + i])
        return tuple(output)

    def summary(self):
        delivered = (self.completed and not self.interrupted and not self.clipped and
                     math.isclose(self.peak_applied_n, abs(self.config.amplitude_n),
                                  rel_tol=1e-9, abs_tol=1e-6))
        return dict(config=asdict(self.config), delivered=delivered, started=self.start_s is not None,
                    pulse_start_s=self.start_s, completed=self.completed,
                    interrupted=self.interrupted, clipped=self.clipped,
                    peak_applied_n=self.peak_applied_n,
                    scope='unknown actuator-force bias after control allocation; not external wind/terrain force')
