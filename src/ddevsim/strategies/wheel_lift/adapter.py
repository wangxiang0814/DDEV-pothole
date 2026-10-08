"""SI boundary around the unchanged native-unit wheel-lift controller."""
from __future__ import annotations

from dataclasses import replace
import json
import math

from ddevsim.runtime.contracts import ControlCommand, StrategyStatus
from ddevsim.units import si_factor
from ddevsim.static_wheel_lift.config import CLOSED_LOOP_RUN, REAR_CYCLE_RUN
from .front import EXPORTS, EVIDENCE
from .cycle import FullRightSideController


class WheelLiftStrategy:
    implemented = True

    def __init__(self):
        self.controller = None

    def reset(self, context):
        if context.strategy_id != 'wheel_lift':
            raise ValueError('wrong strategy context')
        front = replace(CLOSED_LOOP_RUN, **dict(context.config.get('front', {})))
        rear = replace(REAR_CYCLE_RUN, **dict(context.config.get('rear', {})))
        if (front.control_period_s != context.control_period_s or
                rear.control_period_s != context.control_period_s):
            raise ValueError('context and controller periods differ')
        gain_path = context.assets.get('gain_matrix', EVIDENCE / 'gain_matrix.json')
        from pathlib import Path
        gain = json.loads(Path(gain_path).read_text(encoding='utf-8'))['gains']['Fz_n']
        self.controller = FullRightSideController(
            scenario=dict(context.config['scenario']),
            rr_gain_per_coupled_force=float(
                gain[3][1] + REAR_CYCLE_RUN.preload_feedback_fl_per_fr * gain[3][0]),
            front_config=front, rear_config=rear,
            front_steering_feedback=bool(context.config.get('front_steering_feedback', False)))
        self.context = context

    def step(self, observation):
        if self.controller is None:
            raise RuntimeError('strategy must be reset before step')
        if observation.valid:
            observation.require(EXPORTS)
            exports = [observation.values_si[name] / si_factor(name) for name in EXPORTS]
        else:
            # Use the original controller's stale-feedback safe hold, never fake feedback.
            exports = []
        self.controller.set_feedback_timestamp(
            observation.time_s, observation.observed_at_s if observation.valid else None)
        values = self.controller(observation.time_s, exports)
        return ControlCommand(values[:4], values[4:8], math.radians(values[8]))

    def status(self):
        if self.controller is None:
            return StrategyStatus('NOT_INITIALIZED')
        selected = self.controller.rear if self.controller.rear_started else self.controller.front
        return StrategyStatus(selected.mode, selected.mode == 'RR_COMPLETE', selected.abort_reason)

    def summary(self):
        state = self.status()
        return {'strategy_id': 'wheel_lift', 'phase': state.phase,
                'abort_reason': state.abort_reason,
                'task_status': 'ABORTED' if state.abort_reason else 'UNASSESSED',
                'note': 'Task PASS requires native truth and strategy acceptance; use the verified CLI.'}
