"""Explicit development interface; scene preparation is already available."""
from ddevsim.runtime.contracts import StrategyStatus


class ShallowPressStrategy:
    implemented = False

    def __init__(self):
        self.context = None

    def reset(self, context):
        if context.strategy_id != 'shallow_press':
            raise ValueError('wrong strategy context')
        self.context = context

    def step(self, observation):
        raise NotImplementedError('shallow_press control law is not implemented')

    def status(self):
        return StrategyStatus('NOT_IMPLEMENTED')

    def summary(self):
        return {'strategy_id': 'shallow_press', 'task_status': 'NOT_IMPLEMENTED',
                'criteria': {}, 'unassessed': ['bottom_contact', 'stability', 'slip',
                                             'path', 'exit_success', 'comfort']}
