from pathlib import Path
import sys
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

from run_rear_recovery_trial import inject_preload_abort


def test_fault_injection_targets_preload_only_and_preserves_reason():
    events = []
    rear = SimpleNamespace(mode='RR_HOLD', mode_start_s=10., abort_reason=None,
                           _enter=lambda mode, t: events.append((mode, t)))
    assert not inject_preload_abort(rear, 12., after_s=1.)
    rear.mode = 'RR_PRELOAD'
    assert not inject_preload_abort(rear, 10.5, after_s=1.)
    assert inject_preload_abort(rear, 11., after_s=1.)
    assert rear.abort_reason == 'injected pit-front preload abort'
    assert events == [('RR_ABORT_STOP', 11.)]
