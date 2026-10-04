import sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from run_right_side_full_cycle import FullRightSideController


def test_stale_packet_latches_abort_freezes_force_and_retracts_drive():
    c=object.__new__(FullRightSideController)
    events=[]
    rear=SimpleNamespace(mode='RR_HOLD',abort_reason=None,rows=[],
        _enter=lambda mode,now:(events.append((mode,now)),setattr(rear,'mode',mode)))
    c.rear=rear;c.rear_started=True
    c.last_command=(30.,-20.,10.,0.,100.,200.,300.,-200.,12.)
    c.last_command_time_s=1.
    c.last_stale_log_s=None
    c.allocation=SimpleNamespace(last_application_s=1.,last_applied=np.zeros(4))
    c.set_feedback_timestamp(1.02,.8)
    out=c(1.02,[])
    assert events==[('RR_ABORT_STOP',1.02)]
    assert rear.abort_reason=='feedback packet stale'
    np.testing.assert_array_equal(out[4:],c.last_command[4:])
    assert max(abs(x) for x in out[:4])<30.
    c.set_feedback_timestamp(1.04,.8);c(1.04,[])
    assert len(events)==1
    assert c.allocation.last_application_s==1.04
    np.testing.assert_array_equal(c.allocation.last_applied,c.last_command[4:8])


def test_valid_delay_passes_and_future_packet_is_rejected():
    c=object.__new__(FullRightSideController)
    c.set_feedback_timestamp(1.,.9595)
    assert c.feedback_fresh
    c.set_feedback_timestamp(1.,1.1)
    assert not c.feedback_fresh
    c.set_feedback_timestamp(1.,None)
    assert not c.feedback_fresh
