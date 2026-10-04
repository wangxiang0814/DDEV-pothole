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
        drive=SimpleNamespace(torque_nm=np.array([10.,30.,-20.]),last_update_s=1.,integral_n=5.),
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
    np.testing.assert_array_equal(rear.drive.torque_nm,np.array(c.last_command)[[2,0,1]])
    assert rear.drive.last_update_s==1.04
    assert rear.drive.integral_n==5.


def test_valid_delay_passes_and_future_packet_is_rejected():
    c=object.__new__(FullRightSideController)
    c.set_feedback_timestamp(1.,.9595)
    assert c.feedback_fresh
    c.set_feedback_timestamp(1.,1.1)
    assert not c.feedback_fresh
    c.set_feedback_timestamp(1.,None)
    assert not c.feedback_fresh


def test_real_speed_controller_cannot_restore_old_drive_after_outage():
    from ddevsim.static_wheel_lift.closed_loop import CrawlTorqueFeedback
    c=object.__new__(FullRightSideController)
    drive=CrawlTorqueFeedback(control_period_s=.02,tyre_radius_m=.263,
                             half_track_m=.625,friction=.7)
    drive.torque_nm[:]=100.
    drive.last_update_s=1.
    rear=SimpleNamespace(mode='RR_ABORT_STOP',abort_reason='feedback packet stale',rows=[],drive=drive)
    c.rear=rear;c.rear_started=True;c.last_command=(30.,30.,30.,0.,100.,200.,300.,-200.,0.)
    c.last_command_time_s=1.;c.last_stale_log_s=None
    c.set_feedback_timestamp(1.4,1.)
    c(1.4,[])
    np.testing.assert_allclose(drive.torque_nm,0.)
    braking=drive.update(1.42,vx_kph=3.,yaw_deg=0.,yaw_rate_deg_s=0.,
                         lateral_m=0.,loads_n=[4000.,4000.,4000.],target_kph=0.)
    assert max(abs(braking)) <= drive.config.max_torque_slew_nm_s*.02+1e-8
