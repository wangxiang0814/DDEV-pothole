from dataclasses import replace
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np
from ddevsim.static_wheel_lift.config import ROBUST_REAR_RUN
from ddevsim.static_wheel_lift.rear_cycle import RearCycleController
from test_rear_cycle import _rear_observation


def setup_exit():
    c = RearCycleController(scenario={'friction':.6,'start_station_m':101.1,'length_m':.8},
        rr_gain_per_coupled_force=-.4,config=replace(ROBUST_REAR_RUN,allow_abort_exit=True))
    c.mode='RR_ABORT_STOP'
    c.abort_reason='original fault'
    c.recovery_feedback_healthy=True
    x=_rear_observation();x['X_R2']=101.5;x['Vx']=0.
    return c,x


def test_exit_requires_continuous_healthy_stopped_dwell():
    c,x=setup_exit()
    c(0.,x);c(1.,x)
    assert c.mode=='RR_ABORT_STOP'
    c.recovery_feedback_healthy=False;c(1.02,x)
    c.recovery_feedback_healthy=True;c(2.,x);c(3.,x)
    assert c.mode=='RR_ABORT_STOP'
    c(4.02,x)
    assert c.mode=='RR_ABORT_EXIT' and c.abort_reason=='original fault'


def test_exit_bad_clearance_blocks_start_despite_safe_loads():
    c,x=setup_exit();x['Z_R2']=c.config.tyre_radius_m+.005
    c(0.,x);c(3.,x)
    assert c.mode=='RR_ABORT_STOP'


def test_failed_exit_returns_to_brake_without_retry_or_reason_loss():
    c,x=setup_exit();c(0.,x);c(2.02,x)
    assert c.mode=='RR_ABORT_EXIT'
    c.recovery_feedback_healthy=False;c(2.04,x)
    assert c.mode=='RR_ABORT_STOP'
    c.recovery_feedback_healthy=True;c(3.,x);c(6.,x)
    assert c.mode=='RR_ABORT_STOP' and c.abort_reason=='original fault'


def test_exit_reaches_solid_ground_then_stops_before_lowering():
    c,x=setup_exit();c(0.,x);c(2.02,x)
    x['X_R2']=102.4;x['Vx']=.7;c(2.04,x)
    assert c.mode=='RR_ABORT_STOP'
    c(3.1,x);assert c.mode=='RR_ABORT_STOP'
    x['Vx']=0.;c(3.12,x);c(4.14,x)
    assert c.mode=='RR_LOWERING'


def test_reentry_snapshots_last_actual_force_once():
    c,x=setup_exit()
    c.abort_suspension_force_n=np.array([1.,2.,3.,4.])
    c.last_suspension_force_n=np.array([101.,202.,303.,-200.])
    c.mode='RR_ABORT_EXIT';c._enter('RR_ABORT_STOP',1.)
    np.testing.assert_array_equal(c.abort_suspension_force_n,[101.,202.,303.,-200.])


def test_safe_abort_braking_uses_measured_speed_and_smooth_reference():
    c,x=setup_exit();c.mode='RR_CRAWL';x['Vx']=3.
    c(0.,x);c._enter('RR_ABORT_STOP',0.)
    c(.02,x)
    assert 2.9<c.rows[-1]['speed_ref_kph']<=3.
    c(.5,x)
    assert 1.4<c.rows[-1]['speed_ref_kph']<1.6
    c(1.02,x)
    assert c.rows[-1]['speed_ref_kph']==0.


def test_unsafe_support_cancels_gentle_braking_reference():
    c,x=setup_exit();c.mode='RR_CRAWL';x['Vx']=3.
    c(0.,x);c._enter('RR_ABORT_STOP',0.)
    x['Roll_E']=20.
    c(.02,x)
    assert c.rows[-1]['speed_ref_kph']==0. and c.mode=='RR_ABORT_STOP'


def test_safe_braking_does_not_require_permission_to_drive_out():
    c,x=setup_exit()
    c.config=replace(c.config,allow_abort_exit=False)
    c.mode='RR_CRAWL';x['Vx']=3.
    c(0.,x);c._enter('RR_ABORT_STOP',0.)
    c(.02,x)
    assert c.abort_smooth_stop and c.rows[-1]['speed_ref_kph']>2.9
    x['Vx']=0.
    c(1.02,x);c(4.,x)
    assert c.mode=='RR_ABORT_STOP' and not c.abort_exit_attempted
