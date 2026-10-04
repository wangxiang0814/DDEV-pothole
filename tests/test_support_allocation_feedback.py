import numpy as np
import pytest

from ddevsim.static_wheel_lift.config import CLOSED_LOOP_RUN, ROBUST_REAR_RUN
from ddevsim.static_wheel_lift.support_allocation_feedback import SupportAllocationFeedback
from ddevsim.static_wheel_lift.system_identification import validated_support_models


def test_online_allocation_rejects_missing_height_model_before_native_run():
    bundle = {'modes': {mode: {'status': 'PASS', 'mode': mode,
                              'source_model_sha256': 'plant', 'gains': {'Fz_n': np.eye(4).tolist()}}
                        for mode in ('FOUR_CONTACT', 'FR', 'RR')}}
    with pytest.raises(ValueError, match='missing support model'):
        validated_support_models(bundle, model_sha256='plant')


def controller():
    return SupportAllocationFeedback(mode_models={}, active=True, period_s=.02,
                                     front_limits=CLOSED_LOOP_RUN, rear_limits=ROBUST_REAR_RUN)


def test_lowering_handoff_starts_from_current_correction_and_finishes_at_zero():
    c = controller()
    c.stage = 'FR'
    c.correction = np.array([100., -50., 20.])
    command = (0.,) * 9
    first = c.apply(10., {}, command, stage='FR', phase='LOWERING')
    np.testing.assert_allclose(np.asarray(first)[4:8], [100., 0., -50., 20.])
    final = c.apply(14., {}, command, stage='FR', phase='RETURN_TO_FOUR_WHEEL')
    np.testing.assert_allclose(final, 0.)


def test_rear_abort_snapshot_is_not_double_added_by_outer_feedback():
    c = controller()
    c.stage = 'RR'
    c.last_update_s = 10.  # Native handoff between QP ticks does not double-add.
    c.correction = np.array([100., -50., 20.])
    snapshot = (0.,) * 4 + (500., 600., 700., -200., 0.)
    assert c.apply(10., {}, snapshot, stage='RR', phase='RR_ABORT_STOP') == snapshot


def test_lift_transition_uses_height_progress_not_finished_clearance(monkeypatch):
    from types import SimpleNamespace
    import ddevsim.static_wheel_lift.support_allocation_feedback as module
    captured = []
    monkeypatch.setattr(module, 'allocate_support_increment', lambda **kw: (
        captured.append(kw) or SimpleNamespace(status='OPTIMAL', force_n=np.zeros(4),
                                              cost=0., predicted_margin=.1)))
    c = SupportAllocationFeedback(mode_models={m: {'gains': {}} for m in ('FOUR_CONTACT', 'FR', 'RR')},
                                  active=True, period_s=.02, front_limits=CLOSED_LOOP_RUN,
                                  rear_limits=ROBUST_REAR_RUN, transitions=True)
    x = {'Roll_E': 0., 'Pitch': 0., 'AVx': 0., 'AVy': 0., 'XCG_TM': -.4, 'YCG_TM': .4}
    for w, xy, load in zip(('L1','R1','L2','R2'), ((1,1),(1,-1),(-1,1),(-1,-1)), (6000,0,2000,4000)):
        x.update({f'Xctc_{w}i': xy[0], f'Yctc_{w}i': xy[1], f'Fz_{w}': load,
                  f'Jnc_{w}': 0., f'Z_{w}': .263})
    c.apply(1., x, (0.,)*9, stage='FR', phase='LIFTING')
    assert len(captured) == 1
    assert captured[0]['config'].clearance_floor_m <= 0.
    assert c.target_att is None  # Capture a fixed hold posture only on entering HOLD.
    x['Z_R1'] = .283
    x['Roll_E'] = 2.
    c.apply(2., x, (0.,)*9, stage='FR', phase='THREE_WHEEL_HOLD')
    assert captured[-1]['config'].clearance_floor_m == .01
    assert captured[-1]['dt_s'] <= .02  # A phase gap must not permit a large correction step.
    assert c.target_att is not None
    target = c.target_att.copy()
    x['Roll_E'] = 4.
    c.apply(3., x, (0.,)*9, stage='FR', phase='STOP')
    np.testing.assert_array_equal(c.target_att, target)


def test_loaded_preload_does_not_use_three_contact_gain_or_clearance_gate():
    c = SupportAllocationFeedback(mode_models={}, active=True, period_s=.02,
                                  front_limits=CLOSED_LOOP_RUN, rear_limits=ROBUST_REAR_RUN,
                                  transitions=True)
    x = {f'Fz_{w}': 3000. for w in ('L1','R1','L2','R2')}
    assert c.apply(1., x, (0.,)*9, stage='FR', phase='PRELOAD_SHIFT') == (0.,)*9
    assert c.telemetry['support_qp_status'] == 'WAITING_CONTACT'


def test_unloaded_preload_still_waits_for_safe_support_triangle():
    c = SupportAllocationFeedback(mode_models={}, active=True, period_s=.02,
                                  front_limits=CLOSED_LOOP_RUN, rear_limits=ROBUST_REAR_RUN,
                                  transitions=True)
    x = {'XCG_TM': 1., 'YCG_TM': -1.}
    for w, xy, load in zip(('L1','R1','L2','R2'), ((1,1),(1,-1),(-1,1),(-1,-1)), (6000,0,2000,4000)):
        x.update({f'Xctc_{w}i': xy[0], f'Yctc_{w}i': xy[1], f'Fz_{w}': load})
    assert c.apply(1., x, (0.,)*9, stage='FR', phase='PRELOAD_SHIFT') == (0.,)*9
    assert c.telemetry['support_qp_status'] == 'WAITING_TRIANGLE'


def test_rear_posture_feedback_tracks_intended_roll_instead_of_blocking_adjustment(monkeypatch):
    from types import SimpleNamespace
    import ddevsim.static_wheel_lift.support_allocation_feedback as module
    captured = []
    monkeypatch.setattr(module, 'allocate_support_increment', lambda **kw: (
        captured.append(kw) or SimpleNamespace(status='OPTIMAL', force_n=np.zeros(4),
                                              cost=0., predicted_margin=.1)))
    c = SupportAllocationFeedback(mode_models={'RR': {'gains': {}}}, active=True, period_s=.02,
                                  front_limits=CLOSED_LOOP_RUN, rear_limits=ROBUST_REAR_RUN,
                                  transitions=True)
    x = {'Roll_E': -5., 'Pitch': 0., 'AVx': 0., 'AVy': 0., 'XCG_TM': 1/3, 'YCG_TM': 1/3}
    for w, xy, load in zip(('L1','R1','L2','R2'), ((1,1),(1,-1),(-1,1),(-1,-1)), (4000,4000,4000,0)):
        x.update({f'Xctc_{w}i': xy[0], f'Yctc_{w}i': xy[1], f'Fz_{w}': load,
                  f'Jnc_{w}': 0., f'Z_{w}': .263})
    c.apply(1., x, (0.,)*9, stage='RR', phase='RR_POSTURE')
    assert np.rad2deg(captured[-1]['attitude_target_rad'][0]) == pytest.approx(ROBUST_REAR_RUN.posture_target_roll_deg)
    x['Roll_E'], x['Z_R2'] = -8., .303
    c.apply(2., x, (0.,)*9, stage='RR', phase='RR_HOLD')
    assert np.rad2deg(c.target_att[0]) == pytest.approx(ROBUST_REAR_RUN.posture_target_roll_deg)


def test_rear_abort_keeps_closed_loop_after_one_snapshot_handoff(monkeypatch):
    from types import SimpleNamespace
    import ddevsim.static_wheel_lift.support_allocation_feedback as module
    captured = []
    def solve(**kw):
        captured.append(kw)
        force = np.asarray(kw['base_force_n']) + np.r_[kw['correction_n'],0.]
        force[0] += 4.
        return SimpleNamespace(status='OPTIMAL',force_n=force,cost=0.,predicted_margin=.1)
    monkeypatch.setattr(module,'allocate_support_increment',solve)
    c = SupportAllocationFeedback(mode_models={'RR':{'gains':{}}},active=True,
        period_s=.02,front_limits=CLOSED_LOOP_RUN,rear_limits=ROBUST_REAR_RUN,transitions=True)
    c.stage='RR'
    c.correction=np.array([100.,-50.,20.])
    x=dict(Roll_E=-7.2,Pitch=0.,AVx=0.,AVy=0.,XCG_TM=.4,YCG_TM=.4)
    for w,xy,load in zip(('L1','R1','L2','R2'),((1,1),(1,-1),(-1,1),(-1,-1)),(4000,4000,4000,0)):
        x.update({f'Xctc_{w}i':xy[0],f'Yctc_{w}i':xy[1],f'Fz_{w}':load,
                  f'Jnc_{w}':0.,f'Z_{w}':.303})
    snapshot=(0.,)*4+(500.,600.,700.,-200.,0.)
    first=c.apply(1.,x,snapshot,stage='RR',phase='RR_ABORT_STOP')
    second=c.apply(1.02,x,snapshot,stage='RR',phase='RR_ABORT_STOP')
    assert len(captured)==2
    assert captured[0]['config'].force_slew_n_s == 1200.
    np.testing.assert_allclose(captured[0]['correction_n'],0.)
    np.testing.assert_allclose(captured[1]['correction_n'],[4.,0.,0.])
    assert first[4]==504. and second[4]==508.


def test_persistent_failure_does_not_reenter_abort_or_replace_initial_reason():
    import sys
    from pathlib import Path
    from types import SimpleNamespace
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
    from run_right_side_full_cycle import FullRightSideController
    c=object.__new__(FullRightSideController)
    events=[]
    c.rear=SimpleNamespace(mode='RR_ABORT_STOP',rows=[],abort_reason='injected fault',
        _enter=lambda *args:events.append(args))
    c.allocation=SimpleNamespace(active=True,telemetry={'support_qp_failure_s':1.},
        config=SimpleNamespace(max_continuous_failure_s=.5),apply=lambda *args,**kw:args[2])
    c._allocate(1.,{},(0.,)*9,'RR')
    assert events==[] and c.rear.abort_reason=='injected fault'


def test_abort_output_limits_actual_native_step_not_only_qp_average():
    c = controller()
    c.stage = 'RR'
    c.phase = 'RR_ABORT_STOP'
    c.last_update_s = 1.
    c.last_application_s = 1.
    c.correction = np.array([24., -24., 0.])
    first = c.apply(1.0005, {}, (0.,)*9, stage='RR', phase='RR_ABORT_STOP')
    np.testing.assert_allclose(first[4:8], [22.7, -22.7, 0., 0.])
    second = c.apply(1.001, {}, (0.,)*9, stage='RR', phase='RR_ABORT_STOP')
    np.testing.assert_allclose(second[4:8], [24., -24., 0., 0.])


def test_abort_lowering_releases_actual_correction_not_unapplied_target():
    c = controller()
    c.stage = 'RR'
    c.phase = 'RR_ABORT_STOP'
    c.correction = np.array([24., -24., 0.])
    c.last_applied = np.array([522.7, 577.3, 700., -200.])
    snapshot = (0.,)*4 + (500.,600.,700.,-200.,0.)
    output = c.apply(1., {}, snapshot, stage='RR', phase='RR_LOWERING')
    np.testing.assert_allclose(output[4:8], c.last_applied)
    np.testing.assert_allclose(c.release_correction, [22.7,-22.7,0.])
