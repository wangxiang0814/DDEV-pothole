import numpy as np

from ddevsim.static_wheel_lift.suspension_allocator import allocate_support_increment
from ddevsim.static_wheel_lift.config import SUPPORT_QP


def inputs():
    gf = np.array([[-1., .2, 1., 0.], [0., 0., 0., 0.],
                   [1., -.2, -1., -1.], [0., 0., 0., 1.]])
    loads = np.array([6000., 0., 1000., 7000.])
    xy = np.array([[1., 1.], [1., -1.], [-1., 1.], [-1., -1.]])
    zmp = loads @ xy / loads.sum()
    return dict(lifted_corner='FR', contacts_xy=xy, fz_n=loads,
                fz_target_n=np.array([5900., 0., 1200., 6900.]),
                com_xy=zmp, zmp_xy=zmp, attitude_rad=np.zeros(2),
                attitude_target_rad=np.zeros(2), travel_m=np.zeros(4),
                wheel_height_m=np.array([0., .04, 0., 0.]),
                gains={'Fz_n': gf, 'CoM_xy_m': np.zeros((2, 4)),
                       'ZMP_xy_m': np.zeros((2, 4)),
                       'attitude_rad': np.zeros((2, 4)),
                       'travel_m': np.zeros((4, 4)), 'wheel_height_m': np.zeros((4, 4))},
                base_force_n=np.zeros(4), previous_applied_n=np.zeros(4),
                correction_n=np.zeros(3), dt_s=.02, config=SUPPORT_QP)


def test_support_qp_must_not_push_lifted_wheel_into_ground():
    a = inputs()
    a['wheel_height_m'][1] = .009
    result = allocate_support_increment(**a)
    assert result.status == 'INFEASIBLE'


def test_support_qp_preserves_swing_and_respects_force_rate():
    a = inputs()
    result = allocate_support_increment(**a)
    assert result.status == 'OPTIMAL'
    assert result.force_n[1] == 0.
    assert np.max(abs(result.force_n)) <= SUPPORT_QP.force_slew_n_s * .02 + 1e-6
    assert sum(result.predicted_fz_n) == sum(a['fz_n'])
    assert result.predicted_fz_n[2] > a['fz_n'][2]


def test_known_lift_force_is_kept_and_its_coupling_is_predicted():
    a = inputs()
    a['base_force_n'][1] = 10.
    result = allocate_support_increment(**a)
    assert result.status == 'OPTIMAL'
    assert result.force_n[1] == 10.
    np.testing.assert_allclose(result.predicted_fz_n,
                               a['fz_n'] + a['gains']['Fz_n'] @ result.force_n)


def test_unreachable_travel_constraint_reports_infeasible():
    a = inputs()
    a['travel_m'][0] = -.16
    result = allocate_support_increment(**a)
    assert result.status == 'INFEASIBLE'
    assert result.force_n is None
