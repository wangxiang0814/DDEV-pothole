import importlib

import numpy as np
from types import SimpleNamespace


def test_noise_reproducible_and_held_between_control_ticks():
    module = importlib.import_module('ddevsim.static_wheel_lift.observation_noise')
    config = module.ObservationNoiseConfig()
    first = module.ObservationNoise(config)
    second = module.ObservationNoise(config)
    truth = {'Fz_L1': 3000., 'Fz_R1': 0., 'Yo': .1, 'Yaw': 0.,
             'Roll_E': 0., 'Pitch': 0., 'Vx': 0., 'X_R1': 99.}
    at_zero = first.observe(0., truth)
    assert at_zero == second.observe(0., truth)
    later_truth = {**truth, 'Fz_L1': 3100.}
    at_half_tick = first.observe(.0005, later_truth)
    assert np.isclose(at_half_tick['Fz_L1'] - 3100., at_zero['Fz_L1'] - 3000.)
    assert first.observe(.02, truth) != at_zero
    assert truth['Fz_L1'] == 3000.
    assert at_zero['Fz_R1'] >= 0.
    assert at_zero['X_R1'] == truth['X_R1']


def test_truth_assessment_never_uses_noisy_load_or_pose():
    module = importlib.import_module('ddevsim.static_wheel_lift.observation_noise')
    truth = {'Yo': .02, 'Yaw': .1, 'Roll_E': 1., 'Pitch': 2., 'Vx': 3.,
             'XCG_TM': -.2, 'YCG_TM': .2, 'X_R1': 100., 'X_R2': 98.,
             'Z_R1': .4, 'Z_R2': .3}
    for wheel, xy, load in zip(('L1', 'R1', 'L2', 'R2'),
                              ((1., .6), (1., -.6), (-1., .6), (-1., -.6)),
                              (4000., 0., 2000., 4000.)):
        truth[f'Xctc_{wheel}i'], truth[f'Yctc_{wheel}i'] = xy
        truth[f'Fz_{wheel}'] = load
        truth[f'Jnc_{wheel}'] = 0.
    observed = {'time_s': 0., 'mode': 'CRAWL', 'fz_rl_n': 9000.,
                'yo_m': 9., 'torque_fl_nm': 17.}
    audit = module.TruthAudit(tyre_radius_m=.3, lambda_safe=.05)
    row = audit.capture(observed, truth, lifted_corner='FR')
    assert row['fz_rl_n'] == 2000.
    assert row['yo_m'] == .02
    assert row['torque_fl_nm'] == 17.
    assert row['mode'] == 'CRAWL'
    assert row['min_support_n'] == 2000.
    assert np.isclose(row['fr_top_clearance_m'], .1)
    assert observed['yo_m'] == 9.


def test_noisy_trial_records_truth_path_origin_at_reference_capture():
    module = importlib.import_module('ddevsim.static_wheel_lift.observation_noise')
    from ddevsim.static_wheel_lift.config import CLOSED_LOOP_RUN, REAR_CYCLE_RUN

    class Controller:
        def __init__(self):
            self.front = SimpleNamespace(rows=[], initial_yo_m=None)
            self.rear = SimpleNamespace(rows=[], config=REAR_CYCLE_RUN)

        def __call__(self, now, exports):
            self.front.initial_yo_m = exports[0]
            return (0.,) * 9

    controller = Controller()
    trial = module.NoisyFeedbackTrial(controller, ('Yo', 'Yaw'), CLOSED_LOOP_RUN)
    trial(0., (.123, .456))
    assert controller.front.initial_yo_m != .123
    assert trial.path_reference_yo_m == .123
    assert trial.path_reference_yaw_deg == .456
    trial(.02, (.8, .9))
    assert trial.path_reference_yo_m == .123
