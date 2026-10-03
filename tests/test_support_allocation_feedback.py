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
    c.correction = np.array([100., -50., 20.])
    snapshot = (0.,) * 4 + (500., 600., 700., -200., 0.)
    assert c.apply(10., {}, snapshot, stage='RR', phase='RR_ABORT_STOP') == snapshot
