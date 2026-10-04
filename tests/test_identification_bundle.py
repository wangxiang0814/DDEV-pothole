import copy
import numpy as np
import pytest

from ddevsim.static_wheel_lift.system_identification import assemble_support_bundle


def records():
    shapes = {'Fz_n': (4, 4), 'CoM_xy_m': (2, 4), 'ZMP_xy_m': (2, 4),
              'attitude_rad': (2, 4), 'travel_m': (4, 4), 'wheel_height_m': (4, 4)}
    return {mode: {'status': 'PASS', 'mode': mode, 'source_model_sha256': 'same',
                   'gains': {k: np.zeros(s).tolist() for k, s in shapes.items()}}
            for mode in ('FOUR_CONTACT', 'FR', 'RR')}


def test_bundle_keeps_distinct_contact_modes_and_does_not_mutate_records():
    inputs = records()
    before = copy.deepcopy(inputs)
    bundle = assemble_support_bundle(inputs)
    assert bundle['source_model_sha256'] == 'same'
    assert set(bundle['modes']) == set(inputs)
    assert inputs == before


@pytest.mark.parametrize('fault', ['hash', 'failed', 'height'])
def test_bundle_rejects_mismatched_failed_or_incomplete_models(fault):
    inputs = records()
    if fault == 'hash':
        inputs['RR']['source_model_sha256'] = 'different'
    elif fault == 'failed':
        inputs['FR']['status'] = 'FAIL'
    else:
        del inputs['RR']['gains']['wheel_height_m']
    with pytest.raises(ValueError):
        assemble_support_bundle(inputs)
