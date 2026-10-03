import numpy as np
import pytest
import json

from ddevsim.static_wheel_lift.system_identification import assemble_settled_gains, validated_probe_window, validated_contact_gains
from ddevsim.static_wheel_lift.command_trace import cycle_hold_trace


def cases():
    result = {}
    g = np.eye(4) - np.ones((4, 4)) / 4
    for i, wheel in enumerate(('FL', 'FR', 'RL', 'RR')):
        for sign, label in ((1, 'pos'), (-1, 'neg')):
            result[f'{wheel}_{label}'] = {
                'native': {'status': 'COMPLETED'}, 'settle_status': 'SETTLED',
                'response': {'Fz_n': (3000 + sign * 250 * g[:, i]).tolist()},
            }
    return result, g


def test_settled_pairs_recover_load_conservation_and_rank():
    observations, expected = cases()
    result = assemble_settled_gains(observations, amplitude_n=250)
    np.testing.assert_allclose(result['gains']['Fz_n'], expected)
    assert result['G_F_diagnostic']['effective_rank'] == 3
    assert result['G_F_diagnostic']['requires_damping']


def test_singular_probe_diagnostic_is_strict_json_serializable():
    observations, _ = cases()
    for case in observations.values():
        case['response']['Fz_n'] = [3000.] * 4
    result = assemble_settled_gains(observations, amplitude_n=250)
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize('failure', ['NOT_SETTLED', 'NATIVE_FAILED'])
def test_failed_probe_cannot_publish_usable_gains(failure):
    observations, _ = cases()
    if failure == 'NOT_SETTLED':
        observations['RL_neg']['settle_status'] = failure
    else:
        observations['RL_neg']['native']['status'] = 'FAILED'
    with pytest.raises(ValueError, match='RL_neg'):
        assemble_settled_gains(observations, amplitude_n=250)


def test_hold_replay_uses_previous_command_and_never_starts_crawl():
    rows = []
    for t, mode, force in [(0., 'INIT_SETTLE', 0.), (1., 'THREE_WHEEL_HOLD', 10.),
                           (4., 'THREE_WHEEL_HOLD', 20.), (5., 'CRAWL', 999.)]:
        row = {'time_s': t, 'mode': mode, 'steer_sw_deg': 0.}
        row.update({f'fact_{c}_n': force for c in ('fl', 'fr', 'rl', 'rr')})
        row.update({f'torque_{c}_nm': 0. for c in ('fl', 'rl', 'rr')})
        rows.append(row)
    trace, freeze_s = cycle_hold_trace(rows, 'FR', hold_age_s=3.)
    assert freeze_s == 4.
    assert trace.at(3.9)[4] == 10.
    assert trace.at(100.)[4] == 20.
    assert trace.at(100.)[1] == 0.  # Lifted FR torque is absent from FR logs.


@pytest.mark.parametrize('native_status', ['FAILED', 'COMPLETED'])
def test_early_native_stop_reports_unusable_window(native_status):
    with pytest.raises(ValueError, match='native|window'):
        validated_probe_window({'status': native_status}, [{'time_s': 3.}],
                               start_s=16., end_s=18., min_samples=100)


def test_control_gain_bundle_rejects_different_plant():
    bundle = {'modes': {mode: {'status': 'PASS', 'mode': mode,
                              'source_model_sha256': 'measured-plant',
                              'gains': {'Fz_n': np.eye(4).tolist()}}
                        for mode in ('FOUR_CONTACT', 'FR', 'RR')}}
    with pytest.raises(ValueError, match='model'):
        validated_contact_gains(bundle, model_sha256='other-plant')
    result = validated_contact_gains(bundle, model_sha256='measured-plant')
    np.testing.assert_allclose(result['FR'], np.eye(4))
