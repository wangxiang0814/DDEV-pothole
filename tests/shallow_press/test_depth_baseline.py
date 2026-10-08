import importlib.util
from pathlib import Path

import pytest


def module():
    path = Path(__file__).resolve().parents[2] / 'scripts/run_shallow_depth_baseline.py'
    spec = importlib.util.spec_from_file_location('depth_baseline', path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def test_speed_baseline_limits_commands_and_keeps_suspension_passive():
    m = module()
    names = ['Vx', 'X_L2', 'X_R2', 'Roll_E', 'Pitch']
    control = m.Baseline(names)
    for tick in range(1500):
        command = control(tick * .02, [0., 98., 98., 0., 0.])
        assert abs(command[0]) <= 250.
        assert command[:4] == (command[0],) * 4
        assert command[4:] == (0.,) * 5
    assert command[0] == pytest.approx(250.)


def test_long_contact_loss_requires_consecutive_samples():
    m = module()
    assert m.longest([True, True, False, True], .005) == pytest.approx(.01)


@pytest.mark.parametrize('native', [
    {'status': 'TERMINATED', 'error_message': ''},
    {'status': 'COMPLETED', 'error_message': 'native integration error'}])
def test_native_non_exception_failures_are_rejected(native):
    m = module()
    with pytest.raises(RuntimeError, match='native diagnostic failed'):
        m.validate_native_result(native)


def test_invalid_changed_geometry_is_rejected_before_directory_claim(monkeypatch):
    m = module()
    monkeypatch.setattr(m.sys, 'argv', ['probe', '--case', 'wide', '--depth-m', '.06',
                                      '--edge-m', '.2', '--run-id', 'test_invalid_no_claim'])
    claimed = []
    monkeypatch.setattr(m, 'claim_run', lambda *args: claimed.append(args))
    with pytest.raises(ValueError, match='tyre envelope'):
        m.main()
    assert claimed == []


def test_wide_window_includes_first_front_entry_and_last_rear_exit(tmp_path):
    import csv
    from types import SimpleNamespace
    m = module()
    rows = []
    for t in [4., 4.5, 5.5, 6., 7.]:
        row = {'time_s': t, 'imp_IMP_MYUSM_L1': 0.}
        for key in ['Yo', 'Yaw', 'Zo', 'Roll_E', 'Pitch', 'Vx', 'Az_SM',
                    'X_L1', 'X_R1', 'X_L2', 'X_R2']:
            row['exp_' + key] = 0.
        row['exp_Vx'] = 3.
        for w in m.WHEELS:
            for key in ['Fz_'+w, 'Jnc_'+w, 'CmpT_'+w+'i', 'Kappa_'+w+'i',
                        'Zgnd_'+w+'i', 'AVy_'+w]:
                row['exp_' + key] = 0.
            row['exp_Fz_' + w] = 3000.
            row['exp_CmpT_' + w + 'i'] = 10.
        rows.append(row)
    rows[2].update(exp_X_L1=101., exp_X_R1=100.6, exp_Az_SM=2.)
    rows[3].update(exp_X_L1=101.2, exp_X_R1=101.2, exp_Az_SM=1.)
    rows[4].update(exp_X_L1=104., exp_X_R1=104., exp_X_L2=102.,
                   exp_X_R2=102.5, exp_Az_SM=3.)
    path = tmp_path / 'native.csv'
    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    geometry = SimpleNamespace(leading_edge_m=101.1, trailing_edge_m=101.9,
                               depth_m=.06, length_m=.8, width_m=1.8, edge_transition_m=.05)
    case = SimpleNamespace(case_id='wide', geometry=geometry)
    control = SimpleNamespace(speed=3./3.6, limit=250., abort_reason=None)
    result = m.summarize(path, case, control, {})
    assert result['az_peak_m_s2'] == pytest.approx(3. * 9.80665)
