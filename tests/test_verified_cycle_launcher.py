import json
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))


def test_named_case_merges_speed_override_and_keeps_one_option():
    from run_verified_right_side import build_command
    m={'common':{'rear-target-speed-kph':3.6,'measurement-noise':True},
       'cases':{'slow':{'options':{'rear-target-speed-kph':2.6}}}}
    cmd=build_command(m,'slow',Path('new_run'))
    assert cmd.count('--rear-target-speed-kph')==1
    assert cmd[cmd.index('--rear-target-speed-kph')+1]=='2.6'
    assert '--measurement-noise' in cmd


def test_boundary_requires_explicit_gain_reference_and_unknown_case_rejected():
    from run_verified_right_side import build_command
    m={'common':{},'cases':{'boundary':{'requires_reference_model':True,'options':{}}}}
    with pytest.raises(ValueError):build_command(m,'unknown',Path('new_run'))
    with pytest.raises(ValueError):build_command(m,'boundary',Path('new_run'))
    cmd=build_command(m,'boundary',Path('new_run'),Path('reference.par'))
    assert '--support-allocation-reference-model' in cmd


def test_checked_in_cases_select_matching_gain_bundle():
    from run_verified_right_side import build_command, MANIFEST
    m=json.loads(MANIFEST.read_text())
    for name in m['cases']:
        cmd=build_command(m,name,Path('new_run'),Path('reference.par'))
        gain=cmd[cmd.index('--support-allocation-gains')+1]
        assert ('mu06' in gain)==('--road-friction' in cmd)


def test_environment_reference_is_explicit_and_cannot_mix_pit_transfer():
    from run_verified_right_side import build_command
    m={'common':{},'cases':{'mu':{'requires_environment_reference':True,'options':{}}}}
    with pytest.raises(ValueError):build_command(m,'mu',Path('new_run'))
    cmd=build_command(m,'mu',Path('new_run'),Path('reference.par'))
    assert '--support-allocation-environment-reference' in cmd
    assert '--support-allocation-reference-model' not in cmd
    m['cases']['mu']['requires_reference_model']=True
    with pytest.raises(ValueError):build_command(m,'mu',Path('new_run'),Path('reference.par'))
