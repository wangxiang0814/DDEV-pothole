from pathlib import Path
import re


def test_mf52_conversion_preserves_coefficients_and_explicit_dimensions():
    from ddevsim.strategies.shallow_press.tire_comparison import make_mf52_tir
    from ddevsim.vehicle_profiles.tuned_ii import DEFAULT_PROFILE
    source=(DEFAULT_PROFILE.source_dir/'run_all.par').read_text(encoding='utf-8')
    text,record=make_mf52_tir(source)
    assert 'FITTYP = 6\n' in text
    assert 'UNLOADED_RADIUS = 0.263\n' in text
    assert 'VERTICAL_STIFFNESS = 220000\n' in text
    assert 'IYY = 0.4\n' in text
    assert record['is_same_vehicle_identity'] is False
    assert record['source_coefficient_count']>100
    assert 'PCX1 = 1.62\n' in text
    assert 'PTX1 = 1.95\n' in text
    assert record['adopted'] is False


def test_external_paths_are_windows_absolute_and_replacement_is_literal():
    from ddevsim.strategies.shallow_press.tire_comparison import install_mf52_comparison
    from ddevsim.vehicle_profiles.tuned_ii import DEFAULT_PROFILE
    source=(DEFAULT_PROFILE.source_dir/'run_all.par').read_text(encoding='utf-8')
    changed=install_mf52_comparison(source,Path('C:/Temp/tire.tir'),Path('F:/TruckSim/Prog'))
    assert changed.count('DELFT_TYRE_DATA C:\\Temp\\tire.tir\n')==4
    assert changed.count('OPT_TIRE_MODEL 8\n')==4
    assert 'BEGIN_PACEJKA_52_DATA' not in changed


def test_research_adoption_is_explicit_and_keeps_historical_comparison_record():
    from ddevsim.strategies.shallow_press.tire_comparison import research_baseline_metadata
    original=dict(modification='MF_TYRE_52_EXTERNAL_COMPARISON',adopted=False)
    selected=research_baseline_metadata(original)
    assert selected['adopted'] is True
    assert selected['baseline_id']=='expert2_mf52_research_20261024'
    assert selected['adoption_scope']=='EXPERT2_RESEARCH_ONLY'
    assert not original['adopted']


def test_worker_defaults_to_user_adopted_research_model_and_keeps_internal_option():
    import subprocess,sys,json
    root=Path(__file__).resolve().parents[2]
    base=[sys.executable,str(root/'scripts/run_shallow_preview.py'),'--case','right_track',
          '--depth-m','.04','--run-id','adoption_preflight','--dry-run']
    response=subprocess.run(base,cwd=str(root),capture_output=True,check=True)
    assert json.loads(response.stdout)['tire_model']=='mf52_research'
    response=subprocess.run(base+['--tire-model','internal'],cwd=str(root),capture_output=True,check=True)
    assert json.loads(response.stdout)['tire_model']=='internal'
