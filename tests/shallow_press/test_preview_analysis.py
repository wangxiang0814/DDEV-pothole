from pathlib import Path
import hashlib
import json
import runpy
import pytest


def test_source_hash_comparison_accepts_windows_separators_but_rejects_aliases():
    import runpy
    from pathlib import Path
    normalize=runpy.run_path(str(Path(__file__).resolve().parents[2]/'scripts/analyze_shallow_preview.py'))['source_digest_map']
    assert normalize({'src/a.py':'one'})==normalize({'src\\a.py':'one'})
    assert normalize({'src/a.py':'one'})!=normalize({'src\\a.py':'two'})
    with pytest.raises(ValueError,match='alias'):normalize({'src/a.py':'one','src\\a.py':'two'})
    assert normalize({'SRC/A.py':'one'})==normalize({'src/a.py':'one'})
    with pytest.raises(ValueError,match='alias'):normalize({'SRC/A.py':'one','src/a.py':'one'})
    for name in ('src/./a.py','src/x/../a.py','src//a.py','/src/a.py','C:/src/a.py'):
        with pytest.raises(ValueError,match='canonical'):normalize({name:'one'})


def test_unadopted_model_cannot_be_counted_as_baseline_controller_pass():
    from ddevsim.strategies.shallow_press.assessment import model_diagnostic_result
    result=dict(simulation_pass=True,acceptance_pass=False,status='SIMULATION_PASS',criteria={'motion':True})
    diagnostic=model_diagnostic_result(result)
    assert diagnostic['model_diagnostic_motion_pass']
    assert diagnostic['simulation_pass'] is False and diagnostic['acceptance_pass'] is False
    assert diagnostic['status']=='MODEL_DIAGNOSTIC_ONLY'
    assert diagnostic['criteria']==result['criteria']


def test_analysis_rejects_edited_native_data_and_portable_plant_identity(tmp_path):
    module=runpy.run_path(str(Path(__file__).resolve().parents[2]/'scripts/analyze_shallow_preview.py'))
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    (tmp_path/'model').mkdir();(tmp_path/'model/run_all.par').write_text('plant')
    (tmp_path/'native_1ms.csv').write_text('data');(tmp_path/'input_provenance.json').write_text('{}')
    result=dict(native_csv_sha256=sha(tmp_path/'native_1ms.csv'),model_sha256=sha(tmp_path/'model/run_all.par'),
        input_provenance_sha256=sha(tmp_path/'input_provenance.json'),plant={'variant':'comparison'},native={'reached_configured_stop':True})
    (tmp_path/'result.json').write_text(json.dumps(result))
    manifest=dict(result_sha256=sha(tmp_path/'result.json'),run_status='COMPLETED',plant=result['plant'],
        input_provenance_sha256=result['input_provenance_sha256'],final_model_sha256=result['model_sha256'],
        generated_assets={},source_sha256={})
    (tmp_path/'run_manifest.json').write_text(json.dumps(manifest))
    module['verified_run'](tmp_path)
    (tmp_path/'native_1ms.csv').write_text('edited')
    with pytest.raises(ValueError,match='native'):module['verified_run'](tmp_path)
    (tmp_path/'native_1ms.csv').write_text('data');manifest['plant']={'variant':'parent'}
    (tmp_path/'run_manifest.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError,match='plant'):module['verified_run'](tmp_path)
