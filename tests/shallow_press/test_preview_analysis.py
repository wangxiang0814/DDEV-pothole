from pathlib import Path
import hashlib
import json
import runpy
import pytest


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
