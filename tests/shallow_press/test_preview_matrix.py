import hashlib
import json
from pathlib import Path
import runpy
import pytest


def entry():
    return runpy.run_path(str(Path(__file__).resolve().parents[2]/'scripts/run_shallow_preview_matrix.py'))


def test_frozen_matrix_rejects_source_drift_and_duplicate_output(tmp_path):
    module=entry();source=tmp_path/'controller.py';source.write_text('frozen')
    plan=dict(source_sha256={'controller.py':hashlib.sha256(source.read_bytes()).hexdigest()},
              config_path='controller.py',config_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
              cases=[dict(case='wide',run_id='one',args=['--case','wide','--run-id','one'])])
    module['validate_plan'](plan,tmp_path)
    plan['cases']*=2
    with pytest.raises(ValueError,match='duplicate'):module['validate_plan'](plan,tmp_path)
    plan['cases']=plan['cases'][:1];source.write_text('retuned')
    with pytest.raises(ValueError,match='drift'):module['validate_plan'](plan,tmp_path)


def test_resume_requires_exact_plan_and_completed_immutable_result(tmp_path):
    module=entry();result=tmp_path/'result.json'
    (tmp_path/'model').mkdir();(tmp_path/'model/run_all.par').write_text('model')
    (tmp_path/'native_1ms.csv').write_text('native');(tmp_path/'input_provenance.json').write_text('{}')
    digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    record=dict(native_csv_sha256=digest(tmp_path/'native_1ms.csv'),model_sha256=digest(tmp_path/'model/run_all.par'),
        input_provenance_sha256=digest(tmp_path/'input_provenance.json'))
    original=json.dumps(record);result.write_text(original)
    manifest=tmp_path/'run_manifest.json'
    manifest.write_text(json.dumps(dict(run_status='COMPLETED',result_sha256=hashlib.sha256(result.read_bytes()).hexdigest())))
    marker=tmp_path/'matrix_identity.json';marker.write_text(json.dumps(dict(plan_sha256='one',args=['frozen'])))
    assert module['can_resume'](tmp_path,'one',['frozen'])
    assert not module['can_resume'](tmp_path,'two',['frozen'])
    assert not module['can_resume'](tmp_path,'one',['retuned'])
    result.write_text('{"edited":true}')
    assert not module['can_resume'](tmp_path,'one',['frozen'])
    result.write_text(original);(tmp_path/'native_1ms.csv').write_text('edited native')
    assert not module['can_resume'](tmp_path,'one',['frozen'])
