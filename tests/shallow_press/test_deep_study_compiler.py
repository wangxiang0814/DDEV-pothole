import hashlib
import json
from pathlib import Path
import runpy
import sys
import pytest


@pytest.fixture
def compiler_fixture(tmp_path,monkeypatch):
    root=Path(__file__).resolve().parents[2]
    monkeypatch.setattr(sys,'path',[str(root/'scripts')]+sys.path)
    module=runpy.run_path(str(root/'scripts/analyze_shallow_deep_study.py'))
    loader=module['load_verified'];globals_=loader.__globals__
    monkeypatch.setitem(globals_,'ROOT',tmp_path)
    digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    dump=lambda p,v:p.write_text(json.dumps(v),encoding='utf-8')
    folder=tmp_path/'evidence';folder.mkdir()
    config=tmp_path/'config.json';dump(config,{})
    case=dict(case='wide',run_id='trial',kind='validation',depth_m=.1,crossing_kph=1,args=['identity'])
    plan=dict(scope='TEST',config_path='config.json',config_sha256=digest(config),
        source_sha256={'x.py':'source-hash'},cases=[case])
    dump(folder/'frozen_plan.json',plan)
    run=tmp_path/'runs/shallow_press/wide/trial';run.mkdir(parents=True)
    result=dict(status='FAIL',metrics={},criteria={'slip':False},acceptance_pass=False,
        scene={'depth_m':.1},config={'crossing_kph':1},plant={},exit_progress=[],chassis_diagnostic={})
    dump(run/'result.json',result)
    dump(run/'matrix_identity.json',dict(plan_sha256=digest(folder/'frozen_plan.json'),args=case['args']))
    monkeypatch.setitem(globals_,'verified_run',lambda p:(result,{'source_sha256':plan['source_sha256']}))
    row={k:case[k] for k in ('case','run_id','kind','depth_m','crossing_kph')}
    row.update(status='FAIL',metrics={},failed_criteria=['slip'],result_sha256=digest(run/'result.json'))
    data=dict(scope='TEST',plan_sha256=digest(folder/'frozen_plan.json'),
        source_sha256=plan['source_sha256'],config_sha256=plan['config_sha256'],
        count=1,passing=0,full_acceptance=False,rows=[row])
    output=folder/'verified_analysis.json';dump(output,data)
    return loader,output,data,dump,plan


def test_verified_compiler_accepts_intact_cohort(compiler_fixture):
    loader,path,data,_,_=compiler_fixture
    assert loader(path)==data


@pytest.mark.parametrize('change',['false_pass','cleared_failures','wrong_kind','missing_row','plan_drift'])
def test_compiler_rejects_forged_or_partial_derived_cohort(compiler_fixture,change):
    loader,path,data,dump,plan=compiler_fixture
    if change=='false_pass':data['rows'][0]['status']='SIMULATION_PASS'
    elif change=='cleared_failures':data['rows'][0]['failed_criteria']=[]
    elif change=='wrong_kind':data['rows'][0]['kind']='different_experiment'
    elif change=='missing_row':data['rows']=[]
    elif change=='plan_drift':data['plan_sha256']='not-the-frozen-plan'
    dump(path,data)
    with pytest.raises(ValueError):loader(path)
