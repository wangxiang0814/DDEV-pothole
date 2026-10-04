import hashlib
import json
import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))


def test_only_environment_scalars_transfer_with_source_hash(tmp_path,monkeypatch):
    import ddevsim.static_wheel_lift.environment_gain_transfer as module
    source=tmp_path/'source.par';target=tmp_path/'target.par'
    text='MASS 1000\nMU_ROAD_CONSTANT 0.6\nSSTART 99.4\nSSTART(1) 101.1\nSSTART 99.4\n'
    source.write_text(text);target.write_text(text.replace('0.6','0.7').replace('99.4','99.25'))
    seen=[]
    monkeypatch.setattr(module,'validated_support_models',lambda b,model_sha256:(seen.append(model_sha256) or {'valid':True}))
    models,proof=module.validated_environment_models({'source_model_sha256':hashlib.sha256(source.read_bytes()).hexdigest()},source,target)
    assert models=={'valid':True}
    assert seen==[hashlib.sha256(source.read_bytes()).hexdigest()]
    assert proof['target_friction']==.7 and proof['start_station_delta_m']==pytest.approx(-.15)
    assert proof['target_model_sha256']==hashlib.sha256(target.read_bytes()).hexdigest()


@pytest.mark.parametrize('change',['mass','pit','inconsistent_start','nan','missing_mu','large_delta'])
def test_transfer_rejects_vehicle_geometry_or_invalid_environment(tmp_path,monkeypatch,change):
    import ddevsim.static_wheel_lift.environment_gain_transfer as module
    source=tmp_path/'source.par';target=tmp_path/'target.par'
    text='MASS 1000\nMU_ROAD_CONSTANT 0.6\nSSTART 99.4\nPIT_WIDTH 0.9\nSSTART 99.4\n'
    source.write_text(text)
    variants={'mass':text.replace('1000','999'),'pit':text.replace('0.9','1.1'),
        'inconsistent_start':text.replace('SSTART 99.4','SSTART 99.3',1),
        'nan':text.replace('MU_ROAD_CONSTANT 0.6','MU_ROAD_CONSTANT nan'),
        'missing_mu':text.replace('MU_ROAD_CONSTANT 0.6\n',''),
        'large_delta':text.replace('SSTART 99.4','SSTART 95.4')}
    target.write_text(variants[change])
    monkeypatch.setattr(module,'validated_support_models',lambda *a,**k:{})
    with pytest.raises(ValueError):module.validated_environment_models({'source_model_sha256':hashlib.sha256(source.read_bytes()).hexdigest()},source,target)


@pytest.mark.parametrize('damage', ['source_hash', 'mode_shape', 'nonfinite'])
def test_environment_transfer_keeps_real_identification_validation(tmp_path, damage):
    from ddevsim.static_wheel_lift.environment_gain_transfer import validated_environment_models
    root = Path(__file__).resolve().parents[1]
    bundle = json.loads((root / 'evidence/static_fr_ii/support_gains_mu06_close_20261004.json').read_text(encoding='utf-8'))
    source = tmp_path / 'source.par'
    target = tmp_path / 'target.par'
    source.write_text('MU_ROAD_CONSTANT 0.6\nSSTART 99.4\nSSTART 99.4\n')
    target.write_text('MU_ROAD_CONSTANT 0.7\nSSTART 99.3\nSSTART 99.3\n')
    bundle['source_model_sha256'] = hashlib.sha256(source.read_bytes()).hexdigest()
    for mode in bundle['modes'].values():
        mode['source_model_sha256'] = bundle['source_model_sha256']
    # An intact control must pass before corrupting one matrix or the source hash.
    validated_environment_models(bundle, source, target)
    if damage == 'source_hash':
        bundle['source_model_sha256'] = '0' * 64
    elif damage == 'mode_shape':
        bundle['modes']['FR']['gains']['wheel_height_m'] = [[0.]]
    else:
        bundle['modes']['RR']['gains']['travel_m'][0][0] = float('nan')
    with pytest.raises(ValueError):
        validated_environment_models(bundle, source, target)
