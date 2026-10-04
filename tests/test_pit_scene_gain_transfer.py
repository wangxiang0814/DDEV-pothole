import hashlib
from pathlib import Path
import numpy as np
import pytest
from ddevsim.pothole_case import corner_module_scenario, _procedure_block
from ddevsim.static_wheel_lift.pit_scene_transfer import (
    replace_pit_geometry_parameters, validated_pit_scene_models)


def source_and_bundle(tmp_path):
    text = 'VEHICLE_CODE i_i\nSPRUNG_MASS 1360\nSSTART 99.1\nROAD_MU 0.7\nTSTOP 180\n' + _procedure_block(corner_module_scenario())
    path = tmp_path/'reference.par'
    path.write_text(text, encoding='utf-8')
    sha = hashlib.sha256(path.read_bytes()).hexdigest()
    shapes = {'Fz_n':(4,4),'CoM_xy_m':(2,4),'ZMP_xy_m':(2,4),
        'attitude_rad':(2,4),'travel_m':(4,4),'wheel_height_m':(4,4)}
    bundle = {'modes':{m:{'status':'PASS','mode':m,'source_model_sha256':sha,
        'gains':{k:np.zeros(s).tolist() for k,s in shapes.items()}}
        for m in ('FOUR_CONTACT','FR','RR')}}
    variant = corner_module_scenario(width_m=1.1,depth_m=.25)
    target = tmp_path/'target.par'
    target.write_text(replace_pit_geometry_parameters(text,variant),encoding='utf-8')
    return path,target,bundle,variant


def test_width_depth_transfer_keeps_identification_hash_and_records_target(tmp_path):
    source,target,bundle,variant=source_and_bundle(tmp_path)
    models,proof=validated_pit_scene_models(bundle,source,target,variant)
    assert models is bundle['modes']
    assert proof['identified_model_sha256']==hashlib.sha256(source.read_bytes()).hexdigest()
    assert proof['target_model_sha256']==hashlib.sha256(target.read_bytes()).hexdigest()
    assert proof['identified_model_sha256']!=proof['target_model_sha256']
    assert models['FR']['source_model_sha256']==proof['identified_model_sha256']


@pytest.mark.parametrize('old,new', [('SPRUNG_MASS 1360','SPRUNG_MASS 1000'),
    ('SSTART 99.1','SSTART 99.4'), ('VEHICLE_CODE i_i','VEHICLE_CODE s_s'),
    ('ROAD_MU 0.7','ROAD_MU 0.6'), ('TSTOP 180','TSTOP 120')])
def test_any_non_geometry_change_rejected(tmp_path,old,new):
    source,target,bundle,variant=source_and_bundle(tmp_path)
    target.write_text(target.read_text().replace(old,new),encoding='utf-8')
    with pytest.raises(ValueError,match='outside pit width/depth'):
        validated_pit_scene_models(bundle,source,target,variant)


def test_reference_must_be_actual_identified_file(tmp_path):
    source,target,bundle,variant=source_and_bundle(tmp_path)
    source.write_text(source.read_text()+'\n! altered source\n',encoding='utf-8')
    with pytest.raises(ValueError,match='identified model mismatch'):
        validated_pit_scene_models(bundle,source,target,variant)


def test_incomplete_gain_rejected_even_when_only_geometry_changed(tmp_path):
    source,target,bundle,variant=source_and_bundle(tmp_path)
    del bundle['modes']['RR']['gains']['travel_m']
    with pytest.raises(ValueError,match='missing support model'):
        validated_pit_scene_models(bundle,source,target,variant)


def test_scene_replacement_rejects_duplicate_or_missing_grid(tmp_path):
    source,target,bundle,variant=source_and_bundle(tmp_path)
    for text in (source.read_text().replace('ROAD_DZ_CARPET 2D_LINEAR','INVALID_GRID'),
                 source.read_text()+source.read_text()):
        with pytest.raises(ValueError):
            replace_pit_geometry_parameters(text,variant)


def test_transfer_cannot_change_pit_length_or_station_even_with_valid_gain_hash(tmp_path):
    source,target,bundle,variant=source_and_bundle(tmp_path)
    original=source.read_text()
    # A genuine different reference geometry must not become the default pit.
    source.write_text(original.replace('101.85,','102.05,'),encoding='utf-8')
    sha=hashlib.sha256(source.read_bytes()).hexdigest()
    for mode in bundle['modes'].values(): mode['source_model_sha256']=sha
    with pytest.raises(ValueError,match='pit longitudinal geometry'):
        validated_pit_scene_models(bundle,source,target,variant)
