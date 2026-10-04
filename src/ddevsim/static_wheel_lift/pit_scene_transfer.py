"""Explicit gain reuse only for a regenerated pit width/depth, with full-file proof."""
import hashlib
import re
from dataclasses import replace
from ddevsim.pothole_case import _procedure_block
from .system_identification import validated_support_models


def replace_pit_geometry_parameters(content, variant):
    generated = _procedure_block(variant)
    pattern = r'(?m)^ROAD_DZ_CARPET 2D_LINEAR\s*\n(?:.*\n)*?ENDTABLE'
    table = re.search(pattern, generated)
    if table is None:
        raise ValueError('generated pit has no physical road grid')
    content, count = re.subn(pattern, lambda _: table.group(), content)
    if count != 1:
        raise ValueError('expected one physical road grid')
    start = r'ENTER_PARSFILE Roads\Shapes\DDEV_single_wheel_pothole.par'
    end = r'EXIT_PARSFILE Roads\Shapes\DDEV_single_wheel_pothole.par'
    for text in (content, generated):
        if text.count(start) != 1 or text.count(end) != 1:
            raise ValueError('expected one pothole visual shape block')
    return (content[:content.index(start)] +
            generated[generated.index(start):generated.index(end)+len(end)] +
            content[content.index(end)+len(end):])


def validated_pit_scene_models(bundle, reference, target, variant):
    source_hash = hashlib.sha256(reference.read_bytes()).hexdigest()
    models = validated_support_models(bundle, model_sha256=source_hash)
    content = reference.read_text(encoding='utf-8')
    pattern = r'(?m)^ROAD_DZ_CARPET 2D_LINEAR\s*\n(?:.*\n)*?ENDTABLE'
    source_grid = re.search(pattern, content)
    if source_grid is None:
        raise ValueError('missing reference road grid')
    rows = [[float(v) for v in line.split(',')] for line in source_grid.group().splitlines()[1:-1]]
    width = rows[0][-2] - rows[0][2]
    depth = -min(v for row in rows[1:] for v in row[1:])
    original = replace(variant, width_m=width, depth_m=depth)
    expected_source = _procedure_block(original)
    if source_grid.group() != re.search(pattern, expected_source).group():
        raise ValueError('pit longitudinal geometry or other grid parameters differ')
    start = r'ENTER_PARSFILE Roads\Shapes\DDEV_single_wheel_pothole.par'
    end = r'EXIT_PARSFILE Roads\Shapes\DDEV_single_wheel_pothole.par'
    def shape(text):
        if text.count(start) != 1 or text.count(end) != 1:
            raise ValueError('expected one reference pothole visual shape')
        return text[text.index(start):text.index(end)+len(end)]
    if shape(content) != shape(expected_source):
        raise ValueError('reference pit display differs beyond width/depth')
    expected = replace_pit_geometry_parameters(content, variant)
    if expected != target.read_text(encoding='utf-8'):
        raise ValueError('model parameters changed outside pit width/depth')
    return models, dict(identified_model_sha256=source_hash,
        target_model_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
        kind='explicit pit width/depth transfer; not new identification',
        verification='full target parameter text equals regenerated reference',
        width_m=variant.width_m, depth_m=variant.depth_m,
        scope='same vehicle, start, friction, interfaces and control timing; flat-ground local gains reused')
