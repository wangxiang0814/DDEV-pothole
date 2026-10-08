"""Shared physical/visual pit parameter replacement, with strict block checks."""
import re
from ddevsim.pothole_case import _procedure_block

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


