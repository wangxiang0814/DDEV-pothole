import json
import struct

import pytest

from ddevsim.native_video import DEFAULT_RESOURCE_ROOT, find_history, stage_history


def test_staged_box_preserves_rear_parts_and_original_history(tmp_path):
    from ddevsim.vehicle_appearance import CARGO_BED

    def shape(name, content):
        return (f'ENTER_PARSFILE Animator\\STL\\{name}.par\r\n'
                f'{content}EXIT_PARSFILE Animator\\STL\\{name}.par\r\n').encode('ascii')

    bed = shape('bed', f'add_obj {CARGO_BED.as_posix()}\r\n'
                       'SET_OFFSET_X -0.58\r\nUSE_RUN_COLOR\r\nSET_COLOR 1 0.8 0\r\n')
    details = shape('details', f'add_obj {CARGO_BED.with_name("bed_details.obj").as_posix()}\r\n'
                               'SET_OFFSET_X -0.58\r\nSET_OFFSET_Z 0.263\r\n')
    (tmp_path / 'case_all.par').write_bytes(bed + details)
    (tmp_path / 'case.vs').write_text(json.dumps({'VsChannelGroup': {
        'XStep': .01, 'Channels': [{'Name Aliases': ['Xo']}]}}))
    (tmp_path / 'case.vsb').write_bytes(struct.pack('<6if', 1, 0, 0, 0, 4, 1, 0.))
    source = find_history(tmp_path)
    if not (DEFAULT_RESOURCE_ROOT / CARGO_BED).is_file():
        pytest.skip('TruckSim stock meshes are installed locally')
    staged = stage_history(source, tmp_path / 'stage', cargo_box=True)
    display = staged.par.read_bytes()
    mesh = staged.par.parent / 'ddev_stock_box.obj'
    assert mesh.is_file()
    assert 'Walls_TruckSim_di.dds' in mesh.with_suffix('.mtl').read_text()
    assert details in display and b'SET_OFFSET_X -0.58' in display
    assert b'USE_RUN_COLOR' not in display and b'SET_COLOR 1 1 1' in display
    assert source.par.read_bytes() == bed + details
    assert staged.vs.read_bytes() == source.vs.read_bytes()
    assert staged.vsb.read_bytes() == source.vsb.read_bytes()
