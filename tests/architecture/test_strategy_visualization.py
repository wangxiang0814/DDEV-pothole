import hashlib
import json
from pathlib import Path
import struct

import pytest

from ddevsim.native_video import find_history, resolve_stage_dir, stage_history


def history(run, mesh_text):
    output = run / 'model/output'
    output.mkdir(parents=True)
    (output / 'case.vs').write_text(json.dumps({'VsChannelGroup': {
        'XStep': .01, 'Channels': [{'Name Aliases': ['Xo'], 'Units': 'm'}]}}))
    (output / 'case.vsb').write_bytes(struct.pack('<6i2f', 1, 0, 0, 0, 4, 1, 0., 0.))
    (output / 'case_all.par').write_bytes(
        b'PARSFILE\nadd_obj Animator\\3D_Shape_Files\\DDEV\\DDEV_pothole_scene.obj\nEND\n')
    assets = run / 'model/visual_assets'
    assets.mkdir()
    (assets / 'DDEV_pothole_scene.obj').write_text(mesh_text)
    (assets / 'DDEV_pothole_scene.mtl').write_text('newmtl Pit\n')
    return output


def test_nested_strategy_runs_resolve_history_and_select_local_mesh(tmp_path):
    first = tmp_path / 'runs/wheel_lift/case/same'
    second = tmp_path / 'runs/shallow_press/case/same'
    history(first, 'v 0 0 -0.2\n')
    history(second, 'v 0 0 -0.05\n')
    for run, depth, stage in ((first, '-0.2', tmp_path / 'stage_a'),
                              (second, '-0.05', tmp_path / 'stage_b')):
        source = find_history(run)
        original = source.par.read_bytes()
        staged = stage_history(source, stage)
        mesh = stage / 'visual_assets/DDEV_pothole_scene.obj'
        assert mesh.is_file() and depth in mesh.read_text()
        assert str(mesh.resolve()).replace('\\', '/').encode('ascii') in staged.par.read_bytes()
        assert source.par.read_bytes() == original
        assert staged.vs.read_bytes() == source.vs.read_bytes()
        assert staged.vsb.read_bytes() == source.vsb.read_bytes()


def test_equal_case_names_get_distinct_ascii_stages():
    first, _ = resolve_stage_dir(Path('F:/场景/wheel_lift/same/native_video'))
    second, _ = resolve_stage_dir(Path('F:/场景/shallow_press/same/native_video'))
    assert first != second
    str(first).encode('ascii')
    str(second).encode('ascii')
