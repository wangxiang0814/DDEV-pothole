import json
import struct
import subprocess
import sys
from pathlib import Path

import pytest

from ddevsim.native_video import find_history, stage_history


DRIVER = (
    b'ENTER_PARSFILE Animator\\STL\\driver_shape.par\r\n'
    b'#FullDataName Driver\r\n'
    b'add_obj Animator\\Trucks\\Drivers\\Male\\male.obj\r\n'
    b'SET_COLOR 1 1 1\r\n'
    b'EXIT_PARSFILE Animator\\STL\\driver_shape.par\r\n')
OTHER = b'OPT_DRIVER_MODEL 0\r\nMASS 1360\r\n# ANSI text: \xff\r\n'


def history_at(path, content):
    path.mkdir()
    (path / 'case.vs').write_text(json.dumps({'VsChannelGroup': {
        'XStep': .01, 'Channels': [{'Name Aliases': ['Xo'], 'Units': 'm'}]}}))
    (path / 'case.vsb').write_bytes(struct.pack('<6i2f', 1, 0, 0, 0, 4, 1, 0., 0.))
    (path / 'case_all.par').write_bytes(content)
    return find_history(path)


def test_default_hides_only_driver_shape_in_separate_display_copy(tmp_path):
    original = OTHER + DRIVER + b'add_obj Animator\\Trucks\\cab.obj\r\n'
    source = history_at(tmp_path / 'source', original)
    staged = stage_history(source, tmp_path / 'stage')
    assert staged.par.name == 'case_display.par'
    assert staged.par.read_bytes() == original.replace(DRIVER, b'')
    assert source.par.read_bytes() == original
    assert (staged.par.parent / 'case_all.par').read_bytes() == original
    assert staged.vs.read_bytes() == source.vs.read_bytes()
    assert staged.vsb.read_bytes() == source.vsb.read_bytes()


def test_show_driver_restores_from_source_on_repeated_staging(tmp_path):
    source = history_at(tmp_path / 'source', OTHER + DRIVER)
    stage_history(source, tmp_path / 'stage')
    shown = stage_history(source, tmp_path / 'stage', show_driver=True)
    assert shown.par.read_bytes() == source.par.read_bytes()


def test_staging_in_source_directory_keeps_original_model_unchanged(tmp_path):
    source = history_at(tmp_path / 'source', OTHER + DRIVER)
    staged = stage_history(source, source.par.parent)
    assert source.par.read_bytes() == OTHER + DRIVER
    assert staged.par.read_bytes() == OTHER


@pytest.mark.parametrize('content', [
    DRIVER.replace(b'SET_COLOR', b'add_obj Animator\\cab.obj\r\nSET_COLOR'),
    DRIVER.split(b'EXIT_PARSFILE')[0],
])
def test_ambiguous_or_unclosed_driver_shape_fails_without_editing_source(tmp_path, content):
    source = history_at(tmp_path / 'source', OTHER + content)
    with pytest.raises(ValueError, match='driver'):
        stage_history(source, tmp_path / 'stage')
    assert source.par.read_bytes() == OTHER + content


def test_driver_control_and_driveshaft_geometry_are_kept(tmp_path):
    content = OTHER + b'add_obj Animator\\DriveShaft\\shaft.obj\r\n'
    source = history_at(tmp_path / 'source', content)
    assert stage_history(source, tmp_path / 'stage').par.read_bytes() == content


def test_legacy_recording_uses_hidden_display_copy(monkeypatch, tmp_path):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
    import record_pothole_video as legacy
    run = tmp_path / 'run'
    run.mkdir()
    original = history_at(run / 'native', OTHER + DRIVER)
    monkeypatch.setattr(legacy, 'HISTORY', 'case')
    monkeypatch.setattr(legacy, 'STAGE', tmp_path / 'stage')
    staged = legacy.stage_history(run)
    animator = legacy.animator_par(staged)
    assert staged.par.read_bytes() == OTHER
    assert str(staged.par.resolve()) in animator.read_text(encoding='ascii')
    assert original.par.read_bytes() == OTHER + DRIVER


def test_powershell_staging_recipe_uses_shared_visibility_filter(tmp_path):
    root = Path(__file__).resolve().parents[1]
    script = (root / 'scripts/open_pothole_visualizer.ps1').read_text(encoding='utf-8')
    recipe = script.split("$stageCode = @'", 1)[1].split("'@", 1)[0]
    source = history_at(tmp_path / 'source', OTHER + DRIVER)
    stage = tmp_path / 'stage'
    subprocess.run([sys.executable, '-c', recipe, str(root / 'src'),
                    str(source.vs), str(stage)], check=True)
    assert (stage / 'case_display.par').read_bytes() == OTHER
    assert str(stage / 'case_display.par') in (stage / 'animator.par').read_text(encoding='ascii')
    assert source.par.read_bytes() == OTHER + DRIVER
