from types import SimpleNamespace
from pathlib import Path

from ddevsim import native_video


def test_vsrap_packing_resolves_vehicle_assets_from_resource_directory(tmp_path, monkeypatch):
    resources=tmp_path/'Resources'
    shape=resources/'animator/3d_shape_files/vehicle.obj'
    shape.parent.mkdir(parents=True)
    shape.write_text('vehicle geometry')
    stage=tmp_path/'isolated_run';stage.mkdir()
    animator=stage/'animator.par';animator.write_text('isolated history')
    package=stage/'playback.vsrap'
    monkeypatch.setattr(native_video,'default_workdir',lambda: resources)
    def pack(args, **kwargs):
        if (Path(kwargs['cwd'])/'animator/3d_shape_files/vehicle.obj').is_file():
            package.write_bytes(b'complete scene')
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(native_video.subprocess,'run',pack)
    result=native_video.create_vsrap(animator,package)
    assert result['created'], 'packing from history cwd loses relative vehicle assets'


def test_export_default_forwards_resource_workdir(tmp_path, monkeypatch):
    resources = tmp_path / 'Resources'
    history = SimpleNamespace(basename='run', as_dict=lambda: {})
    monkeypatch.setattr(native_video, 'default_workdir', lambda: resources)
    monkeypatch.setattr(native_video, 'find_history', lambda _: history)
    monkeypatch.setattr(native_video, 'stage_history', lambda *_, **kwargs: history)
    monkeypatch.setattr(native_video, 'write_animator_par', lambda *_: tmp_path / 'animator.par')
    monkeypatch.setattr(native_video, 'manual_export_steps', lambda *_: [])
    calls = []
    def record(*args, **kwargs):
        calls.append(kwargs['workdir'])
        return {}
    monkeypatch.setattr(native_video, 'create_vsrap', record)
    monkeypatch.setattr(native_video, 'launch_visualizer', record)
    native_video.export_native_video(tmp_path, tmp_path / 'stage',
                                    create_vsrap_package=True, ascii_stage=False)
    assert calls == [resources, resources]
