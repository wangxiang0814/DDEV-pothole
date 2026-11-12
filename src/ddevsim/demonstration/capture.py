"""Native historical rendering joined by simulation timestamps, never OS time."""
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time
import uuid

import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation

from ddevsim.native_video import find_history, stage_history, DEFAULT_VISUALIZER_64, default_workdir
from .cameras import NAMES, camera_par, validate_config
from .recording import TransitionReader
from .shared_buffer import SharedBuffer, synchronized_frames
from .mounts import verify_mounts


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def phase_rows(run):
    rows = []
    for name in ('approach_control_20ms.csv', 'front_control_20ms.csv', 'rear_control_20ms.csv'):
        path = run / name
        if path.exists():
            with path.open(encoding='utf-8', newline='') as src:
                rows.extend((float(r['time_s']), r['mode']) for r in csv.DictReader(src))
    path=run/'control_diagnostics.json'
    if path.exists():
        rows.extend((float(r['time_s']),r['phase']) for r in json.loads(path.read_text(encoding='utf-8')))
    return sorted(rows)


def episode_identity(run,result):
    manifest=run/'run_manifest.json'
    record=json.loads(manifest.read_text(encoding='utf-8')) if manifest.exists() else {}
    return dict(strategy_id=record.get('strategy_id',result.get('strategy_id','wheel_lift')),
                scene=result.get('scene',result.get('scenario')),plant=result.get('plant',record.get('plant')),
                capability=result.get('capability',record.get('capability')),
                input_provenance_sha256=result.get('input_provenance_sha256'))


def preview_grid(frames, path, label):
    canvas = Image.new('RGB', (960, 540), '#17212b')
    draw = ImageDraw.Draw(canvas)
    for i, (name, (_, rgb)) in enumerate(zip(NAMES, frames)):
        x, y = (i % 3)*320, (i//3)*265
        draw.text((x+8, y+4), name + '  ' + label, fill='white')
        canvas.paste(Image.fromarray(rgb).resize((320, 240)), (x, y+23))
    canvas.save(path)


def capture_episode(run_dir, config, *, capture_name='episode', preview_duration_s=None):
    """Replay after solver termination; fail quality separately from expert task."""
    cfg = validate_config(config)
    run = Path(run_dir).resolve()
    source = run / 'demonstration'
    reader = TransitionReader(source)
    target = source / capture_name
    if Path(capture_name).name != capture_name:
        raise ValueError('capture_name must be a single path component')
    target.mkdir(exist_ok=False)
    (target / 'images').mkdir()
    for name in NAMES:
        (target / 'images' / name).mkdir()
    (target / 'previews').mkdir()
    result = json.loads((run / 'result.json').read_text(encoding='utf-8'))
    history = find_history(run)
    original_hashes = {str(p): sha(p) for p in (history.vs, history.vsb, history.par)}
    native_header = json.loads(history.vs.read_text(encoding='utf-8'))['VsChannelGroup']
    import struct
    value_bytes = struct.unpack('<6i', history.vsb.read_bytes()[:24])[4]
    motions = np.memmap(history.vsb, dtype='<f%d' % value_bytes, mode='r', offset=24).reshape(-1, len(history.channels))
    native_times = float(native_header['XStart']) + np.arange(len(motions))*float(native_header['XStep'])
    render_end = min(reader.final_time, float(native_times[-1]))
    mounts = verify_mounts(cfg, history.par, default_workdir())
    if preview_duration_s is not None:
        if not math.isfinite(preview_duration_s) or preview_duration_s <= 0:
            raise ValueError('preview duration must be positive')
        render_end = min(render_end, float(reader.times[0])+preview_duration_s)
    pose_indices = [history.channels.index(n) for n in ('Xo', 'Yo', 'Zo', 'Yaw', 'Pitch', 'Roll_E')]
    phases = phase_rows(run)
    phase_times = np.array([x[0] for x in phases])
    prefix = 'DDEV_VLA_' + uuid.uuid4().hex[:12]
    stage = Path(tempfile.gettempdir()) / prefix
    staged = stage_history(history, stage)
    animator = stage / 'animator.par'
    animator.write_text(
        'PARSFILE\nSET_RUN_SLOT 0\nDATASET %s\nPARSFILE %s\n%sSIMTIME_LOOP_TYPE NONE\n'
        'SET_PLAYBACK_TIME_MULTIPLIER %.12g\nEND\n'
        % (staged.vs, staged.par, camera_par(cfg, prefix), cfg['playback_speed']), encoding='mbcs')
    # A dedicated named preference file: never change the user's normal viewer.
    config_path = Path(os.environ['LOCALAPPDATA']) / 'VS Visualizer/2019/Config' / (prefix+'.ini')
    config_path.parent.mkdir(parents=True, exist_ok=True)
    config_path.write_text(
        'TimerAutoStart=1\nTimerTimeMult=%s\nTimerRestoreMult=1\nTimerLoopEnable=0\n'
        'MaxFps=30\nRenderMode=0\nAllowRenderWarnings=0\nAudioEnable=0\n'
        'WindowFullScreen=0\nDynamicShadowQuality=1\n' % cfg['playback_speed'], encoding='ascii')
    meta = {
        'schema_version': 1, 'collection_status': 'RUNNING', 'trainable': False,
        'source_run': str(run), **episode_identity(run,result),
        'task_instruction': cfg['task_instruction'], 'expert_status': result.get('status'),
        'expert_criteria': result.get('criteria'),
        'config': cfg, 'source_hashes': original_hashes,
        'surface_mount_verification': mounts,
        'preview_only': preview_duration_s is not None,
        'provenance_hashes': {str(p.relative_to(run)): sha(p) for p in (
            source/'transitions.bin', source/'transitions.json', run/'result.json',
            run/'model/run_all.par', run/'control_config.json') if p.exists()},
        'capture_backend': 'VS Visualizer native history + VS Shared Buffer API v2',
        'capture_mode': 'post-run rendering; simulation-time joined',
        'state_source': reader.schema,
        'initial_state_si': reader.sample(float(reader.times[0]))['state_si'],
        'coordinate_systems': {
            'world_body_native': 'x forward, y left, z up; metres; body origin Xo/Yo/Zo',
            'body_extrinsics': 'translation followed by intrinsic yaw-pitch-roll, degrees',
            'camera_native': 'x optical forward, y left, z up',
            'camera_optical': 'x right=-native_y, y down=-native_z, z forward=native_x',
            'pixels': 'RGB uint8, top-left origin; integer pixel-centre coordinates',
            'distortion': 'ideal pinhole; zero distortion',
        },
        'timing': {'image_time': 'native mFrameSimTime',
                   'state': 'linear interpolation between solver-step sensor observations',
                   'action': 'actual input held for containing solver interval',
                   'render_pose': 'native history linear interpolation; original XStep retained',
                   'native_history_step_s': float(native_header['XStep']),
                   'requested_fps': cfg['fps']},
        'action_conventions': {
            'wheel_order': ['FL', 'FR', 'RL', 'RR'],
            'torque': 'IMP_MYUSM_* signed external wheel moment, N*m; native sign retained',
            'force': 'IMP_FS_* signed ADD spring-seat force, N; positive adds spring force; not a height target',
            'steering': 'IMP_STEER_SW steering-wheel angle, rad in SI record; not road-wheel angle; native deg sign retained',
            'reference': 'IO_MAPPING.md and repository unit/interface validation',
        },
    }
    meta_path = target / 'episode.json'
    def write_meta():
        meta_path.write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding='utf-8')
    write_meta()
    process, buffers = None, []
    sample_times, max_pose_error, saved_phases = [], 0., set()
    rejected_groups = 0
    try:
        process = subprocess.Popen(
            [str(DEFAULT_VISUALIZER_64), '-c', prefix, '-i', prefix,
             '-fs', '0', '-res', '640', '480', '-nosound', str(animator)],
            cwd=str(default_workdir()), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for name in NAMES:
            buffers.append(SharedBuffer(prefix+'_'+name, cfg['connection_timeout_s']))
        next_bin, last_time = 0, -1.
        deadline = time.monotonic()+cfg['frame_timeout_s']
        with (target/'frames.jsonl').open('x', encoding='utf-8') as dest:
            while True:
                if process.poll() is not None:
                    raise RuntimeError('native renderer exited before capture completed')
                first = buffers[0].read(after_time=last_time,
                                        minimum_time=min(next_bin/cfg['fps'], render_end))
                frames = ([first]+[b.read() for b in buffers[1:]] if first is not None
                          else [None]*5)
                if not synchronized_frames(frames, last_time):
                    rejected_groups += 1
                    if time.monotonic() > deadline:
                        raise TimeoutError('no new synchronized five-camera frame')
                    time.sleep(.002)
                    continue
                timestamp = frames[0][0]['time_s']
                if timestamp > reader.final_time:
                    break
                if timestamp < next_bin/cfg['fps'] and timestamp < render_end - 1e-8:
                    if time.monotonic() > deadline:
                        raise TimeoutError('native playback stopped before next sample')
                    time.sleep(.002)
                    continue
                sample = reader.sample(timestamp)
                pose = np.array([np.interp(timestamp, native_times, motions[:, i]) for i in pose_indices])
                body_rotation = Rotation.from_euler('ZYX', pose[3:], degrees=True)
                camera_records = {}
                for cam, (info, rgb) in zip(cfg['cameras'], frames):
                    if rgb.shape != (cam['height'], cam['width'], 3):
                        raise ValueError('render dimensions disagree with config')
                    expected = pose[:3] + body_rotation.apply(cam['position_m'])
                    error = float(np.linalg.norm(expected - [info['x'], info['y'], info['z']]))
                    max_pose_error = max(max_pose_error, error)
                    if error > .002:
                        raise ValueError('native camera is not on the specified body transform: '+cam['name'])
                    axis = np.array([info['ax'], info['ay'], info['az']])
                    angle = info['angle_rad']
                    if abs(angle) > 1e-9 and abs(np.linalg.norm(axis)-1) > 1e-5:
                        raise ValueError('invalid native camera orientation axis')
                    native_rotation = Rotation.from_rotvec(axis*angle)
                    expected_rotation = body_rotation * Rotation.from_euler(
                        'ZYX', cam['yaw_pitch_roll_deg'], degrees=True)
                    orientation_error = float((expected_rotation.inv()*native_rotation).magnitude())
                    if orientation_error > 1e-4:
                        raise ValueError('native camera orientation differs from configured body mount')
                    # Column vectors: optical x/right, y/down, z/forward in native camera axes.
                    optical_to_native = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
                    transform = np.eye(4)
                    transform[:3, :3] = native_rotation.as_matrix() @ optical_to_native
                    transform[:3, 3] = [info['x'], info['y'], info['z']]
                    relative = 'images/%s/%06d.png' % (cam['name'], len(sample_times))
                    Image.fromarray(rgb).save(target/relative, compress_level=1)
                    camera_records[cam['name']] = {
                        'image': relative, **info, 'body_mount_position_error_m': error,
                        'body_mount_orientation_error_rad': orientation_error,
                        'T_world_from_optical': transform.tolist(),
                    }
                idx = int(np.searchsorted(phase_times, timestamp, side='right'))-1
                phase = phases[idx][1] if idx >= 0 else 'INIT_SETTLE'
                sample.update(index=len(sample_times), cameras=camera_records, phase=phase)
                dest.write(json.dumps(sample, allow_nan=False)+'\n')
                if phase not in saved_phases:
                    preview_grid(frames, target/'previews'/('%s.png' % phase), 't=%.3fs'%timestamp)
                    saved_phases.add(phase)
                sample_times.append(timestamp)
                last_time = timestamp
                next_bin = math.floor(timestamp*cfg['fps'])+1
                deadline = time.monotonic()+cfg['frame_timeout_s']
                if len(sample_times) % 100 == 0:
                    print('VLA capture: %d groups, simulation %.2f / %.2f s' %
                          (len(sample_times), timestamp, reader.final_time), flush=True)
                if timestamp >= render_end - 1e-8:
                    break
        if not sample_times:
            raise ValueError('no complete camera groups captured')
        gaps = np.diff(sample_times)
        quality = {
            'five_cameras_all_samples': True,
            'source_history_unchanged': all(sha(p) == h for p, h in original_hashes.items()),
            'start_covered': bool(sample_times[0] <= reader.times[0]+cfg['start_end_tolerance_s']),
            'end_covered': bool(sample_times[-1] >= reader.final_time-cfg['start_end_tolerance_s']),
            'cadence_gap': not len(gaps) or float(max(gaps)) <= cfg['max_gap_s'],
            'body_mounts_verified': max_pose_error <= .002,
        }
        passed = all(quality.values())
        meta.update(collection_status=('PREVIEW' if preview_duration_s is not None else
                                       'PASS' if passed else 'FAIL'), quality_criteria=quality,
                    trainable=passed and result.get('status') == 'PASS' and preview_duration_s is None,
                    frame_groups=len(sample_times), total_rgb_images=len(sample_times)*5,
                    first_time_s=sample_times[0], last_time_s=sample_times[-1],
                    maximum_gap_s=float(max(gaps)) if len(gaps) else 0,
                    achieved_fps=(len(sample_times)-1)/(sample_times[-1]-sample_times[0]) if len(sample_times)>1 else 0,
                    phases=sorted(saved_phases), max_mount_error_m=max_pose_error,
                    rejected_poll_groups=rejected_groups)
    except Exception as exc:
        meta.update(collection_status='ERROR', trainable=False, error=repr(exc),
                    frame_groups=len(sample_times))
        raise
    finally:
        for buf in buffers:
            buf.close()
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
        write_meta()
    print('VLA episode:', target, meta['collection_status'], flush=True)
    return meta
