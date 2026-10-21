"""Body-fixed native camera descriptions, without visual camera-shaped objects."""
import json
import math
import re

NAMES = ('front', 'wheel_FL', 'wheel_FR', 'wheel_RL', 'wheel_RR')


def validate_config(cfg):
    if cfg.get('schema_version') != 1:
        raise ValueError('unsupported demonstration schema')
    if cfg.get('backend') != 'native_history_shared_buffer':
        raise ValueError('unsupported rendering backend')
    for key in ('fps', 'playback_speed', 'max_gap_s', 'frame_timeout_s',
                'connection_timeout_s', 'start_end_tolerance_s'):
        if not math.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError('invalid positive parameter: ' + key)
    if [x['name'] for x in cfg['cameras']] != list(NAMES):
        raise ValueError('cameras must be unique front/FL/FR/RL/RR in order')
    for cam in cfg['cameras']:
        for key in ('position_m', 'yaw_pitch_roll_deg'):
            if len(cam[key]) != 3 or not all(math.isfinite(x) for x in cam[key]):
                raise ValueError('invalid camera pose')
        if not 1 < cam['horizontal_fov_deg'] < 170:
            raise ValueError('invalid FOV')
        near, far = cam.get('clip_near_m', .005), cam.get('clip_far_m', 200)
        if not math.isfinite(near) or not math.isfinite(far) or not 0 < near < far:
            raise ValueError('invalid clipping planes')
        mount = cam.get('mount', {})
        if mount.get('surface_axis') not in (0, 1, 2) or mount.get('outward_sign') not in (-1, 1):
            raise ValueError('camera requires a body surface mount')
        if not 0 < mount.get('lens_standoff_m', 0) <= mount.get('max_standoff_m', 0) <= .05:
            raise ValueError('camera must be mounted within 50 mm of body surface')
        for key in ('width', 'height'):
            if type(cam[key]) is not int or not 16 <= cam[key] <= 4096:
                raise ValueError('invalid image size')
    return cfg


def load_config(path):
    with open(path, encoding='utf-8') as src:
        return validate_config(json.load(src))


def camera_par(cfg, prefix):
    validate_config(cfg)
    if not re.fullmatch(r'[A-Za-z0-9_]+', prefix):
        raise ValueError('buffer prefix must be an ASCII identifier')
    # Close the legacy main cameraman before adding independent render cameras.
    lines = ['END_ENTITY CAMERAMAN', 'ADD_REFERENCE_FRAME VLA_Body',
             'SET_X_NAME Xo', 'SET_Y_NAME Yo', 'SET_Z_NAME Zo',
             'SET_YAW_NAME Yaw', 'SET_PITCH_NAME Pitch', 'SET_ROLL_NAME Roll_E',
             'SET_EULER_ANGLES YAW_PITCH_ROLL']
    for cam in cfg['cameras']:
        name = prefix + '_' + cam['name']
        lines += ['ADD_ENTITY Transform ' + name + '_mount']
        lines += ['TRANSLATE_%s %.12g' % (axis, value)
                  for axis, value in zip('XYZ', cam['position_m'])]
        lines += ['ROTATE_%s %.12g' % (axis, value)
                  for axis, value in zip(('YAW', 'PITCH', 'ROLL'), cam['yaw_pitch_roll_deg'])]
        lines += ['END_ENTITY Transform ' + name + '_mount',
                  'ADD_ENTITY Cameraman Fixed', 'NAME ' + name + '_camera',
                  'BASE_REFERENCE_FRAME ' + name + '_mount',
                  'SET_FIELD_OF_VIEW %.12g' % cam['horizontal_fov_deg'],
                  'CLIP_NEAR %.12g' % cam.get('clip_near_m', .005),
                  'CLIP_FAR %.12g' % cam.get('clip_far_m', 200),
                  'END_ENTITY Cameraman Fixed',
                  'ADD_ENTITY RenderView ' + name + '_view',
                  'CAMERAMAN_NAME ' + name + '_camera',
                  'TEX_W %d' % cam['width'], 'TEX_H %d' % cam['height'],
                  'END_ENTITY RenderView ' + name + '_view',
                  'ADD_ENTITY SharedBuffer', 'BUFFER_NAME ' + name,
                  'BUFFER_RENDERVIEW ' + name + '_view',
                  'BUFFER_ADD_CONTENT_TYPE rgb', 'END_ENTITY SharedBuffer']
    lines += ['END_REFERENCE_FRAME VLA_Body']
    return '\n'.join(lines) + '\n'
