"""Display-only I_I ladder frame and four articulated corner modules."""

import json
import re
import struct

import numpy as np
from scipy.spatial.transform import Rotation


CORNERS = ('L1', 'R1', 'L2', 'R2')
BODY_CHANNELS = ('Xo', 'Yo', 'Zo', 'Yaw', 'Pitch', 'Roll_E')
WHEEL_CHANNELS = ('X', 'Y', 'Z', 'Yaw', 'Roll')
COLORS = {'frame': (.24, .27, .31), 'battery': (.32, .36, .40),
          'bracket': (.39, .42, .46), 'motor': (.24, .42, .53),
          'actuator': (.72, .53, .22), 'rod': (.67, .70, .72)}


class Mesh:
    def __init__(self):
        self.lines = []
        self.count = 0

    def add(self, points, faces, material):
        self.lines.extend('v %.8f %.8f %.8f' % tuple(point) for point in points)
        self.lines.append('usemtl ' + material)
        self.lines.extend('f ' + ' '.join(str(self.count + i + 1) for i in face)
                          for face in faces)
        self.count += len(points)

    def box(self, lower, upper, material):
        x0, y0, z0 = lower
        x1, y1, z1 = upper
        self.add([(x0,y0,z0), (x1,y0,z0), (x1,y1,z0), (x0,y1,z0),
                  (x0,y0,z1), (x1,y0,z1), (x1,y1,z1), (x0,y1,z1)],
                 [(3,2,1,0), (4,5,6,7), (0,1,5,4), (1,2,6,5), (2,3,7,6), (3,0,4,7)],
                 material)

    def cylinder(self, begin, end, radius, material, axis=0, center=(0., 0.)):
        count = 20
        angles = np.arange(count) * (2 * np.pi / count)
        ring = np.column_stack((np.cos(angles), np.sin(angles))) * radius + center
        points = np.concatenate([np.column_stack((np.full(count, x), ring))
                                 for x in (begin, end)])
        points = np.roll(points, axis, axis=1)
        faces = [tuple(range(count-1, -1, -1)), tuple(range(count, 2*count))]
        for i in range(count):
            j = (i+1) % count
            faces.append((i, j, j+count, i+count))
        self.add(points, faces, material)

    def write(self, path):
        path.write_text('mtllib ddev_mechanics.mtl\n' + '\n'.join(self.lines) + '\n',
                        encoding='ascii')
        return path.resolve().as_posix()


def _make_meshes(directory):
    material = '\n'.join('newmtl %s\nKa .18 .18 .18\nKd %.3f %.3f %.3f\n'
                         'Ks .24 .24 .24\nNs 24\nillum 2\n' % (name, *color)
                         for name, color in COLORS.items())
    (directory / 'ddev_mechanics.mtl').write_text(material, encoding='ascii')
    frame = Mesh()
    for side in (-1., 1.):
        y = side * .265
        frame.box((-2.42, y-.048, .30), (.48, y+.048, .43), 'frame')
        for x in (0., -1.925):
            frame.box((x-.11, y-.06, .40), (x+.11, y+.06, .62), 'bracket')
    for x in (-2.29, -1.79, -.97, -.26, .34):
        frame.box((x-.046, -.31, .32), (x+.046, .31, .41), 'frame')
    frame.box((-1.63, -.237, .205), (-.34, .237, .32), 'battery')
    for x in np.linspace(-1.54, -.43, 7):
        frame.box((x-.013, -.25, .198), (x+.013, .25, .221), 'bracket')
    meshes = {'frame': frame.write(directory / 'ddev_ladder_frame.obj')}
    for side, name in ((1, 'L'), (-1, 'R')):
        motor = Mesh()
        lo, hi = sorted((-side*.225, -side*.065))
        motor.cylinder(lo, hi, .106, 'motor', axis=1)
        motor.cylinder(lo-.009, lo+.014, .116, 'bracket', axis=1)
        motor.cylinder(hi-.014, hi+.009, .116, 'bracket', axis=1)
        motor.box((-.075, lo, .07), (.075, hi, .134), 'bracket')
        meshes[name] = motor.write(directory / f'ddev_corner_motor_{name}.obj')
    for name, length, radius, color in (
            ('arm', 1., .021, 'bracket'), ('rod', 1., .019, 'rod'),
            ('actuator', .125, .045, 'actuator')):
        mesh = Mesh()
        mesh.cylinder(0., length, radius, color)
        meshes[name] = mesh.write(directory / f'ddev_{name}.obj')
    return meshes


def _frame(lines, name, channels, mesh, *, scale=None):
    lines.extend(['ADD_REFERENCE_FRAME ' + name, 'SET_EULER_ANGLES YAW_PITCH_ROLL'])
    for axis, channel in zip(('X', 'Y', 'Z', 'YAW', 'PITCH', 'ROLL'), channels):
        if channel is not None:
            lines.append(f'SET_{axis}_NAME {channel}')
    if scale is not None:
        lines.append('SET_FRAME_SCALE_X_NAME ' + scale)
    lines.append('add_obj ' + mesh)
    lines.extend(['vsv_enable_shadows_recursive', 'END_REFERENCE_FRAME ' + name])


def stage_chassis(content, history, directory):
    """Add render channels in a separate native history and retain every source value."""
    required = set(BODY_CHANNELS) | {f'{axis}_{corner}' for corner in CORNERS
                                     for axis in WHEEL_CHANNELS}
    if not required.issubset(history.channels):
        # Older histories can still use the cab and cargo shell.
        return content, directory / history.vs.name, directory / history.vsb.name
    meshes = _make_meshes(directory)
    data = history.vsb.read_bytes()
    header = struct.unpack_from('<6i', data)
    dtype = '<f' + str(header[4])
    values = np.frombuffer(data, dtype=dtype, offset=24).reshape(-1, len(history.channels))
    columns = {name: i for i, name in enumerate(history.channels)}

    def channels(names):
        return values[:, [columns[n] for n in names]].astype(float)

    origin = channels(BODY_CHANNELS[:3])
    body = Rotation.from_euler('ZYX', channels(BODY_CHANNELS[3:]), degrees=True)
    lines = []
    _frame(lines, 'DDEV_Ladder', BODY_CHANNELS, meshes['frame'])
    extra, channel_specs = [], []
    for corner in CORNERS:
        side = 1. if corner.startswith('L') else -1.
        axle = 0. if corner.endswith('1') else -1.925
        wheel_names = [f'{axis}_{corner}' for axis in WHEEL_CHANNELS]
        center = channels(wheel_names[:3])
        carrier = Rotation.from_euler('ZX', channels(wheel_names[3:]), degrees=True)
        _frame(lines, 'DDEV_Motor_' + corner, wheel_names[:4]+[None, wheel_names[4]],
               meshes[corner[0]])
        joints = (
            ('screw', (axle, side*.27, .62), (0., -side*.17, .070)),
            ('arm_front', (axle+.145, side*.27, .33), (0., -side*.145, -.050)),
            ('arm_rear', (axle-.145, side*.27, .33), (0., -side*.145, -.050)),
            ('upper', (axle-.04, side*.28, .47), (0., -side*.16, .075)),
        )
        for label, mount, knuckle in joints:
            start = origin + body.apply(mount)
            end = center + carrier.apply(knuckle)
            delta = end - start
            yaw = np.rad2deg(np.arctan2(delta[:, 1], delta[:, 0]))
            pitch = np.rad2deg(np.arctan2(-delta[:, 2], np.hypot(delta[:, 0], delta[:, 1])))
            span = np.linalg.norm(delta, axis=1)
            name = f'DDEV_{corner}_{label}'
            names = [f'{name}_{suffix}' for suffix in ('X', 'Y', 'Z', 'Yaw', 'Pitch', 'Length')]
            extra.append(np.column_stack((start, np.rad2deg(np.unwrap(np.deg2rad(yaw))),
                                          pitch, span)))
            channel_specs.extend(zip(names, ('m', 'm', 'm', 'deg', 'deg', 'm')))
            if label == 'screw':
                _frame(lines, name + '_housing', names[:5]+[None], meshes['actuator'])
            _frame(lines, name, names[:5]+[None], meshes['rod' if label == 'screw' else 'arm'],
                   scale=names[-1])

    original = json.loads(history.vs.read_text(encoding='utf-8'))
    group = original['VsChannelGroup']
    for name, unit in channel_specs:
        group['Channels'].append({'Name Aliases': [name], 'Generic Name': name,
                                  'Long Name': name, 'Rigid Body': 'DDEV display',
                                  'Units': unit, 'Version': 2})
    basename = history.basename + '_appearance'
    vs, vsb = (directory / (basename + extension) for extension in ('.vs', '.vsb'))
    matrix = np.concatenate([values, *extra], axis=1).astype(dtype)
    vsb.write_bytes(struct.pack('<6i', *header[:5], matrix.shape[1]) + matrix.tobytes())
    vs.write_text(json.dumps(original), encoding='utf-8')
    # Insert ahead of the final parser END.
    final = list(re.finditer(rb'(?mi)^END[ \t]*(?=\r?$)', content))
    if not final:
        raise ValueError('native display parsfile is missing END')
    marker = final[-1].start()
    return content[:marker] + ('\n'.join(lines)+'\n').encode('mbcs') + content[marker:], vs, vsb
