"""Fit TruckSim stock cargo geometry to the DDEV display mesh."""

from dataclasses import dataclass
from pathlib import Path
import re

import numpy as np


TRUCK_ASSETS = Path('Animator/3D_Shape_Files/Trucks')
CARGO_BED = TRUCK_ASSETS / 'LD_Utility_Truck/bed.obj'
STOCK_BOX = TRUCK_ASSETS / 'Chassis/Conventional'
FLAT_CAB = TRUCK_ASSETS / 'Cabs/Two_Axle_Cab_Over'
DECK_Z_M = 0.4
CAB_RAISE_M = 0.050
SHAPE_BLOCK = re.compile(
    rb'^[ \t]*ENTER_PARSFILE[ \t]+(?P<path>Animator[\\/]STL[\\/][^\r\n]+)\r?\n'
    rb'(?P<body>.*?)^[ \t]*EXIT_PARSFILE[ \t]+(?P=path)[ \t]*(?:\r?\n|$)',
    re.MULTILINE | re.IGNORECASE | re.DOTALL)
ADD_OBJ = re.compile(rb'(?mi)^([ \t]*add_obj[ \t]+)([^\r\n]+)')


@dataclass
class ObjPart:
    points: np.ndarray
    texcoords: list
    normals: np.ndarray
    faces: list
    material_libraries: list

    @classmethod
    def read(cls, path, *, min_x=-np.inf, min_z=-np.inf, max_z=np.inf):
        path = Path(path)
        points, texcoords, normals, faces, libraries = [], [], [], [], []
        material = None
        for line in path.read_text(encoding='ascii').splitlines():
            key, _, value = line.strip().partition(' ')
            if key == 'v':
                points.append([float(x) for x in value.split()[:3]])
            elif key == 'vt':
                texcoords.append(value)
            elif key == 'vn':
                normals.append([float(x) for x in value.split()])
            elif key == 'mtllib':
                libraries.append(path.parent / value)
            elif key == 'usemtl':
                material = value
            elif key == 'f':
                refs = [tuple(int(i) if i else None for i in ref.split('/'))
                        for ref in value.split()]
                counts = (len(points), len(texcoords), len(normals))
                refs = [tuple(counts[axis] + i + 1 if i is not None and i < 0 else i
                              for axis, i in enumerate(ref)) for ref in refs]
                if any(i is not None and not 1 <= i <= counts[axis]
                       for ref in refs for axis, i in enumerate(ref)):
                    raise ValueError('invalid stock OBJ index: ' + str(path))
                if all(points[ref[0]-1][0] >= min_x and
                       min_z - 1e-6 <= points[ref[0]-1][2] <= max_z + 1e-6
                       for ref in refs):
                    faces.append((material, refs))

        # Keep only vertices used by retained faces, so discarded chassis
        # geometry does not influence the fit. UVs remain exactly as supplied.
        used = sorted({ref[0] for _, refs in faces for ref in refs})
        remap = {old: new for new, old in enumerate(used, 1)}
        retained = np.asarray([points[i-1] for i in used], dtype=float).reshape(-1, 3)
        faces = [(material, [(remap[ref[0]], *ref[1:]) for ref in refs])
                 for material, refs in faces]
        return cls(retained, texcoords, np.asarray(normals).reshape(-1, 3), faces, libraries)


def fit_parts(parts, lower, upper):
    points = np.concatenate([part.points for part in parts])
    source_min, source_max = points.min(axis=0), points.max(axis=0)
    if np.any(source_max <= source_min) or np.any(upper <= lower):
        raise ValueError('invalid visual fit bounds')
    scale = (upper - lower) / (source_max - source_min)
    transform_parts(parts, scale, lower - source_min * scale)


def transform_parts(parts, scale, offset):
    for part in parts:
        part.points = part.points * scale + offset
        normals = part.normals / scale
        lengths = np.linalg.norm(normals, axis=1, keepdims=True)
        part.normals = normals / np.where(lengths > 0, lengths, 1)


def write_obj(parts, target):
    """Write independent OBJ index spaces and namespaced stock materials."""
    target = Path(target)
    mtl = target.with_suffix('.mtl')
    lines, materials = ['mtllib ' + mtl.name], []
    offsets = [0, 0, 0]
    for index, part in enumerate(parts):
        prefix = f'p{index}_'
        lines.extend('v %.10g %.10g %.10g' % tuple(point) for point in part.points)
        lines.extend('vt ' + uv for uv in part.texcoords)
        lines.extend('vn %.10g %.10g %.10g' % tuple(normal) for normal in part.normals)
        active = None
        for material, refs in part.faces:
            if material != active:
                lines.append('usemtl ' + prefix + material)
                active = material
            indices = ['/'.join(str(i + offsets[axis]) if i is not None else ''
                                for axis, i in enumerate(ref)) for ref in refs]
            lines.append('f ' + ' '.join(indices))
        for library in part.material_libraries:
            for line in library.read_text(encoding='ascii').splitlines():
                key, _, value = line.strip().partition(' ')
                if key == 'newmtl':
                    line = 'newmtl ' + prefix + value
                elif key.lower().startswith('map_'):
                    texture = library.parent / value.strip()
                    if not texture.is_file():
                        raise FileNotFoundError(texture)
                    line = key + ' ' + texture.resolve().as_posix()
                materials.append(line)
        offsets[0] += len(part.points)
        offsets[1] += len(part.texcoords)
        offsets[2] += len(part.normals)
    mtl.write_text('\n'.join(materials) + '\n', encoding='ascii')
    target.write_text('\n'.join(lines) + '\n', encoding='ascii')


def _asset_name(raw):
    return raw.strip().strip(b'"').replace(b'\\', b'/').lower()


def _shape_block(content, asset):
    original = asset.as_posix().lower().encode('ascii')
    matching = [m for m in ADD_OBJ.finditer(content) if _asset_name(m[2]) == original]
    if not matching:
        return None
    blocks = [m for m in SHAPE_BLOCK.finditer(content)
              if any(_asset_name(obj[2]) == original for obj in ADD_OBJ.finditer(m['body']))]
    if len(matching) != 1 or len(blocks) != 1 or len(ADD_OBJ.findall(blocks[0]['body'])) != 1:
        raise ValueError('expected one standalone display block for ' + str(asset))
    return blocks[0]


def _replace_mesh(content, block, mesh, *, neutral_color=False):
    mesh_ref = mesh.resolve().as_posix().encode('mbcs')
    body = ADD_OBJ.sub(lambda obj: obj[1] + mesh_ref, block['body'])
    if neutral_color:
        body = re.sub(rb'(?mi)^[ \t]*USE_RUN_COLOR[^\r\n]*(?:\r?\n|$)', b'', body)
        body = re.sub(rb'(?mi)^([ \t]*SET_COLOR)[^\r\n]+', rb'\1 1 1 1', body)
    return content[:block.start('body')] + body + content[block.end('body'):]


def stage_cargo_box(content, stage, resources):
    """Replace exactly one standalone LD Utility cargo display block."""
    block = _shape_block(content, CARGO_BED)
    if block is None:
        return content

    bed = resources / CARGO_BED
    original_bed = ObjPart.read(bed)
    lower, upper = original_bed.points.min(axis=0), original_bed.points.max(axis=0)
    if not lower[2] < DECK_Z_M < upper[2]:
        raise ValueError('stock bed does not contain the cargo deck height')
    lower[2] = DECK_Z_M
    walls = ObjPart.read(resources / STOCK_BOX / 'walls_with_decal_2.obj')
    trim = ObjPart.read(resources / STOCK_BOX / 'box_details.obj', min_z=walls.points[:, 2].min())
    fit_parts([walls, trim], lower, upper)
    chassis = ObjPart.read(bed, max_z=DECK_Z_M)
    mesh = stage / 'ddev_stock_box.obj'
    write_obj([walls, trim, chassis], mesh)
    return _replace_mesh(content, block, mesh, neutral_color=True)


def stage_flat_cab(content, stage, resources):
    """Retain the chosen cab's single front arch and register it to axle 1."""
    originals = ('body.obj', 'cab_details.obj', 'windows.obj', 'steering_wheel.obj')
    stock = ('cab_body.obj', *originals[1:])
    source_dir = CARGO_BED.parent
    if _shape_block(content, source_dir / originals[0]) is None:
        return content
    # The stock model's apron beyond X=-1.13 belongs to its extended chassis.
    # The front arch crown is at approximately X=-0.04 in the stock mesh.
    parts = [ObjPart.read(resources / FLAT_CAB / name, min_x=-1.13) for name in stock]
    rear = parts[0].points[:, 0].min()
    arch_x = -.04
    # Original cargo front X = -.5825 m in the body frame. Allow a 30 mm gap.
    scale_x = .5525 / (arch_x - rear)
    transform_parts(parts, np.array([scale_x, .53, .48]),
                    np.array([-arch_x * scale_x, 0., .18 - .263 + CAB_RAISE_M]))
    for original, part in zip(originals, parts):
        block = _shape_block(content, source_dir / original)
        if block is None:
            continue
        mesh = stage / ('ddev_flat_' + original)
        write_obj([part], mesh)
        content = _replace_mesh(content, block, mesh)
    return content
