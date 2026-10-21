"""Verify lens stand-off against the actual native body mesh, in body metres."""
from pathlib import Path
import hashlib
import numpy as np


def surface_coordinate(triangles, position, axis, sign):
    """Intersect an axis ray; choose the outermost surface on this body side."""
    other = [i for i in range(3) if i != axis]
    projected = triangles[:, :, other]
    edges = projected[:, 1:] - projected[:, :1]
    det = edges[:, 0, 0]*edges[:, 1, 1] - edges[:, 0, 1]*edges[:, 1, 0]
    valid = abs(det) > 1e-12
    q = np.asarray(position)[other] - projected[:, 0]
    u, v = np.zeros(len(triangles)), np.zeros(len(triangles))
    u[valid] = (q[valid, 0]*edges[valid, 1, 1]-q[valid, 1]*edges[valid, 1, 0])/det[valid]
    v[valid] = (edges[valid, 0, 0]*q[valid, 1]-edges[valid, 0, 1]*q[valid, 0])/det[valid]
    valid &= (u >= -1e-8) & (v >= -1e-8) & (u+v <= 1+1e-8)
    hits = (triangles[valid, 0, axis] +
            u[valid]*(triangles[valid, 1, axis]-triangles[valid, 0, axis]) +
            v[valid]*(triangles[valid, 2, axis]-triangles[valid, 0, axis]))
    if not len(hits):
        raise ValueError('camera has no body surface beneath its mounting axis')
    return float(max(hits) if sign > 0 else min(hits))


def verify_mounts(config, history_par, resource_root):
    lines = Path(history_par).read_text(encoding='utf-8', errors='replace').splitlines()
    reports = {}
    for cam in config['cameras']:
        mount = cam['mount']
        asset = mount['surface_asset']
        matches = [i for i, line in enumerate(lines)
                   if line.lower().startswith('add_obj ') and
                   Path(line.split(None, 1)[1]).name == asset]
        if len(matches) != 1:
            raise ValueError('ambiguous or missing camera mounting asset: '+asset)
        i = matches[0]
        path = Path(resource_root)/lines[i].split(None, 1)[1]
        offset = np.zeros(3)
        for line in lines[i+1:]:
            parts = line.split()
            if not parts:
                continue
            key = parts[0].lower()
            if key in ('add_obj', 'add_stl', 'add_reference_frame'):
                break
            if key.startswith(('set_scale_', 'set_angle_')):
                raise ValueError('mount audit requires an unrotated/unscaled body asset')
            if key in ('set_offset_x', 'set_offset_y', 'set_offset_z'):
                offset['xyz'.index(key[-1])] += float(parts[1])
        if not np.allclose(offset, mount['asset_offset_m'], atol=1e-9):
            raise ValueError('configured mount asset transform differs from native history')
        vertices, faces = [], []
        for line in path.read_text().splitlines():
            if line.startswith('v '):
                vertices.append(list(map(float, line.split()[1:4])))
            elif line.startswith('f '):
                f = [int(x.split('/')[0])-1 for x in line.split()[1:]]
                faces.extend([f[0], f[j], f[j+1]] for j in range(1, len(f)-1))
        triangles = (np.asarray(vertices)+offset)[np.asarray(faces)]
        axis, sign = mount['surface_axis'], mount['outward_sign']
        surface = surface_coordinate(triangles, cam['position_m'], axis, sign)
        distance = sign*(cam['position_m'][axis]-surface)
        if not 0 < distance <= mount['max_standoff_m']:
            raise ValueError('camera lens is inside or floating away from body: '+cam['name'])
        if abs(distance-mount['lens_standoff_m']) > 1e-5:
            raise ValueError('camera housing stand-off does not match configuration')
        anchor = list(cam['position_m'])
        anchor[axis] = surface
        reports[cam['name']] = dict(surface_anchor_body_m=anchor,
            lens_position_body_m=cam['position_m'], standoff_m=distance,
            asset=str(path), asset_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            asset_offset_m=offset.tolist())
    return reports
