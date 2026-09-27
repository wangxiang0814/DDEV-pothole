"""Explicit TruckSim Visualizer mesh for the DDEV pothole scene.

TruckSim's solver consumes the road elevation map, but it does not compile a newly
hand-authored ``Road: Animator Surface Shapes`` block into the ``.obj/.ani`` pair that
VS Visualizer expects.  This module writes that missing render mesh explicitly.  The
physical road remains the TruckSim dZ map; this mesh is visual-only and uses the same
geometry and dimensions.
"""

from __future__ import annotations

from pathlib import Path
from math import cos, pi, sin
from typing import List, Tuple


OBJ_NAME = "DDEV_pothole_scene.obj"
MTL_NAME = "DDEV_pothole_scene.mtl"


def _fmt(value: float) -> str:
    return "%.6f" % float(value)


def build_visual_mesh_text(scenario) -> Tuple[str, str]:
    """Return a UV-mapped OBJ/MTL for natural ground, rough asphalt and the pit."""
    vertices: List[Tuple[float, float, float]] = []
    texcoords: List[Tuple[float, float]] = []
    chunks: List[str] = ["# DDEV explicit visual road mesh", "mtllib " + MTL_NAME]

    def quad(material: str, points, uv=None) -> None:
        first = len(vertices) + 1
        vertices.extend(points)
        # TruckSim's road textures are tileable.  World-aligned UVs keep adjacent
        # quads continuous and give the asphalt/grass a natural physical scale.
        metres_per_tile = {
            "DDEV_Grass": 4.0,
            "DDEV_Road": 2.0,
            "DDEV_Pit": 0.5,
            "DDEV_Sky": 1.0,
        }[material]
        texcoords.extend(
            uv if uv is not None
            else ((x / metres_per_tile, y / metres_per_tile) for x, y, _ in points)
        )
        chunks.append("usemtl " + material)
        chunks.append(
            "f %d/%d %d/%d %d/%d %d/%d"
            % (first, first, first + 1, first + 1, first + 2, first + 2, first + 3, first + 3)
        )

    x0 = scenario.road_start_m
    x1 = scenario.road_end_m
    road_y0 = -abs(scenario.road_half_width_m)
    road_y1 = abs(scenario.road_half_width_m)
    outer = abs(scenario.offroad_half_width_m)
    zg = float(scenario.ground_visual_offset_m)
    zr = float(scenario.road_visual_offset_m)

    # API-created histories on TruckSim 2019 do not always instantiate the database
    # skybox even when its directives are present.  A nearby textured cylinder in the
    # already-proven explicit OBJ makes the sky deterministic.  Its radius is kept
    # below the Visualizer far clip plane; ``show_back on`` exposes its inward side.
    # Centre it on the short manoeuvre, not on the full 61 m visual road.  Radius 35 m
    # keeps every wall within the old Visualizer's practical clip range while leaving
    # ample room for the camera orbit and the complete 11 s crawl.
    sky_cx = scenario.start_station_m + 5.0
    sky_cy = 0.0
    sky_radius = 35.0
    sky_bottom = -4.0
    sky_top = 45.0
    sky_segments = 24
    for i in range(sky_segments):
        a0 = 2.0 * pi * i / sky_segments
        a1 = 2.0 * pi * (i + 1) / sky_segments
        p0 = (sky_cx + sky_radius * cos(a0), sky_cy + sky_radius * sin(a0), sky_bottom)
        p1 = (sky_cx + sky_radius * cos(a1), sky_cy + sky_radius * sin(a1), sky_bottom)
        p2 = (p1[0], p1[1], sky_top)
        p3 = (p0[0], p0[1], sky_top)
        # Reverse the usual cylinder winding so front faces point inward toward the
        # camera.  This does not depend on Visualizer honouring ``show_back on``.
        quad(
            "DDEV_Sky", [p1, p0, p3, p2],
            [((i + 1) / sky_segments, 0.05), (i / sky_segments, 0.05),
             (i / sky_segments, 0.95), ((i + 1) / sky_segments, 0.95)],
        )

    # Road-external green ground.  It deliberately stops at the road edge, so there is
    # no coplanar grass surface available to cover the asphalt.
    quad("DDEV_Grass", [(x0, -outer, zg), (x1, -outer, zg),
                         (x1, road_y0, zg), (x0, road_y0, zg)])
    quad("DDEV_Grass", [(x0, road_y1, zg), (x1, road_y1, zg),
                         (x1, outer, zg), (x0, outer, zg)])

    s0, s1 = scenario.station_span()
    y0, y1 = scenario.lateral_span()
    # Asphalt before/after the pit, plus the two side strips alongside it.
    quad("DDEV_Road", [(x0, road_y0, zr), (s0, road_y0, zr),
                        (s0, road_y1, zr), (x0, road_y1, zr)])
    quad("DDEV_Road", [(s1, road_y0, zr), (x1, road_y0, zr),
                        (x1, road_y1, zr), (s1, road_y1, zr)])
    quad("DDEV_Road", [(s0, road_y0, zr), (s1, road_y0, zr),
                        (s1, y0, zr), (s0, y0, zr)])
    quad("DDEV_Road", [(s0, y1, zr), (s1, y1, zr),
                        (s1, road_y1, zr), (s0, road_y1, zr)])

    # True depressed pothole with a 50 mm mesh.  The edge-transition distance is used
    # on all four sides, giving visible entry/exit and side walls without a vertical,
    # zero-area face.  Every face is explicitly marked as the near-black pit material.
    step = min(0.05, float(scenario.pothole_mesh_interval_m))
    nx = max(2, int(round((s1 - s0) / step)))
    ny = max(2, int(round((y1 - y0) / step)))
    xs = [s0 + (s1 - s0) * i / nx for i in range(nx + 1)]
    ys = [y0 + (y1 - y0) * j / ny for j in range(ny + 1)]
    edge = max(1e-6, float(scenario.edge_transition_m))

    def pit_z(x: float, y: float) -> float:
        factor = min(
            1.0,
            max(0.0, (x - s0) / edge),
            max(0.0, (s1 - x) / edge),
            max(0.0, (y - y0) / edge),
            max(0.0, (y1 - y) / edge),
        )
        return zr - float(scenario.depth_m) * factor

    for i in range(nx):
        for j in range(ny):
            xa, xb = xs[i], xs[i + 1]
            ya, yb = ys[j], ys[j + 1]
            quad("DDEV_Pit", [
                (xa, ya, pit_z(xa, ya)),
                (xb, ya, pit_z(xb, ya)),
                (xb, yb, pit_z(xb, yb)),
                (xa, yb, pit_z(xa, yb)),
            ])

    vertex_lines = ["v %s %s %s" % (_fmt(x), _fmt(y), _fmt(z)) for x, y, z in vertices]
    texture_lines = ["vt %s %s" % (_fmt(u), _fmt(v)) for u, v in texcoords]
    obj = "\n".join(chunks[:2] + vertex_lines + texture_lines + chunks[2:]) + "\n"
    mtl = """# DDEV textured scene materials (textures shipped with TruckSim 2019)
newmtl DDEV_Grass
Ka 0.65 0.72 0.60
Kd 0.82 0.90 0.78
Ks 0.02 0.02 0.02
Ns 4
illum 2
map_Kd Animator/Road_Materials/Grass/grass_dark_di.dds

newmtl DDEV_Road
Ka 0.38 0.40 0.42
Kd 0.56 0.58 0.60
Ks 0.08 0.08 0.08
Ns 12
illum 2
map_Kd Animator/Road_Materials/Road_Surfaces/Asphalt_Fine_di.dds

newmtl DDEV_Pit
Ka 0.55 0.48 0.37
Kd 0.72 0.64 0.50
Ks 0.00 0.00 0.00
Ns 2
illum 2
map_Kd Animator/Road_Materials/Dirt/Dirt_Ground_B_di.dds

newmtl DDEV_Sky
Ka 1.00 1.00 1.00
Kd 1.00 1.00 1.00
Ks 0.00 0.00 0.00
Ns 1
illum 1
map_Kd Animator/3D_Shape_Files/Environment/Sky_Boxes/Partly_Cloudy_Sky/Sky_Partly_Cloudy_di.dds
"""
    return obj, mtl


def write_visual_mesh(directory: Path, scenario) -> Tuple[Path, Path]:
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    obj_text, mtl_text = build_visual_mesh_text(scenario)
    obj = directory / OBJ_NAME
    mtl = directory / MTL_NAME
    obj.write_text(obj_text, encoding="ascii")
    mtl.write_text(mtl_text, encoding="ascii")
    return obj, mtl
