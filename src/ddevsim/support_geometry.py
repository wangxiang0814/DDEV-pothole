"""Planar contact-polygon geometry for a three-wheel-supported vehicle."""

from __future__ import annotations

import math
from typing import Mapping


def cg_projection_world(
    front_origin_xy: tuple[float, float], cg_to_front_axle_m: float,
    roll_arm_m: float, pitch_arm_m: float, roll_deg: float,
    pitch_deg: float, yaw_deg: float,
) -> tuple[float, float]:
    """Approximate horizontal CG projection from TruckSim front-origin attitude."""
    local_x = -cg_to_front_axle_m - pitch_arm_m * math.sin(math.radians(pitch_deg))
    local_y = -roll_arm_m * math.sin(math.radians(roll_deg))
    yaw = math.radians(yaw_deg)
    return (
        front_origin_xy[0] + local_x * math.cos(yaw) - local_y * math.sin(yaw),
        front_origin_xy[1] + local_x * math.sin(yaw) + local_y * math.cos(yaw),
    )


def support_triangle_margin_m(
    contacts: Mapping[str, tuple[float, float]], cg_xy: tuple[float, float]
) -> float:
    """Signed minimum distance to the three edges; positive means inside."""
    if len(contacts) != 3:
        raise ValueError("exactly three contact locations are required")
    points = [(float(x), float(y)) for x, y in contacts.values()]
    center_x = sum(p[0] for p in points) / 3.0
    center_y = sum(p[1] for p in points) / 3.0
    points.sort(key=lambda p: math.atan2(p[1] - center_y, p[0] - center_x))
    area2 = sum(
        points[i][0] * points[(i + 1) % 3][1]
        - points[(i + 1) % 3][0] * points[i][1]
        for i in range(3)
    )
    if abs(area2) < 1e-9:
        raise ValueError("degenerate support triangle")
    px, py = map(float, cg_xy)
    distances = []
    for i in range(3):
        x0, y0 = points[i]
        x1, y1 = points[(i + 1) % 3]
        edge_length = math.hypot(x1 - x0, y1 - y0)
        distances.append(((x1 - x0) * (py - y0) - (y1 - y0) * (px - x0))
                         / edge_length)
    return min(distances)
