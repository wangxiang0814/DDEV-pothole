"""Measured contact geometry and nearest safe quasi-static ZMP target."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping


ORDER = ("FL", "FR", "RL", "RR")
SUPPORT = ("FL", "RL", "RR")


@dataclass(frozen=True)
class SupportAssessment:
    support_corners: tuple[str, str, str]
    zmp_xy: tuple[float, float]
    com_xy: tuple[float, float]
    barycentric: tuple[float, float, float]
    lambda_min: float
    edge_distance_m: float
    inside: bool
    safe_inside: bool
    target_zmp_xy: tuple[float, float]
    target_lambda_min: float
    target_distance_m: float


def _lambda(points, point):
    a, b, c = points
    den = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
    if abs(den) < 1e-12:
        raise ValueError("degenerate support triangle")
    l0 = ((b[1] - c[1]) * (point[0] - c[0]) + (c[0] - b[0]) * (point[1] - c[1])) / den
    l1 = ((c[1] - a[1]) * (point[0] - c[0]) + (a[0] - c[0]) * (point[1] - c[1])) / den
    return l0, l1, 1.0 - l0 - l1


def _closest_segment(point, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    fraction = max(0.0, min(1.0, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) / (dx * dx + dy * dy)))
    return a[0] + fraction * dx, a[1] + fraction * dy


def assess_support(
    contacts: Mapping[str, tuple[float, float]],
    wheel_load_n: Mapping[str, float],
    com_xy: tuple[float, float],
    lambda_safe: float,
    *,
    lifted_corner: str = "FR",
) -> SupportAssessment:
    """Assess three-wheel support and project ZMP to its safe triangle."""
    if not 0.0 <= lambda_safe < 1.0 / 3.0:
        raise ValueError("lambda_safe must be in [0, 1/3)")
    if set(contacts) != set(ORDER) or set(wheel_load_n) != set(ORDER):
        raise ValueError("four contact positions and wheel loads required")
    if lifted_corner not in ORDER:
        raise ValueError("lifted corner must be a vehicle wheel")
    values = [*com_xy, *(v for xy in contacts.values() for v in xy), *wheel_load_n.values()]
    if not all(math.isfinite(float(v)) for v in values) or min(wheel_load_n.values()) < 0.0:
        raise ValueError("nonfinite geometry or negative wheel load")
    total = sum(wheel_load_n.values())
    if total <= 0.0:
        raise ValueError("total wheel load must be positive")
    zmp = tuple(sum(wheel_load_n[c] * contacts[c][axis] for c in ORDER) / total for axis in (0, 1))
    support_corners = tuple(c for c in ORDER if c != lifted_corner)
    triangle = tuple(contacts[c] for c in support_corners)
    barycentric = _lambda(triangle, zmp)
    lam_min = min(barycentric)
    twice_area = abs((triangle[1][0] - triangle[0][0]) * (triangle[2][1] - triangle[0][1])
                     - (triangle[1][1] - triangle[0][1]) * (triangle[2][0] - triangle[0][0]))
    edge = min(
        barycentric[i] * twice_area / math.dist(triangle[(i + 1) % 3], triangle[(i + 2) % 3])
        for i in range(3)
    )
    # Barycentric interpolation is affine: these are exactly lambda_i >= lambda_safe.
    shrink = tuple(
        tuple((1.0 - 3.0 * lambda_safe) * triangle[i][axis]
              + lambda_safe * sum(v[axis] for v in triangle) for axis in (0, 1))
        for i in range(3)
    )
    if lam_min >= lambda_safe - 1e-12:
        target = zmp
    else:
        candidates = (_closest_segment(zmp, shrink[i], shrink[(i + 1) % 3]) for i in range(3))
        target = min(candidates, key=lambda p: math.dist(p, zmp))
    return SupportAssessment(
        support_corners=support_corners,
        zmp_xy=zmp, com_xy=tuple(com_xy), barycentric=barycentric,
        lambda_min=lam_min, edge_distance_m=edge,
        inside=lam_min >= -1e-12, safe_inside=lam_min >= lambda_safe - 1e-12,
        target_zmp_xy=target, target_lambda_min=min(_lambda(triangle, target)),
        target_distance_m=math.dist(target, zmp),
    )
