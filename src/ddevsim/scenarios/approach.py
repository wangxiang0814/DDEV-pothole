"""Shared episode start geometry and routing for the nominal pit scenes."""
from __future__ import annotations

from dataclasses import dataclass, replace
import math


INITIAL_DISTANCE_RANGE_M = (10., 15.)
DEFAULT_INITIAL_DISTANCE_M = 12.
VEHICLE_START_STATION_M = 0.
DRY_ROAD_FRICTION = .70
ROAD_START_STATION_M = -20.
ROAD_END_STATION_M = 65.


def approach_distance(value=None):
    distance = DEFAULT_INITIAL_DISTANCE_M if value is None else value
    if not INITIAL_DISTANCE_RANGE_M[0] <= distance <= INITIAL_DISTANCE_RANGE_M[1]:
        raise ValueError('approach distance must be between 10 and 15 m')
    return distance


def place_pit_from_origin(geometry, distance_m):
    edge_m = VEHICLE_START_STATION_M + distance_m
    return replace(geometry, start_station_m=edge_m, approach_distance_m=distance_m,
                   friction=DRY_ROAD_FRICTION, road_lead_in_m=edge_m - ROAD_START_STATION_M,
                   road_run_out_m=ROAD_END_STATION_M - edge_m - geometry.length_m)


@dataclass(frozen=True)
class ExpertRoute:
    strategy: str
    case: str
    speed_kph: float | None = None


def select_expert(coverage, depth_m, length_m, width_m):
    width = {'right_track': .9, 'wide': 1.8}.get(coverage)
    if width is not None and math.isclose(length_m, .8) and math.isclose(width_m, width):
        if coverage == 'right_track' and math.isclose(depth_m, .20):
            return ExpertRoute('wheel_lift', 'seed_repeat')
        for depth, speed in ((.02, 3.), (.04, 2. if coverage == 'right_track' else 3.), (.06, 1.)):
            if math.isclose(depth_m, depth) and (depth < .06 or coverage == 'right_track'):
                return ExpertRoute('shallow_press', coverage, speed)
    raise ValueError('no automatic expert route for this pit geometry')
