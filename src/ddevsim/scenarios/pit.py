"""Single-side and both-track pit contracts with explicit geometry."""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Mapping

from ddevsim.pothole_case import PotholeScenario, corner_module_scenario
from ddevsim.runtime.contracts import freeze
from ddevsim.vehicle_profiles.tuned_ii import DEFAULT_PROFILE


@dataclass(frozen=True)
class PitCase:
    strategy_id: str
    case_id: str
    vehicle_profile_id: str
    coverage: str
    geometry: PotholeScenario
    initial_state: Mapping
    display: Mapping

    def __post_init__(self):
        object.__setattr__(self, 'initial_state', freeze(self.initial_state))
        object.__setattr__(self, 'display', freeze(self.display))

    @property
    def wheel_sequence(self):
        if self.coverage == 'right_track':
            return (('FR',), ('RR',))
        if self.coverage == 'both_tracks':
            return (('FL', 'FR'), ('RL', 'RR'))
        raise ValueError('unknown pit coverage')


def validate_case(case: PitCase, profile=DEFAULT_PROFILE) -> None:
    if case.strategy_id not in ('wheel_lift', 'shallow_press'):
        raise ValueError('unknown strategy in scene')
    if case.vehicle_profile_id != profile.profile_id:
        raise ValueError('scene and vehicle profile differ')
    geometry = case.geometry
    for name in ('length_m', 'width_m', 'depth_m', 'edge_transition_m', 'friction',
                 'stop_s', 'road_half_width_m', 'road_mesh_interval_m', 'pothole_mesh_interval_m'):
        value = getattr(geometry, name)
        if not math.isfinite(value) or value <= 0:
            raise ValueError('scene requires positive finite ' + name)
    for name in ('start_station_m', 'center_y_m', 'camera_distance_m', 'camera_field_of_view_deg',
                 'camera_azimuth_deg', 'camera_elevation_deg', 'road_visual_offset_m'):
        if not math.isfinite(getattr(geometry, name)):
            raise ValueError('nonfinite ' + name)
    if (geometry.length_m <= 2 * geometry.edge_transition_m or
            geometry.width_m <= 2 * geometry.edge_transition_m):
        raise ValueError('pit edge transitions overlap')
    lo, hi = geometry.lateral_span()
    if lo <= -geometry.road_half_width_m or hi >= geometry.road_half_width_m:
        raise ValueError('pit must fit inside physical road')
    if case.coverage not in ('right_track', 'both_tracks'):
        raise ValueError('unknown pit coverage')
    footprints = {'right': (-profile.track_m / 2 - profile.tyre_half_width_m,
                            -profile.track_m / 2 + profile.tyre_half_width_m),
                  'left': (profile.track_m / 2 - profile.tyre_half_width_m,
                           profile.track_m / 2 + profile.tyre_half_width_m)}
    for side in ('right',) if case.coverage == 'right_track' else ('left', 'right'):
        a, b = footprints[side]
        if lo + geometry.edge_transition_m > a or hi - geometry.edge_transition_m < b:
            raise ValueError('pit does not fully cover expected ' + side + ' tyre envelope')
    if case.coverage == 'right_track' and hi >= footprints['left'][0]:
        raise ValueError('single-side pit reaches the left tyre envelope')
    speed = float(case.initial_state['speed_mps'])
    start = float(case.initial_state['vehicle_start_station_m'])
    if not math.isfinite(speed) or speed != 0 or not math.isfinite(start):
        raise ValueError('vehicle must initialize at rest with finite start station')
    # Source I_I origin is the front axle; reserve lip transition for static geometry settling.
    if start + profile.tyre_radius_m + geometry.edge_transition_m >= geometry.leading_edge_m:
        raise ValueError('front tyres must be entirely before pit entry')
    if start - profile.wheelbase_m - profile.tyre_radius_m < geometry.road_start_m:
        raise ValueError('rear tyres start outside the generated road')


def load_case(template: Path, *, depth_m=None, length_m=None, width_m=None,
              approach_distance_m=None, friction=None, stop_s=None) -> PitCase:
    data = json.loads(Path(template).read_text(encoding='utf-8'))
    values = dict(data['geometry'])
    overrides = dict(depth_m=depth_m, length_m=length_m, width_m=width_m,
                     approach_distance_m=approach_distance_m, friction=friction, stop_s=stop_s)
    values.update({k: v for k, v in overrides.items() if v is not None})
    for name in ('depth_m', 'length_m', 'width_m'):
        if values.get(name) is None:
            raise ValueError('explicit scene dimension required: ' + name)
    geometry = corner_module_scenario(**values)
    case = PitCase(data['strategy_id'], data['case_id'], data['vehicle_profile_id'],
                   data['coverage'], geometry,
                   {'speed_mps': 0., 'vehicle_start_station_m': geometry.leading_edge_m - geometry.approach_distance_m},
                   data.get('display', {}))
    validate_case(case)
    return case
