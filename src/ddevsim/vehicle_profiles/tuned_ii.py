"""The current tuned I_I vehicle, with explicit provenance and capabilities."""
from __future__ import annotations

from dataclasses import dataclass, asdict
import hashlib
import json
import math
from pathlib import Path
import re
import shutil

from ddevsim.runtime.run_workspace import ROOT, sha256_file
from .transformations import (configure_crawl_text, enable_steering_import,
                             clamp_kinematics_at_original_bounds, scale_lateral_ride_movement)


@dataclass(frozen=True)
class TunedIIProfile:
    profile_id: str = 'tuned_ii_20261005'
    source_dir: Path = ROOT / 'evidence/static_fr_ii/model'
    track_m: float = 1.260
    wheelbase_m: float = 1.925
    # Software coverage-check envelope; not a measured hardware tyre width.
    # It affects scene validation only, never TruckSim vehicle parameters.
    tyre_half_width_m: float = .10
    tyre_radius_m: float = .263
    tyre_low_speed_slip_m_s: float = .2
    lateral_ride_scale: float = 0.
    steering_enabled: bool = True
    solver_step_s: float = .0005
    control_period_s: float = .02

    def __post_init__(self):
        for name in ('track_m', 'wheelbase_m', 'tyre_half_width_m', 'tyre_radius_m',
                     'tyre_low_speed_slip_m_s', 'solver_step_s', 'control_period_s'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError('profile requires positive finite ' + name)
        if not math.isfinite(self.lateral_ride_scale) or not 0 <= self.lateral_ride_scale <= 1:
            raise ValueError('lateral_ride_scale must be in 0..1')
        object.__setattr__(self, 'source_dir', Path(self.source_dir).resolve())


DEFAULT_PROFILE = TunedIIProfile()


def vehicle_identity(run_all: Path) -> str:
    """Hash vehicle text separately from declared road/procedure/display changes.

    Existing full-file calibration checks remain authoritative for gain reuse.
    This identity is never substituted for an identification model hash.
    """
    text = Path(run_all).read_text(encoding='utf-8')
    text = re.sub(r'ENTER_PARSFILE Procedures\\[^\n]+\n.*?EXIT_PARSFILE Procedures\\[^\n]+',
                  '', text, flags=re.DOTALL)
    text = re.sub(r'(?m)^(?:SSTART|TSTOP)\s+[-+0-9.eE]+\s*$', '', text)
    text = re.sub(r'(?m)^SET_(?:AZIMUTH|ELEVATION|DISTANCE|FIELD_OF_VIEW|LOOKPOINT_[XYZ])\s+[-+0-9.eE]+\s*$',
                  '', text)
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


def build_vehicle(target: Path, profile: TunedIIProfile = DEFAULT_PROFILE) -> dict:
    target = Path(target).resolve()
    if target == profile.source_dir or profile.source_dir in target.parents:
        raise ValueError('cannot write into vehicle source assets')
    shutil.copytree(profile.source_dir, target, ignore=shutil.ignore_patterns('output'))
    (target / 'output').mkdir()
    path = target / 'run_all.par'
    path.write_text(configure_crawl_text(path.read_text(encoding='utf-8'),
                                         profile.tyre_low_speed_slip_m_s), encoding='utf-8')
    clamp_kinematics_at_original_bounds(target)
    scale_lateral_ride_movement(target, profile.lateral_ride_scale)
    if profile.steering_enabled:
        enable_steering_import(target)
    recipe = asdict(profile)
    recipe.pop('source_dir')
    recipe['source_model_sha256'] = sha256_file(profile.source_dir / 'run_all.par')
    recipe['source_simfile_sha256'] = sha256_file(profile.source_dir / 'simfile.sim')
    recipe_hash = hashlib.sha256(json.dumps(recipe, sort_keys=True).encode('utf-8')).hexdigest()
    return {'profile_id': profile.profile_id, 'recipe_sha256': recipe_hash,
            'vehicle_sha256': vehicle_identity(path), 'recipe': recipe,
            'input_capability': 'torque4_force4_steering' if profile.steering_enabled else 'torque4_force4',
            'solver_step_s': profile.solver_step_s, 'control_period_s': profile.control_period_s}
