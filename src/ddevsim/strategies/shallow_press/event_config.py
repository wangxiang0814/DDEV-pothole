"""Comfort-first event crawl assumptions; independent of expert one."""
from dataclasses import dataclass,fields
import math


@dataclass(frozen=True)
class EventCrawlConfig:
    force_policy: str='legacy'
    entry_kph: float=.5
    climb_kph: float=.7
    pit_kph: float=.8
    prepare_distance_m: float=.6
    braking_margin_m: float=.15
    braking_acceleration_m_s2: float=.16
    reference_acceleration_m_s2: float=.16
    reference_jerk_m_s3: float=.4
    recontact_s: float=.08
    phase_hysteresis_m: float=.01
    entry_load_deviation: float=.1
    radius_m: float=.263
    suspension_preview_s: float=.35
    suspension_bandwidth: float=8.
    road_follow_fraction: float=.25
    load_feedback_gain: float=.3
    load_correction_limit_n: float=1500.
    support_target_ratio: float=.75
    contact_force_floor_fraction: float=.35
    contact_force_floor_n: float=1000.
    recontact_ff_scale: float=.2
    climb_ff_scale: float=.35
    descent_brake_gain: float=0.
    allocation_antiwindup_gain: float=0.
    airborne_sync_gain: float=0.
    airborne_sync_limit_nm: float=40.
    speed_tolerance_kph: float=.3
    max_speed_overshoot_kph: float=.8
    max_overspeed_duration_s: float=.15
    max_uncommanded_stall_s: float=8.

    def __post_init__(self):
        for f in fields(self):
            v=getattr(self,f.name)
            if f.name=='force_policy':
                if v not in ('legacy','phase'):raise ValueError('invalid event force policy')
            elif not math.isfinite(v) or v<0:raise ValueError('finite nonnegative event crawl settings required')
        if min(self.entry_kph,self.climb_kph,self.pit_kph,self.reference_acceleration_m_s2,
               self.reference_jerk_m_s3,self.braking_acceleration_m_s2,self.radius_m,self.suspension_bandwidth)<=0:
            raise ValueError('positive speed and shaping bounds required')
        if max(self.entry_kph,self.climb_kph,self.pit_kph)>3 or self.recontact_s>.5:
            raise ValueError('event crawl research envelope exceeded')
        if max(self.road_follow_fraction,self.support_target_ratio,self.contact_force_floor_fraction,self.recontact_ff_scale,self.climb_ff_scale)>1 or self.suspension_bandwidth>20 or self.suspension_preview_s>1:
            raise ValueError('invalid event force shaping bounds')
        if self.load_feedback_gain>2 or self.load_correction_limit_n>4000 or self.airborne_sync_limit_nm>100:
            raise ValueError('event research correction envelope exceeded')
        if not 0<self.entry_load_deviation<1 or self.descent_brake_gain>2:
            raise ValueError('invalid entry evidence or braking gain')


def four(value):
    import numpy as np
    a=np.asarray(value,float)
    if a.shape!=(4,) or not np.isfinite(a).all():raise ValueError('finite four-wheel SI values required')
    return a
