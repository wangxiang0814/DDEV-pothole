"""Comfort-first event crawl assumptions; independent of expert one."""
from dataclasses import dataclass,fields
import math


@dataclass(frozen=True)
class EventCrawlConfig:
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
    radius_m: float=.263

    def __post_init__(self):
        if any(not math.isfinite(getattr(self,f.name)) or getattr(self,f.name)<=0 for f in fields(self)):
            raise ValueError('positive finite event crawl settings required')
        if max(self.entry_kph,self.climb_kph,self.pit_kph)>3 or self.recontact_s>.5:
            raise ValueError('event crawl research envelope exceeded')


def four(value):
    import numpy as np
    a=np.asarray(value,float)
    if a.shape!=(4,) or not np.isfinite(a).all():raise ValueError('finite four-wheel SI values required')
    return a
