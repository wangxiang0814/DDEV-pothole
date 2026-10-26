"""Causal reference shaping and spatial deceleration before every covered axle."""
import math
import numpy as np
from .event_config import four


def speed_ceiling(x,expected,scene,config,planning_margin_m=0.):
    x=four(x);expected=np.asarray(expected,bool)
    if expected.shape!=(4,):raise ValueError('four wheel mask required')
    if not math.isfinite(planning_margin_m) or planning_margin_m<0:raise ValueError('invalid planning margin')
    c=config;start=scene['start_station_m'];end=start+scene['length_m'];cap=math.inf
    margin=c.braking_margin_m+planning_margin_m
    relevant=x[expected]
    for station in relevant:
        if station>=end+c.radius_m:continue
        if station<start-margin:
            value=math.sqrt((c.entry_kph/3.6)**2+2*c.braking_acceleration_m_s2*(start-margin-station))
        elif station<start+c.radius_m:value=c.entry_kph/3.6
        elif station<end-c.radius_m:
            value=min(c.pit_kph/3.6,math.sqrt((c.climb_kph/3.6)**2+
                2*c.braking_acceleration_m_s2*max(end-c.radius_m-margin-station,0)))
        else:value=c.climb_kph/3.6
        cap=min(cap,value)
    if len(relevant) and relevant.max()>=start-c.braking_margin_m and relevant.min()<end+c.radius_m:
        cap=min(cap,c.pit_kph/3.6)
    return float(cap)


class EventSpeedPlanner:
    def __init__(self,config):self.config=config;self.value=0.;self.acceleration=0.

    def braking_margin_m(self):
        # Reserve for the reference filter (gain2/s) and positive-acceleration
        # jerk reversal. This changes planning only, never the assessment limit.
        horizon=1.+self.config.reference_acceleration_m_s2/self.config.reference_jerk_m_s3
        return self.value*horizon+.5*max(self.acceleration,0.)*horizon*horizon

    def step(self,dt,requested_m_s,ceiling_m_s):
        if not math.isfinite(dt) or not 0<dt<=.05 or not math.isfinite(requested_m_s) or requested_m_s<0 or math.isnan(ceiling_m_s) or ceiling_m_s<0:
            raise ValueError('invalid speed planner input')
        c=self.config;target=min(requested_m_s,ceiling_m_s)
        desired=float(np.clip(2*(target-self.value),-c.reference_acceleration_m_s2,c.reference_acceleration_m_s2))
        self.acceleration+=float(np.clip(desired-self.acceleration,-c.reference_jerk_m_s3*dt,c.reference_jerk_m_s3*dt))
        self.value=max(0.,self.value+self.acceleration*dt)
        return self.value
