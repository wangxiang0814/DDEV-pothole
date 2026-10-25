"""Opt-in sampled, bounded load feedback; uses the explicit fast sensor delay.

The flat static load is a research reference, not a guarantee of feasible contact.
The existing body loop remains responsible for posture; no measured-force inversion.
"""
import math
import numpy as np
from .suspension_geometry import seat_kinematics


class FastLoadFeedback:
    def __init__(self,config):
        self.config=config;self.anchor=None;self.filtered=None

    def step(self,t,load,jounce,x,y,scene,dt):
        c=self.config;load=np.maximum(np.asarray(load,float),0)
        if load.shape!=(4,) or not np.isfinite(load).all() or dt<=0:raise ValueError('invalid load feedback')
        if t<c.settle_s:
            self.anchor=load.copy();self.filtered=load.copy()
            return np.zeros(4)
        if self.anchor is None:raise ValueError('fast load feedback requires static initialization')
        self.filtered+=(1-math.exp(-dt/c.fast_load_filter_s))*(load-self.filtered)
        _,mu=seat_kinematics(jounce)
        raw=c.fast_load_gain*(self.anchor-self.filtered)/mu
        start=scene['start_station_m'];end=start+scene['length_m']
        window=np.clip((np.asarray(x)-(start-.35))/.20,0,1)*np.clip((end+.55-np.asarray(x))/.20,0,1)
        window*=abs(np.asarray(y)-scene['center_y_m'])<scene['width_m']/2-scene['edge_transition_m']
        return window*np.clip(raw,-c.fast_load_limit_n,c.fast_load_limit_n)
