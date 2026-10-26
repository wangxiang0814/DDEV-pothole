"""Measured per-wheel contact branches over spatial pit phases, no timer script."""
import numpy as np
from .event_config import four


class WheelPhaseTracker:
    def __init__(self,config):
        self.config=config;self.last_t=None;self.last_x=None
        self.contact=np.zeros(4,bool);self.restored_at=np.full(4,-np.inf)
        self.entered=np.zeros(4,bool);self.crossed=np.zeros(4,bool)
        self.expected=None;self.base=['APPROACH']*4

    def update(self,t,x,y,fz,compression,scene):
        if not np.isfinite(t) or (self.last_t is not None and t<=self.last_t):raise ValueError('monotonic phase time required')
        x=four(x);y=four(y);fz=four(fz);compression=four(compression);c=self.config
        if self.expected is None:self.expected=abs(y-scene['center_y_m'])<scene['width_m']/2-scene['edge_transition_m']
        expected=self.expected
        start=scene['start_station_m'];end=start+scene['length_m']
        was=self.contact.copy()
        self.contact=np.where((fz<150)|(compression<=0),False,
            np.where((fz>300)&(compression>.0001),True,self.contact))
        if self.last_t is not None:
            self.restored_at=np.where(self.contact&~was,t,self.restored_at)
            self.entered|=expected&(self.last_x<start)&(x>=start)
            self.crossed|=self.entered&(self.last_x<end+c.radius_m)&(x>=end+c.radius_m)
        base=[];phases=[]
        for i in range(4):
            if not expected[i]:label='BYPASS'
            elif x[i]>=end+c.radius_m:label='RECOVERED'
            elif x[i]>=end-c.radius_m:label='CLIMB'
            elif x[i]>=start+c.radius_m:label='BOTTOM'
            elif x[i]>=start:label='ENTRY'
            elif x[i]>=start-c.prepare_distance_m:label='PREPARE'
            else:label='APPROACH'
            if expected[i] and self.last_t is not None and label!=self.base[i]:
                levels=['APPROACH','PREPARE','ENTRY','BOTTOM','CLIMB','RECOVERED']
                old=levels.index(self.base[i]);new=levels.index(label)
                if abs(old-new)==1:
                    boundary=[start-c.prepare_distance_m,start,start+c.radius_m,end-c.radius_m,end+c.radius_m][min(old,new)]
                    if abs(x[i]-boundary)<c.phase_hysteresis_m:label=self.base[i]
            base.append(label)
            if expected[i] and label not in ('APPROACH','RECOVERED'):
                phases.append('UNLOADED' if not self.contact[i] else
                    'RECONTACT' if t-self.restored_at[i]<c.recontact_s else label)
            else:phases.append(label)
        self.last_t=float(t);self.last_x=x.copy();self.base=base.copy()
        return dict(phases=phases,base_phases=base,contact=self.contact.tolist(),
            expected=expected.tolist(),entered=self.entered.tolist(),crossed=self.crossed.tolist())
