"""Measured per-wheel contact branches over spatial pit phases, no timer script."""
import numpy as np
from .event_config import four


def required_wheels(case_id):
    if case_id not in ('right_track','wide'):raise ValueError('explicit supported event case required')
    return np.array([False,True,False,True]) if case_id=='right_track' else np.ones(4,bool)


class WheelPhaseTracker:
    def __init__(self,config,case_id):
        self.config=config;self.last_t=None;self.last_x=None
        self.contact=np.zeros(4,bool);self.restored_at=np.full(4,-np.inf)
        self.entered=np.zeros(4,bool);self.crossed=np.zeros(4,bool)
        self.expected=required_wheels(case_id);self.base=['APPROACH']*4
        self.flat_load=None;self.entry_evidence=np.zeros(4,bool)

    def update(self,t,x,y,fz,compression,scene):
        if not np.isfinite(t) or (self.last_t is not None and t<=self.last_t):raise ValueError('monotonic phase time required')
        x=four(x);y=four(y);fz=four(fz);compression=four(compression);c=self.config
        expected=self.expected
        start=scene['start_station_m'];end=start+scene['length_m']
        if self.flat_load is None:self.flat_load=fz.copy()
        # Anchor only on undisturbed road. A contact edge can reach the tire
        # before the hub crosses the surveyed lip; retain this measured branch
        # through load oscillation but never use it to claim geometric progress.
        flat=x<start-c.prepare_distance_m
        self.flat_load=np.where(flat,.9*self.flat_load+.1*fz,self.flat_load)
        self.entry_evidence=np.where(x<start-c.radius_m-c.phase_hysteresis_m,False,self.entry_evidence)
        near=(x>=start-c.radius_m)&(x<start+c.radius_m)
        evidence=abs(fz-self.flat_load)>c.entry_load_deviation*np.maximum(self.flat_load,300.)
        self.entry_evidence|=expected&near&evidence
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
            if expected[i] and self.entry_evidence[i] and label=='PREPARE':label='ENTRY'
            base.append(label)
            if expected[i] and label not in ('APPROACH','RECOVERED'):
                phases.append('UNLOADED' if not self.contact[i] else
                    'RECONTACT' if t-self.restored_at[i]<c.recontact_s else label)
            else:phases.append(label)
        self.last_t=float(t);self.last_x=x.copy();self.base=base.copy()
        return dict(phases=phases,base_phases=base,contact=self.contact.tolist(),
            expected=expected.tolist(),entered=self.entered.tolist(),crossed=self.crossed.tolist(),
            entry_load_evidence=self.entry_evidence.tolist())
