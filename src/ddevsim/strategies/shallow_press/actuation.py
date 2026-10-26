"""SI execution layer: sampled wheel feedback plus continuous command slew.

Motor sampling and feedback delay are explicit research assumptions. Integration
refinement changes neither control sample period. No tire-force cancellation.
"""
import math
import numpy as np
from .fast_load import FastLoadFeedback


class CrawlActuation:
    def __init__(self,config):
        self.config=config;self.command=np.zeros(9);self.motor_target=np.zeros(4)
        self.next_motor=0.;self.motor_ticks=0
        self.contact=np.zeros(4,bool);self.contact_since=np.full(4,np.nan)
        self.previous_slip=None;self.previous_slip_time=None;self.slip_acceleration=np.zeros(4)
        self.load_loop=FastLoadFeedback(config) if config.fast_load_gain else None
        self.load_correction=np.zeros(4)

    def step(self,t,stamp,feedback,target,origin,scene,dt,fault=False,event_context=None):
        c=self.config;target=np.array(target,float).copy()
        stale=fault or t-stamp>c.max_age_s
        if stale:
            target[:]=0
            self.previous_slip=None;self.previous_slip_time=None;self.slip_acceleration[:]=0
            self.load_correction[:]=0
        if t+1e-8>=self.next_motor:
            self.next_motor+=c.motor_period_s*max(1,math.floor((t-self.next_motor+1e-8)/c.motor_period_s)+1)
            self.motor_ticks+=1;desired=target[:4].copy()
            if c.mode in ('full','traction') and not stale:
                w=('L1','R1','L2','R2')
                radii=np.array([feedback['RRE_'+n+'i'] for n in w])
                spin=np.array([feedback['AVy_'+n] for n in w])
                # Outer speed PI owns target-speed tracking. This loop uses
                # measured translation to suppress tire slip, avoiding a second
                # hard wheel-speed brake when gravity accelerates the body.
                # Measured rolling velocity; outer feedback owns yaw correction.
                local=feedback['Vx']-feedback['AVz']*np.array([feedback['Y_'+n]-feedback['Yo'] for n in w])
                slip=spin*radii-local
                elapsed=0. if self.previous_slip_time is None else t-self.previous_slip_time
                if self.previous_slip is None or not 0<elapsed<=2*c.motor_period_s:
                    self.slip_acceleration[:]=0
                else:
                    derivative=np.clip((slip-self.previous_slip)/elapsed,-50.,50.)
                    self.slip_acceleration+=(1-math.exp(-elapsed/.01))*(derivative-self.slip_acceleration)
                self.previous_slip=slip.copy();self.previous_slip_time=t
                load=np.array([feedback['Fz_'+n] for n in w])
                compression=np.array([feedback['CmpT_'+n+'i'] for n in w])
                was=self.contact.copy()
                self.contact=np.where((load<150)|(compression<=0),False,
                    np.where((load>300)&(compression>.0001),True,self.contact))
                self.slip_acceleration[was!=self.contact]=0.
                correction=-c.motor_slip_gain*slip-c.motor_slip_derivative*self.slip_acceleration
                self.contact_since=np.where(self.contact&~was,t,self.contact_since)
                self.contact_since=np.where(self.contact,self.contact_since,np.nan)
                restored=np.where(self.contact,np.clip((t-self.contact_since-.02)/.10,0,1),0)
                limit=np.where(self.contact,np.minimum(c.torque_limit_nm,.85*scene.get('friction',.7)*load*radii),0)
                # Ramp nominal drive/brake demand after contact restoration.
                # Slip correction must remain available during that ramp: a
                # spinning wheel cannot wait120ms for braking to be restored.
                # Total corrective torque still obeys load, amplitude and slew.
                nominal=np.clip(desired,-limit*restored,limit*restored)
                desired=np.clip(nominal+correction,-limit,limit)
                if c.policy=='event_crawl' and event_context is not None:
                    phases=event_context.get('phases',[])
                    if len(phases)!=4:raise ValueError('event motor context requires four phases')
                    air=np.array([p not in ('BYPASS','APPROACH','RECOVERED') for p in phases])&~self.contact
                    sync_limit=min(c.torque_limit_nm,c.event.airborne_sync_limit_nm,float(event_context.get('airborne_sync_limit_nm',0)))
                    sync=np.clip(-float(event_context.get('airborne_sync_gain',0))*slip,-sync_limit,sync_limit)
                    desired=np.where(air,sync,desired)
            self.motor_target=desired
            if self.load_loop is not None and c.mode in ('full','suspension') and not stale:
                w=('L1','R1','L2','R2')
                get=lambda p:np.array([feedback[p+n] for n in w])
                self.load_correction=self.load_loop.step(t,get('Fz_'),get('Jnc_'),get('X_'),get('Y_'),scene,c.motor_period_s)
        if stale:self.motor_target[:]=0
        if self.load_loop is not None and not stale:
            target[4:8]=np.clip(target[4:8]+self.load_correction,-c.force_working_limit_n,c.force_working_limit_n)
        cap=np.array([c.torque_limit_nm]*4+[c.force_limit_n]*4+[math.radians(220)])
        rate=np.array([c.actuation_torque_rate_nm_s]*4+[c.force_rate_n_s]*4+[math.radians(400)])
        desired=np.clip(np.r_[self.motor_target,target[4:]],-cap,cap)
        self.command=np.clip(self.command+np.clip(desired-self.command,-rate*dt,rate*dt),-cap,cap)
        return self.command.copy()
