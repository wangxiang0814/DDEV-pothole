"""SI execution layer: sampled wheel feedback plus continuous command slew.

Motor sampling and feedback delay are explicit research assumptions. Integration
refinement changes neither control sample period. No tire-force cancellation.
"""
import math
import numpy as np
from .contact_kinematics import WheelMotion


class CrawlActuation:
    def __init__(self,config):
        self.config=config;self.command=np.zeros(9);self.motor_target=np.zeros(4)
        self.next_motor=0.;self.motor_ticks=0
        self.contact=np.zeros(4,bool);self.contact_since=np.full(4,np.nan)
        self.previous_slip=None;self.previous_slip_time=None;self.slip_acceleration=np.zeros(4)
        self.motion=WheelMotion(config.wheel_velocity_filter_s)
        self.last_slip=np.zeros(4);self.last_transport=np.zeros(4)
        self.suspension_target=np.zeros(4);self.passive_reference=None

    def step(self,t,stamp,feedback,target,origin,scene,dt,fault=False,reference_rate=None):
        c=self.config;target=np.array(target,float).copy()
        stale=fault or t-stamp>c.max_age_s
        if stale:
            target[:]=0
            self.previous_slip=None;self.previous_slip_time=None;self.slip_acceleration[:]=0
            self.motion.reset()
        if t+1e-8>=self.next_motor:
            self.next_motor+=c.motor_period_s*max(1,math.floor((t-self.next_motor+1e-8)/c.motor_period_s)+1)
            self.motor_ticks+=1;desired=target[:4].copy()
            need_motion=c.slip_frame=='hub_tangent' or c.suspension_mode!='legacy'
            transport=self.motion.observe(stamp,feedback) if need_motion and not stale else None
            self.suspension_target=target[4:8].copy()
            if c.suspension_mode!='legacy' and not stale and c.mode in ('full','suspension'):
                rate_ref=np.zeros(4) if reference_rate is None else np.asarray(reference_rate,float)
                fresh_groundhook=c.groundhook_gain*(self.motion.velocity[:,2]-rate_ref)
                if c.suspension_mode=='support':
                    passive=np.array([feedback['Fs_'+w]+feedback['Fd_'+w] for w in ('L1','R1','L2','R2')])
                    if t<c.settle_s or self.passive_reference is None:self.passive_reference=passive.copy()
                    self.suspension_target+=self.passive_reference-passive
                    fresh_groundhook/=np.asarray(c.motion_ratios)
                self.suspension_target+=fresh_groundhook
                if t<c.settle_s:self.suspension_target[:]=0.
                self.suspension_target=np.clip(self.suspension_target,-c.force_working_limit_n,c.force_working_limit_n)
            if c.mode in ('full','traction') and not stale:
                w=('L1','R1','L2','R2')
                radii=np.array([feedback['RRE_'+n+'i'] for n in w])
                spin=np.array([feedback['AVy_'+n] for n in w])
                # Outer speed PI owns target-speed tracking. This loop uses
                # measured translation to suppress tire slip, avoiding a second
                # hard wheel-speed brake when gravity accelerates the body.
                local=feedback['Vx']-feedback['AVz']*np.array([feedback['Y_'+n]-feedback['Yo'] for n in w])
                if c.slip_frame=='hub_tangent':local=transport
                slip=spin*radii-local
                self.last_slip=slip.copy();self.last_transport=local.copy()
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
                if c.motor_control_mode=='rate':
                    # Collocated low-bandwidth slip damping acts on torque rate.
                    # Anchor the state at realised torque, so slew clipping
                    # cannot wind up an inaccessible +/-500Nm target.
                    change=c.motor_torque_bandwidth*(nominal-self.command[:4])-c.motor_slip_rate_gain*slip
                    change=np.clip(change,-c.actuation_torque_rate_nm_s,c.actuation_torque_rate_nm_s)
                    desired=self.command[:4]+c.motor_period_s*change
                else:desired=np.clip(nominal+correction,-limit,limit)
            self.motor_target=desired
        if stale:self.motor_target[:]=0
        cap=np.array([c.torque_limit_nm]*4+[c.force_limit_n]*4+[math.radians(220)])
        rate=np.array([c.actuation_torque_rate_nm_s]*4+[c.force_rate_n_s]*4+[math.radians(400)])
        if stale:self.suspension_target[:]=0
        desired=np.clip(np.r_[self.motor_target,self.suspension_target,target[8]],-cap,cap)
        self.command=np.clip(self.command+np.clip(desired-self.command,-rate*dt,rate*dt),-cap,cap)
        return self.command.copy()
