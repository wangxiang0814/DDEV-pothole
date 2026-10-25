"""Causal native/SI bridge; the strategy never consumes native units."""
from collections import deque
import math
import re
import numpy as np
from ddevsim.runtime.contracts import Observation
from ddevsim.units import si_factor
from .controller import REQUIRED
from .config import ShallowPressConfig
from .actuation import CrawlActuation
from .contact_kinematics import HUB_REQUIRED


def low_speed_comparison(text,speed_m_s):
    """Independent low-speed tyre variant; shared legacy recipe stays exact."""
    if not math.isfinite(speed_m_s) or not 0<speed_m_s<=2:
        raise ValueError('invalid low speed regularizer')
    for key in ('VLOW_KAPPA','VLOW_ALPHA'):
        text,count=re.subn(r'(?m)^'+key+r'[ \t]+0\.2[ \t]*$',
                           key+' '+format(speed_m_s*3.6,'.9g'),text)
        if count!=4:raise ValueError('expected legacy four-tire low speed values')
    return text


def contact_patch_comparison(text,length_m):
    """Optional independent plant, NOT a correction to the shared vehicle.

    VS Tire Models (2018), pp48-49: two longitudinal road samples supply
    averaged height and secant slope. Length is a research assumption.
    """
    if not math.isfinite(length_m) or not 0<=length_m<=.3:
        raise ValueError('invalid comparison contact length')
    if length_m==0:return text
    # The solver echo declares this configurable-function length in mm.
    result,count=re.subn(r'(?m)^L_CONTACT_X_CONSTANT[ \t]+0[ \t]*$',
                         'L_CONTACT_X_CONSTANT '+format(length_m*1000,'.9g'),text)
    if count!=4:raise ValueError('expected four point contact tires')
    return result


def release_constant_brake(text):
    pattern=r'(ENTER_PARSFILE Procedures\\[^\n]+\n)(.*?)(EXIT_PARSFILE Procedures\\[^\n]+)'
    def modify(match):
        body,count=re.subn(r'(?m)^PBK_CON_CONSTANT[ \t]+1[ \t]*$','PBK_CON_CONSTANT 0',match.group(2))
        if count!=1:raise ValueError('expected one inherited constant brake')
        return match.group(1)+body+match.group(3)
    amended,count=re.subn(pattern,modify,text,flags=re.S)
    if count!=1:raise ValueError('expected one independent procedure')
    return amended


class NativePreviewBridge:
    def __init__(self,strategy,exports,step_s=.0005,delay_s=0.,motor_delay_s=.002):
        if not 0<step_s<=.001 or not 0<=delay_s<=.08 or not 0<=motor_delay_s<=.08: raise ValueError('invalid time settings')
        self.strategy=strategy; self.names=tuple(exports); self.index={n:i for i,n in enumerate(exports)}
        self.config=getattr(strategy,'config',ShallowPressConfig())
        extra=HUB_REQUIRED if self.config.slip_frame=='hub_tangent' or self.config.suspension_mode!='legacy' else ()
        if self.config.suspension_mode=='support':extra+=tuple(p+w for p in ('Fs_','Fd_') for w in ('L1','R1','L2','R2'))
        self.factors={n:si_factor(n) for n in tuple(dict.fromkeys(REQUIRED+extra))}
        if any(n not in self.index for n in self.factors): raise ValueError('missing native observation')
        self.step_s=step_s; self.delay=delay_s; self.history=deque(); self.last_tick=None
        self.motor_history=deque();self.motor_delay=motor_delay_s
        self.command=(0.,)*9; self.outer_command=(0.,)*9; self.rows=[]
        self.actuation=CrawlActuation(self.config);self.last_time=None

    def __call__(self,t,exports):
        observed=max(0,t-self.step_s)
        values={n:float(exports[self.index[n]])*f for n,f in self.factors.items()}
        self.history.append((observed,values))
        self.motor_history.append((observed,values))
        while len(self.history)>1 and self.history[1][0]<=observed-self.delay+1e-8:self.history.popleft()
        while len(self.motor_history)>1 and self.motor_history[1][0]<=observed-self.motor_delay+1e-8:self.motor_history.popleft()
        stamp,feedback=self.history[0]
        motor_stamp,motor_feedback=self.motor_history[0]
        outer=self.last_tick is None or observed-self.last_tick>=self.config.control_period_s-1e-8
        if outer:
            self.last_tick=observed
            c=self.strategy.step(Observation(observed,stamp,True,feedback))
            self.outer_command=tuple(c.wheel_torque_nm)+tuple(c.suspension_force_n)+(c.steering_wheel_angle_rad,)
        dt=self.step_s if self.last_time is None else t-self.last_time
        if dt<=0:raise ValueError('monotonic native time required')
        self.last_time=t
        applied=self.actuation.step(observed,motor_stamp,motor_feedback,self.outer_command,
            getattr(self.strategy,'origin',None),
            getattr(self.strategy,'scene',{}),dt,getattr(self.strategy,'phase','')=='FEEDBACK_FAULT',
            reference_rate=getattr(self.strategy,'rd',None))
        self.command=tuple(applied[:8])+(math.degrees(applied[8]),)
        if outer:
            row=dict(observed_time_s=observed,command_start_s=observed,command_end_s=t,
                     feedback_time_s=stamp,motor_feedback_time_s=motor_stamp)
            row.update({'exp_'+n:float(exports[i]) for i,n in enumerate(self.names)})
            names=tuple('IMP_MYUSM_'+w for w in ('L1','R1','L2','R2'))+tuple('IMP_FS_'+w for w in ('L1','R1','L2','R2'))+('IMP_STEER_SW',)
            row.update({'imp_'+n:v for n,v in zip(names,self.command)})
            self.rows.append(row)
        return self.command
