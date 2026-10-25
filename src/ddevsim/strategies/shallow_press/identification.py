"""Bounded stationary diagnostics; these do not establish hardware ratings."""
import math
import numpy as np


def extension_headroom(settled_jounce_m,reserve_m=.02,minimum_jounce_m=-.149):
    j=np.asarray(settled_jounce_m,float)
    if j.shape!=(4,) or not np.isfinite(j).all() or not math.isfinite(reserve_m) or reserve_m<0:
        raise ValueError('finite four-corner settled jounce and nonnegative reserve required')
    if not math.isfinite(minimum_jounce_m):raise ValueError('finite rebound bound required')
    room=j-minimum_jounce_m
    return dict(corner_headroom_m=room.tolist(),minimum_headroom_m=float(room.min()),
        minimum_with_reserve_m=float(room.min()-reserve_m),reserve_m=reserve_m,
        physical_hardware_rating=False,assumption='Fixed local body pose and unchanged tire compression')


class RateBoundedStrokeProbe:
    """Four small signed pulses then a slow common-force ramp and recovery.

    A reach/posture/load guard ends force increase, never the native simulation.
    Its trigger is a finite experiment boundary, not maximum physical stroke.
    """
    rate_n_s=1000.

    def __init__(self):
        self.command=np.zeros(4);self.last_t=None;self.guard_reason=None

    def step(self,t,jounce_m,loads_n,roll_rad,pitch_rad):
        dt=.02 if self.last_t is None else t-self.last_t
        state=np.r_[t,jounce_m,loads_n,roll_rad,pitch_rad]
        if not np.isfinite(state).all() or dt<=0:raise ValueError('finite monotonic probe feedback required')
        self.last_t=t;desired=np.zeros(4)
        if 3<=t<15:
            corner=min(3,int((t-3)//3));phase=(t-3)%3
            desired[corner]=np.interp(phase,[0,.5,1,1.5,2,2.5,3],[0,400,400,0,-400,-400,0])
        elif 15<=t<29:desired[:]=min(4000.,400*(t-15))
        elif 29<=t<39:desired[:]=max(0.,4000.-400*(t-29))
        if t>=15 and self.guard_reason is None:
            if np.min(jounce_m)<=-.125:self.guard_reason='EXTENSION_RESERVE'
            elif np.min(loads_n)<300:self.guard_reason='CONTACT_LOAD'
            elif max(abs(roll_rad),abs(pitch_rad))>math.radians(5):self.guard_reason='BODY_POSTURE'
        if self.guard_reason:desired[:]=0
        self.command+=np.clip(desired-self.command,-self.rate_n_s*dt,self.rate_n_s*dt)
        return self.command.copy()
