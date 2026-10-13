"""Small independent identification excitations and causal preview for research."""
import math
import numpy as np
from ddevsim.chassis_contact import road_surface


WHEELS = ('L1','R1','L2','R2')


class ValidationExcitation:
    def __init__(self,names,*,speed_kph,seed,excite,end_station_m,step_s=.0005,amplitude_n=350):
        self.names = tuple(names)
        self.index = {n:i for i,n in enumerate(names)}
        self.speed = float(speed_kph)/3.6
        self.step_s = float(step_s)
        self.end_station_m = float(end_station_m)
        self.excite = bool(excite)
        self.amplitude = float(amplitude_n)
        self.integral = 0.
        self.command = (0.,)*9
        self.last_tick = None
        self.records = []
        self.command_history = []
        self.completed = False
        self.attitude_exceeded = False
        self.y0 = None
        rng = np.random.RandomState(seed)
        self.phases = rng.uniform(-math.pi,math.pi,(8,4))
        self.frequencies = rng.uniform(.7,1.3,(8,4))*np.array([.25,.7,1.6,3.1])

    def __call__(self,t,exports):
        if self.last_tick is not None and t-self.last_tick < .02-1e-8:
            return self.command
        dt = .02 if self.last_tick is None else t-self.last_tick
        self.last_tick = t
        x = {n:float(exports[i]) for n,i in self.index.items()}
        if not all(math.isfinite(v) for v in x.values()):
            raise ValueError('nonfinite native feedback')
        if self.y0 is None:
            self.y0 = x['Yo']
        self.attitude_exceeded |= max(abs(x['Roll_E']),abs(x['Pitch'])) > 20
        self.completed = min(x['X_L2'],x['X_R2']) > self.end_station_m
        ramp = np.clip((t-5)/3,0,1)
        target = 0. if self.completed else self.speed*ramp
        error = target-x['Vx']/3.6
        wanted = 220*error+100*self.integral
        if abs(wanted)<450 or wanted*error<0:
            self.integral += error*dt
        common = np.clip(220*error+100*self.integral,-450,450)
        signal = np.zeros(8)
        if self.excite and t>=5 and not self.completed:
            phase = 2*math.pi*self.frequencies*(t-5)+self.phases
            signal = np.sin(phase).mean(axis=1)*np.clip((t-5)/1.5,0,1)
        torques = common+20*signal[:4]
        forces = self.amplitude*signal[4:]
        desired_steer = np.clip(-350*(x['Yo']-self.y0)-12*x['Yaw']-2*x['AVz'],-220,220)
        if t<5:
            torques[:]=0
            forces[:]=0
            desired_steer=0
            self.integral=0
        desired = np.r_[torques,forces,desired_steer]
        limits = np.array([500.]*4+[18200.]*4+[220.])
        rates = np.array([1000.]*4+[45400.]*4+[400.])
        old = np.asarray(self.command)
        command = np.clip(old+np.clip(desired-old,-rates*dt,rates*dt),-limits,limits)
        self.command = tuple(command)
        self.command_history.append(self.command)
        row = {'observed_time_s': max(0.,t-self.step_s),'command_start_s': t}
        row.update({'exp_'+n:v for n,v in x.items()})
        input_names = tuple('IMP_MYUSM_'+w for w in WHEELS)+tuple('IMP_FS_'+w for w in WHEELS)+('IMP_STEER_SW',)
        row.update({'imp_'+n:float(v) for n,v in zip(input_names,command)})
        self.records.append(row)
        return self.command


def preview_road(start_xy,*,speed_mps,dt,steps,scene):
    xy = np.asarray(start_xy,dtype=float)
    if xy.shape!=(4,2) or not np.isfinite(xy).all() or not np.isfinite(speed_mps):
        raise ValueError('finite four wheel positions and speed required')
    if steps<1 or dt<=0:
        raise ValueError('positive preview horizon required')
    points = np.tile(xy,(steps+1,1,1))
    points[:,:,0] += (np.arange(steps+1)*dt*speed_mps)[:,None]
    return road_surface(points,scene)[0]
