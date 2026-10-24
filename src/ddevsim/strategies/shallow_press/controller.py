"""Expert two: spatial preview, bounded feedforward and classical feedback."""
import math
import numpy as np
from ddevsim.runtime.contracts import ControlCommand,StrategyStatus
from .config import ShallowPressConfig
from .preview import wheel_support_height,exit_relief,grade_torque_feedforward,climb_resistance_envelope
from .traction import allocate_torque

WHEELS=('L1','R1','L2','R2')
REQUIRED=('Xo','Yo','Zo','Vx','Roll_E','Pitch','Yaw','AVx','AVy','AVz','Az_SM')+tuple(
    p+w+s for p,s in (('X_',''),('Y_',''),('Jnc_',''),('JncR_',''),('Fz_',''),('Fx_',''),('CmpT_','i'),('AVy_',''),('RRE_','i')) for w in WHEELS)


class ShallowPressStrategy:
    implemented=True

    def __init__(self): self.context=None

    def reset(self,context):
        if context.strategy_id!='shallow_press' or context.vehicle_id!='tuned_ii_20261005' or 'scenario' not in context.config:
            raise ValueError('shallow_press requires same-vehicle scenario context')
        self.config=ShallowPressConfig(**dict(context.config.get('controller',{})))
        if abs(context.control_period_s-self.config.control_period_s)>1e-10: raise ValueError('control periods differ')
        self.scene=dict(context.config['scenario'])
        wheel_support_height(np.zeros((4,2)),self.scene)
        self.context=context; self.command=np.zeros(9); self.last_t=None; self.origin=None
        self.phase='SETTLE'; self.completed=False; self.failures=set(); self.records=[]
        self.j0=np.zeros(4); self.fz0=np.zeros(4); self.r=np.zeros(4); self.rd=np.zeros(4)
        self.filtered_load=np.zeros(4); self.filtered_vz=0.; self.previous_z=None
        self.integral=0.; self.speed_reference=0.; self.stop_since=None
        self.entered=np.zeros(4,bool); self.crossed=np.zeros(4,bool); self.last_x=None

    def _project(self,desired,dt):
        c=self.config
        cap=np.array([c.torque_limit_nm]*4+[c.force_limit_n]*4+[math.radians(220)])
        rate=np.array([c.torque_rate_nm_s]*4+[c.force_rate_n_s]*4+[math.radians(400)])
        self.command=np.clip(self.command+np.clip(desired-self.command,-rate*dt,rate*dt),-cap,cap)
        return ControlCommand(tuple(self.command[:4]),tuple(self.command[4:8]),float(self.command[8]))

    def step(self,observation):
        if self.context is None: raise RuntimeError('reset required')
        t=observation.time_s; c=self.config
        dt=c.control_period_s if self.last_t is None else t-self.last_t
        if dt<=0: raise ValueError('monotonic control time required')
        if dt>c.control_period_s*1.5: self.failures.add('CONTROL_GAP')
        self.last_t=t
        if not observation.valid or t-observation.observed_at_s>c.max_age_s:
            self.failures.add('STALE'); self.phase='FEEDBACK_FAULT'; self.integral=0
            self.previous_z=None
            return self._project(np.zeros(9),min(dt,c.control_period_s))
        observation.require(REQUIRED); x=observation.values_si
        get=lambda name:np.array([x[name+w] for w in WHEELS])
        xs=get('X_'); ys=get('Y_'); j=get('Jnc_'); jr=get('JncR_'); load=np.maximum(get('Fz_'),0)
        if self.origin is None:
            self.origin={n:x[n] for n in ('Yo','Zo','Roll_E','Pitch','Yaw')}
            self.last_x=xs.copy()
        if t<c.settle_s:
            self.j0=j.copy(); self.fz0=load.copy(); self.filtered_load=load.copy()
            self.origin={n:x[n] for n in self.origin}; self.previous_z=x['Zo']
            return self._project(np.zeros(9),dt)
        self.filtered_load+=(1-math.exp(-dt/.06))*(load-self.filtered_load)
        vz=0. if self.previous_z is None else (x['Zo']-self.previous_z)/dt
        self.previous_z=x['Zo']; self.filtered_vz+=(1-math.exp(-dt/.08))*(vz-self.filtered_vz)
        start=self.scene['start_station_m']; end=start+self.scene['length_m']
        covered=(abs(ys-self.scene['center_y_m'])<self.scene['width_m']/2-.05)
        self.entered|=covered&(self.last_x<start)&(xs>=start)
        self.crossed|=self.entered&(self.last_x<end+.263)&(xs>=end+.263)
        self.last_x=xs.copy()
        expected=np.array([False,True,False,True]) if self.context.scenario_id=='right_track' else np.ones(4,bool)
        behind=bool(np.all(self.crossed[expected]))
        rear_clear=bool(min(xs[2:])>end+.263+.7)
        if behind and rear_clear:
            self.phase='STOPPING'
            if abs(x['Vx'])<.025:
                if self.stop_since is None: self.stop_since=t
                if t-self.stop_since>1.5: self.completed=True; self.phase='COMPLETE'
            else: self.stop_since=None
        else: self.phase='CROSSING' if xs[:2].max()>start-.6 else 'APPROACH'
        target=c.cruise_kph/3.6
        if xs[:2].max()>start-1.0 and min(xs[2:])<end+.263+.3: target=min(target,c.crossing_kph/3.6)
        if behind and rear_clear: target=0.
        if max(abs(x['Roll_E']),abs(x['Pitch']))>math.radians(8):
            self.failures.add('ATTITUDE'); target=min(target,.15)
        self.speed_reference+=float(np.clip(target-self.speed_reference,-c.acceleration_m_s2*dt,c.acceleration_m_s2*dt))
        error=self.speed_reference-x['Vx']; proposed=self.integral+error*dt
        common=c.speed_kp*error+c.speed_ki*proposed
        if abs(common)<400 or common*error<0: self.integral=proposed
        total=4*float(np.clip(c.speed_kp*error+c.speed_ki*self.integral,-400,400))
        radii=np.clip(np.array([x['RRE_'+w+'i'] for w in WHEELS]),.18,.3)
        local_speed=x['Vx']-x['AVz']*(ys-x['Yo'])
        slip=get('AVy_')*radii-local_speed
        effective=np.where(np.array([x['CmpT_'+w+'i'] for w in WHEELS])>0,load,0)
        # Begin coasting before the short unloading/edge-contact transition;
        # finite torque slew cannot remove arbitrary torque instantaneously.
        edge_distance=abs(xs-start)
        coast=np.clip((edge_distance-.04)/.20,0,1)
        effective*=np.where(covered,coast,1.)
        velocity=max(x['Vx'],0)*np.array([math.cos(x['Yaw']),math.sin(x['Yaw'])])
        points=np.column_stack((xs,ys))+c.preview_s*velocity
        grade_torque=grade_torque_feedforward(points,self.scene,self.filtered_load,radii,c.grade_torque_gain)
        # Preview climbing resistance only. The existing speed feedback owns
        # downhill braking; signed grade FF previously overbraked entry.
        assist_weight=float(np.clip((self.speed_reference-x['Vx'])/max(.2*self.speed_reference,.03),0,1))
        resistance=climb_resistance_envelope(points,self.scene,self.filtered_load,radii,c.grade_torque_gain,
            c.climb_geometry_lag_m,[math.cos(x['Yaw']),math.sin(x['Yaw'])])
        climb_torque=resistance*assist_weight
        if c.mode in ('full','traction'):total+=float(climb_torque.sum())
        if c.mode in ('full','traction'):
            torques=allocate_torque(total,effective,slip,x['Yaw']-self.origin['Yaw'],x['AVz'],
                                    self.scene['friction'],radii,c.slip_gain)
        else: torques=np.full(4,total/4)
        geometric=wheel_support_height(points,self.scene)
        rdd=np.clip(c.reference_bandwidth**2*(geometric-self.r)-2*c.reference_bandwidth*self.rd,-1.5,1.5)
        self.rd=np.clip(self.rd+rdd*dt,-.25,.25); self.r+=self.rd*dt
        relative_x=xs-x['Xo']; relative_y=ys-x['Yo']
        corner_v=self.filtered_vz+x['AVx']*relative_y-x['AVy']*relative_x
        attitude=relative_y*(x['Roll_E']-self.origin['Roll_E'])-relative_x*(x['Pitch']-self.origin['Pitch'])
        ff=-c.spring_n_m*self.r-c.damper_n_s_m*self.rd-c.wheel_mass_kg*rdd
        ff+=exit_relief(points,self.scene,c.exit_relief_n)
        feedback=-c.height_gain*(x['Zo']-self.origin['Zo'])-c.attitude_gain*attitude-c.skyhook_gain*corner_v
        feedback+=c.travel_gain*(j-self.j0-self.r)+c.load_gain*(self.fz0-self.filtered_load)
        feedback+=c.groundhook_gain*(corner_v+jr-self.rd)
        force=np.clip(ff+feedback,-c.force_working_limit_n,c.force_working_limit_n)
        if c.mode in ('passive','traction'): force[:]=0
        steer=np.clip(-6.1*(x['Yo']-self.origin['Yo'])-12*(x['Yaw']-self.origin['Yaw'])-2*x['AVz'],
                      -math.radians(220),math.radians(220))
        command=self._project(np.r_[torques,force,steer],dt)
        self.records.append(dict(time_s=t,phase=self.phase,speed_reference_m_s=self.speed_reference,
            geometric_road_m=geometric.tolist(),road_reference_m=self.r.tolist(),
            grade_torque_feedforward_nm=grade_torque.tolist(),
            climb_torque_feedforward_nm=climb_torque.tolist(),
            exit_assist_weight=assist_weight,
            climb_resistance_estimate_nm=resistance.tolist(),
            feedforward_n=ff.tolist(),feedback_n=feedback.tolist(),slip_m_s=slip.tolist(),target_force_n=force.tolist()))
        return command

    def status(self):
        return StrategyStatus(self.phase,self.completed,next(iter(sorted(self.failures)),None)) if self.context is not None else StrategyStatus('NOT_INITIALIZED')

    def summary(self):
        if self.context is None: return dict(task_status='NOT_INITIALIZED')
        return dict(strategy_id='shallow_press',implemented=True,task_status='UNASSESSED',phase=self.phase,
            completed=self.completed,wheel_entered=self.entered.tolist(),wheel_crossed=self.crossed.tolist(),
            failure_reasons=sorted(self.failures),unassessed=['REAL_CHASSIS_GEOMETRY','REAL_ACTUATOR_POWER'])
