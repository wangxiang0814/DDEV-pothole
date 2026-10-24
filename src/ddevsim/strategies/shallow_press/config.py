"""Shallow controller settings are independent of wheel-lift settings."""
from dataclasses import dataclass,fields
import math
from typing import Optional,Tuple


@dataclass(frozen=True)
class ShallowPressConfig:
    control_period_s: float = .02
    cruise_kph: float = 2.
    crossing_kph: float = 1.
    settle_s: float = 3.
    preview_s: float = .2
    torque_preview_s: Optional[float] = None
    preview_schedule: Tuple[Tuple[float,float,float],...] = ()
    reference_bandwidth: float = 8.
    spring_n_m: float = 5000.
    damper_n_s_m: float = 0.
    wheel_mass_kg: float = 40.
    height_gain: float = 8000.
    attitude_gain: float = 10000.
    attitude_damping_gain: Optional[float] = None
    skyhook_gain: float = 2000.
    body_modal_blend: float = 0.
    body_leveling_s: float = 0.
    travel_gain: float = 0.
    load_gain: float = 0.
    speed_kp: float = 640.
    speed_ki: float = 60.
    slip_gain: float = 0.
    motor_slip_gain: float = 100.
    motor_slip_derivative: float = 0.
    motor_period_s: float = .002
    grade_torque_gain: float = 1.
    climb_geometry_lag_m: float = .15
    groundhook_gain: float = 2000.
    exit_relief_n: float = 0.
    acceleration_m_s2: float = .16
    force_working_limit_n: float = 3500.
    force_limit_n: float = 18200.
    force_rate_n_s: float = 45400.
    torque_limit_nm: float = 500.
    torque_rate_nm_s: float = 1000.
    diagnostic_torque_rate_nm_s: Optional[float] = None
    max_age_s: float = .1
    mode: str = 'full'

    def __post_init__(self):
        for item in fields(self):
            value=getattr(self,item.name)
            if item.name=='preview_schedule':
                try:rows=tuple(tuple(float(v) for v in row) for row in value)
                except (TypeError,ValueError):raise ValueError('invalid preview schedule')
                if any(len(row)!=3 or not all(math.isfinite(v) for v in row) or
                       not(0<row[0]<=10 and 0<=row[1]<=1 and 0<row[2]<=20) for row in rows):
                    raise ValueError('invalid preview schedule values')
                if any(a[0]>=b[0] for a,b in zip(rows,rows[1:])):raise ValueError('preview schedule speed must increase')
                object.__setattr__(self,'preview_schedule',rows)
            elif item.name=='mode':
                if value not in ('full','passive','traction','suspension'): raise ValueError('invalid mode')
            elif item.name in ('torque_preview_s','attitude_damping_gain','diagnostic_torque_rate_nm_s') and value is None:continue
            elif not math.isfinite(value) or value<0: raise ValueError('invalid '+item.name)
        if abs(self.control_period_s-.02)>1e-10 or not 0<self.crossing_kph<=self.cruise_kph<=10:
            raise ValueError('invalid speed or period')
        if self.settle_s<2 or self.max_age_s<=0 or self.acceleration_m_s2<=0:
            raise ValueError('static calibration window, observation age and acceleration must be positive')
        if not 0<self.reference_bandwidth<=20 or not 0<self.force_working_limit_n<=self.force_limit_n<=18200:
            raise ValueError('invalid force or reference bandwidth')
        if not 0<self.torque_limit_nm<=500 or not 0<self.force_rate_n_s<=45400 or not 0<self.torque_rate_nm_s<=1000:
            raise ValueError('aligned capability envelope exceeded')
        if not .001<=self.motor_period_s<=.01:raise ValueError('invalid motor sample period')
        if self.climb_geometry_lag_m>.263:raise ValueError('exit uncertainty exceeds tire radius')
        if self.body_modal_blend>1:raise ValueError('body modal blend must be within0..1')
        if self.motor_slip_derivative>20:raise ValueError('motor slip derivative exceeds research envelope')
        if self.body_leveling_s and (not 1<=self.body_leveling_s<=10 or self.body_modal_blend!=1):
            raise ValueError('body leveling needs full modal feedback and1..10s transition')
        if self.diagnostic_torque_rate_nm_s is not None and not 1000<self.diagnostic_torque_rate_nm_s<=50000:
            raise ValueError('diagnostic torque slew must be above baseline and at most50000')

    @property
    def actuation_torque_rate_nm_s(self):
        return self.diagnostic_torque_rate_nm_s or self.torque_rate_nm_s

    def capability_metadata(self):
        diagnostic=self.diagnostic_torque_rate_nm_s is not None
        return dict(diagnostic=diagnostic,baseline_torque_rate_nm_s=self.torque_rate_nm_s,
            actual_torque_rate_nm_s=self.actuation_torque_rate_nm_s,torque_limit_nm=self.torque_limit_nm,
            force_limit_n=self.force_limit_n,force_rate_n_s=self.force_rate_n_s,
            scope='AUTHORIZED_INDEPENDENT_SCAN' if diagnostic else 'EXPERT_ONE_ALIGNED_SOFTWARE_BASELINE',
            physical_hardware_validated=False)


DEFAULT_CONFIG = ShallowPressConfig()
