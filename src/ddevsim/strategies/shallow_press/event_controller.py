"""Event-based classic crawl; legacy primitives retained through explicit hooks."""
import numpy as np
from .controller import ShallowPressStrategy,REQUIRED,WHEELS
from .event_phases import WheelPhaseTracker
from .event_speed import EventSpeedPlanner,speed_ceiling
from .suspension_geometry import seat_kinematics


class EventCrawlStrategy(ShallowPressStrategy):
    supports_event_policy=True

    def reset(self,context):
        super().reset(context)
        if self.config.policy!='event_crawl':raise ValueError('explicit event_crawl policy required')
        e=self.config.event;self.tracker=WheelPhaseTracker(e,context.scenario_id);self.planner=EventSpeedPlanner(e)
        self.preview_s=e.suspension_preview_s;self.reference_bandwidth=e.suspension_bandwidth
        self.wheel_state=None;self.motor_context=None;self.event_record={};self.ceiling=float('inf')
        self.speed_target=0.;self.force_components={}

    def step(self,observation):
        c=self.config
        if observation.valid and observation.time_s-observation.observed_at_s<=c.max_age_s:
            observation.require(REQUIRED);x=observation.values_si
            get=lambda p:np.array([x[p+w] for w in WHEELS])
            self.wheel_state=self.tracker.update(observation.time_s,get('X_'),get('Y_'),get('Fz_'),
                np.array([x['CmpT_'+w+'i'] for w in WHEELS]),self.scene)
        before=len(self.records);command=super().step(observation)
        if len(self.records)>before:
            self.records[-1]['event']=dict(**self.wheel_state,speed_ceiling_m_s=None if not np.isfinite(self.ceiling) else self.ceiling,
                speed_target_m_s=self.speed_target,speed_reference_acceleration_m_s2=self.planner.acceleration,
                force_components=self.force_components,drive=self.event_record,integral=self.integral)
        self.motor_context=None if self.phase=='FEEDBACK_FAULT' else dict(
            phases=[] if self.wheel_state is None else self.wheel_state['phases'],
            airborne_sync_gain=c.event.airborne_sync_gain,airborne_sync_limit_nm=c.event.airborne_sync_limit_nm)
        return command

    def _target_speed(self,target,x,xs,t,dt,finished):
        if finished or self.phase=='LEVELING':self.ceiling=0.;self.speed_target=0.;return 0.
        self.ceiling=speed_ceiling(xs,self.wheel_state['expected'],self.scene,self.config.event,
            planning_margin_m=self.planner.braking_margin_m())
        self.speed_target=min(self.config.cruise_kph/3.6,self.ceiling)
        return self.speed_target

    def _advance_speed(self,target,dt):
        self.speed_reference=self.planner.step(dt,target,float('inf'))

    def _drive_request(self,total,grade_torque,x,dt):
        e=self.config.event;phase=np.array(self.wheel_state['base_phases'])
        braking=e.descent_brake_gain*float(np.minimum(grade_torque,0)[phase=='ENTRY'].sum())
        self.event_record=dict(drive_requested_nm=float(total),descent_brake_nm=braking)
        return total+braking

    def _on_allocated(self,total,torques,dt):
        c=self.config
        if c.speed_ki and c.event.allocation_antiwindup_gain:
            self.integral+=c.event.allocation_antiwindup_gain*(float(torques.sum())-total)*dt/(4*c.speed_ki)
        self.event_record.update(drive_total_request_nm=float(total),allocated_total_nm=float(torques.sum()))

    def _force_request(self,ff,feedback,x,j,load,dt):
        c=self.config;e=c.event
        if e.force_policy=='legacy':self.force_components={};return ff+feedback
        phases=self.wheel_state['phases'];base=self.wheel_state['base_phases']
        weights=np.array([e.recontact_ff_scale if p=='RECONTACT' else e.climb_ff_scale if b=='CLIMB' else 1. for p,b in zip(phases,base)])
        staged_ff=ff*weights
        body_reference=e.road_follow_fraction*float(np.mean(self.r))
        body_feedback=feedback+c.height_gain*body_reference
        _,mu=seat_kinematics(j)
        active=np.array([p in ('ENTRY','BOTTOM','CLIMB','UNLOADED','RECONTACT') for p in phases])
        correction=active*np.clip(e.load_feedback_gain*(e.support_target_ratio*self.fz0-self.filtered_load)/mu,
            -e.load_correction_limit_n,e.load_correction_limit_n)
        request=staged_ff+body_feedback+correction
        floor=np.array([min(e.contact_force_floor_n,e.contact_force_floor_fraction*max(staged_ff[i],0))
            if p=='UNLOADED' or (p=='ENTRY' and x['X_'+WHEELS[i]]>=self.scene['start_station_m'])
            else -c.force_working_limit_n for i,p in enumerate(phases)])
        request=np.maximum(request,floor)
        self.force_components=dict(feedforward_n=staged_ff.tolist(),body_feedback_n=body_feedback.tolist(),
            load_correction_n=correction.tolist(),unclipped_n=request.tolist(),body_reference_m=body_reference)
        return request

    def summary(self):
        summary=super().summary()
        summary.update(policy='event_crawl',task_version='event_crawl_v1',
            wheel_state=self.wheel_state,event_settings=self.config.event.__dict__.copy())
        return summary
