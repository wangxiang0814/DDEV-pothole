import math
import numpy as np
import pytest
from test_preview_control import observation,SCENE
from ddevsim.runtime.contracts import StrategyContext,Observation


def event_controller(**settings):
    from ddevsim.strategies.shallow_press.event_controller import EventCrawlStrategy
    s=EventCrawlStrategy();cfg=dict(policy='event_crawl',cruise_kph=3.,crossing_kph=1.,body_modal_blend=1.,body_leveling_s=2.,event=settings)
    s.reset(StrategyContext('shallow_press','tuned_ii_20261005','right_track',.02,dict(scenario=SCENE,controller=cfg),{}))
    return s


def test_event_controller_starts_static_and_brakes_for_the_rear_after_front_has_cleared():
    a=event_controller();b=event_controller()
    for t in np.arange(0,5,.02):a.step(observation(float(t)))
    assert a.speed_reference==0 and a.status().phase=='LEVELING'
    for t in np.arange(5,8,.02):a.step(observation(float(t),101.0))
    assert a.speed_reference<.16
    a.step(observation(8.,102.95))
    assert a.records[-1]['event']['speed_ceiling_m_s']<=.5/3.6+1e-9
    assert not a.status().completed
    assert b.step(observation(0.)).wheel_torque_nm==(0.,)*4


def test_event_commands_obey_baseline_limits_and_stale_feedback_unloads():
    a=event_controller(force_policy='phase');old=np.zeros(9)
    rates=np.array([1000.]*4+[45400.]*4+[math.radians(400)])
    for t in np.arange(0,8,.02):
        u=a.step(observation(float(t),101.2 if t>5 else 99.))
        value=np.r_[u.wheel_torque_nm,u.suspension_force_n,u.steering_wheel_angle_rad]
        assert np.max(abs(value[:4]))<=500 and np.max(abs(value[4:8]))<=18200
        assert np.all(abs(value-old)<=rates*.02+1e-7);old=value
    for t in np.arange(8.,8.5,.02):u=a.step(Observation(float(t),6.,False,{}))
    assert np.allclose(np.r_[u.wheel_torque_nm,u.suspension_force_n,u.steering_wheel_angle_rad],0)
    assert a.status().phase=='FEEDBACK_FAULT'


def test_event_configuration_is_frozen_and_legacy_cannot_silently_ignore_policy():
    from ddevsim.strategies.shallow_press.config import ShallowPressConfig
    from test_preview_control import controller
    settings={'entry_kph':.4};c=ShallowPressConfig(policy='event_crawl',event=settings);settings['entry_kph']=2.
    assert c.event.entry_kph==.4
    with pytest.raises(ValueError):controller(policy='event_crawl')
