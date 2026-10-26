import numpy as np
from test_preview_control import SCENE


def test_speed_envelope_brakes_before_front_and_rear_edges_without_using_bypass_wheels():
    from ddevsim.strategies.shallow_press.event_speed import speed_ceiling
    from ddevsim.strategies.shallow_press.event_config import EventCrawlConfig
    c=EventCrawlConfig();expected=np.array([False,True,False,True])
    far=speed_ceiling(np.array([99.]*4),expected,SCENE,c)
    front=speed_ceiling(np.array([99,101.,99,99.075]),expected,SCENE,c)
    rear=speed_ceiling(np.array([99,102.95,99,101.025]),expected,SCENE,c)
    assert far>front and front<=c.entry_kph/3.6+1e-9 and rear<=c.entry_kph/3.6+1e-9
    assert speed_ceiling(np.array([101.2,99,101.2,99]),expected,SCENE,c)==far


def test_reference_acceleration_jerk_and_state_are_independent():
    from ddevsim.strategies.shallow_press.event_speed import EventSpeedPlanner
    from ddevsim.strategies.shallow_press.event_config import EventCrawlConfig
    c=EventCrawlConfig();a=EventSpeedPlanner(c);b=EventSpeedPlanner(c);last_a=0.;last_v=0.
    for i in range(500):
        v=a.step(.02,1.,.2 if i>250 else 1.)
        assert v>=0 and abs(a.acceleration)<=c.reference_acceleration_m_s2+1e-8
        assert abs(a.acceleration-last_a)<=c.reference_jerk_m_s3*.02+1e-8
        assert abs(v-last_v)<=c.reference_acceleration_m_s2*.02+1e-8
        last_a=a.acceleration;last_v=v
    assert b.value==0 and v<.22


def test_ideal_vehicle_can_obey_the_independent_envelope_after_jerk_shaping():
    from ddevsim.strategies.shallow_press.event_speed import EventSpeedPlanner,speed_ceiling
    from ddevsim.strategies.shallow_press.event_config import EventCrawlConfig
    from ddevsim.strategies.shallow_press.event_assessment import longest_interval
    c=EventCrawlConfig();planner=EventSpeedPlanner(c);station=SCENE['start_station_m']-3.;excess=[]
    for _ in range(6000):
        xs=np.array([station,station,station-1.925,station-1.925]);expected=np.array([False,True,False,True])
        independent=speed_ceiling(xs,expected,SCENE,c)
        planned=speed_ceiling(xs,expected,SCENE,c,planning_margin_m=planner.braking_margin_m())
        speed=planner.step(.02,2/3.6,planned);station+=speed*.02
        if xs[1]>SCENE['start_station_m']-.5 and xs[3]<SCENE['start_station_m']+1.4:excess.append((speed-independent)*3.6)
        if xs[3]>SCENE['start_station_m']+1.4:break
    assert excess and longest_interval(np.array(excess)>c.speed_tolerance_kph,.02)<=c.max_overspeed_duration_s
