import numpy as np


def test_rate_controller_filters_nominal_torque_and_uses_actual_applied_state():
    from ddevsim.strategies.shallow_press.config import ShallowPressConfig
    from ddevsim.strategies.shallow_press.actuation import CrawlActuation
    from test_actuation import feedback
    c=ShallowPressConfig(motor_control_mode='rate',motor_torque_bandwidth=20.,motor_slip_rate_gain=1000.)
    a=CrawlActuation(c);d=feedback()
    for t in np.arange(0,.30,.0005):
        u=a.step(float(t),float(t),d,np.r_[[100]*4,[0]*5],None,{'friction':.7},.0005)
    # Restore ramp has ended; the collocated rate controller approaches demand.
    assert np.all((u[:4]>90)&(u[:4]<100))
    # Correct an overspeed relative to the actually applied torque, without a
    # differentiated proxy producing +/-500Nm target flips.
    before=a.command[:4].copy()
    for w in ('L1','R1','L2','R2'):d['AVy_'+w]=.8/.263
    u=a.step(.30,.30,d,np.r_[[100]*4,[0]*5],None,{'friction':.7},.0005)
    assert np.all(u[:4]<before)
    assert np.max(abs(a.motor_target-before))<=2.+1e-9


def test_rate_mode_never_changes_nominal_rate_or_amplitude_envelope():
    from ddevsim.strategies.shallow_press.config import ShallowPressConfig
    from ddevsim.strategies.shallow_press.actuation import CrawlActuation
    from test_actuation import feedback
    a=CrawlActuation(ShallowPressConfig(motor_control_mode='rate'));d=feedback();previous=np.zeros(9)
    for t in np.arange(0,.4,.0005):
        target=np.r_[[500 if t<.2 else -500]*4,[0]*5]
        u=a.step(float(t),float(t),d,target,None,{'friction':.7},.0005)
        assert np.max(abs(u[:4]-previous[:4]))<=.5+1e-9
        assert np.max(abs(u[:4]))<=500
        previous=u
