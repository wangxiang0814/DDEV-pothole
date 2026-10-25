import numpy as np
import pytest


def test_free_rolling_up_slope_uses_hub_velocity_projected_to_contact_tangent():
    from ddevsim.strategies.shallow_press.contact_kinematics import tangent_transport
    angle=np.deg2rad(30)
    normal=np.array([-np.sin(angle),0,np.cos(angle)])
    centers=np.tile(normal*.25,(4,1));contact=np.zeros((4,3))
    velocity=np.tile([.8,0,.8*np.tan(angle)],(4,1))
    result=tangent_transport(velocity,centers,contact,0.)
    assert result==pytest.approx(np.full(4,.8/np.cos(angle)))
    assert np.max(abs(result-.8))>.12


def test_flat_ground_and_missing_radial_geometry_have_finite_forward_fallback():
    from ddevsim.strategies.shallow_press.contact_kinematics import tangent_transport
    velocity=np.array([[.5,0,.3]]*4)
    assert tangent_transport(velocity,np.zeros((4,3)),np.zeros((4,3)),0.)==pytest.approx([.5]*4)
    with pytest.raises(ValueError):tangent_transport(np.ones((3,3)),np.zeros((4,3)),np.zeros((4,3)),0.)


def test_motion_estimator_uses_causal_metric_positions_with_repeated_stamp_handling():
    from ddevsim.strategies.shallow_press.contact_kinematics import WheelMotion
    from test_actuation import feedback
    d=feedback()
    for i,w in enumerate(('L1','R1','L2','R2')):
        d['X_'+w]=0.;d['Z_'+w]=.25
        d['Xctc_'+w+'i']=0.;d['Yctc_'+w+'i']=d['Y_'+w];d['Zgnd_'+w+'i']=0.
    m=WheelMotion(filter_s=0.)
    m.observe(0.,d)
    for w in ('L1','R1','L2','R2'):
        d['X_'+w]=.0012;d['Xctc_'+w+'i']=.0012
    assert m.observe(.002,d)==pytest.approx([.6]*4)
    assert m.observe(.002,d)==pytest.approx([.6]*4)


def test_motor_hub_frame_does_not_brake_free_rolling_climb():
    from ddevsim.strategies.shallow_press.actuation import CrawlActuation
    from ddevsim.strategies.shallow_press.config import ShallowPressConfig
    from test_actuation import feedback
    a=CrawlActuation(ShallowPressConfig(slip_frame='hub_tangent'))
    d=feedback();d['Vx']=.8;angle=np.deg2rad(30)
    for t in np.arange(0,.3,.0005):
        for w in ('L1','R1','L2','R2'):
            d['X_'+w]=.8*t;d['Z_'+w]=.25*np.cos(angle)+.8*np.tan(angle)*t
            d['Xctc_'+w+'i']=d['X_'+w]+.25*np.sin(angle)
            d['Yctc_'+w+'i']=d['Y_'+w];d['Zgnd_'+w+'i']=.8*np.tan(angle)*t
            d['AVy_'+w]=.8/np.cos(angle)/.263
        u=a.step(float(t),float(t),d,np.zeros(9),None,{'friction':.7},.0005)
    assert np.allclose(u[:4],0.,atol=1e-6)
