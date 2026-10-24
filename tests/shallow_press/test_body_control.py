import numpy as np
import pytest


def test_modal_feedback_has_no_heave_from_pure_pitch_and_opposes_body_motion():
    from ddevsim.strategies.shallow_press.body_control import modal_feedback
    x=np.array([.96,.96,-.965,-.965]);y=np.array([.55,-.71,.55,-.71])
    a=np.stack((np.ones(4),y,-x))
    f=modal_feedback(x,y,0.,0.,0.,.02,0.,0.,8000.,10000.,2000.)
    q=a@f
    assert q[0]==pytest.approx(0,abs=1e-9)
    assert q[1]==pytest.approx(0,abs=1e-9)
    assert q[2]<0
    f=modal_feedback(x,y,.01,.1,.02,.01,.02,.03,8000.,10000.,2000.)
    assert np.all(a@f<0)


def test_modal_feedback_rejects_degenerate_geometry():
    from ddevsim.strategies.shallow_press.body_control import modal_feedback
    with pytest.raises(ValueError):
        modal_feedback(np.zeros(4),np.zeros(4),0,0,0,0,0,0,1,1,1)


def test_angular_damping_can_be_strengthened_without_increasing_heave_damping():
    from ddevsim.strategies.shallow_press.body_control import modal_feedback
    x=np.array([.96,.96,-.965,-.965]);y=np.array([.55,-.71,.55,-.71])
    a=np.stack((np.ones(4),y,-x))
    normal=a@modal_feedback(x,y,0,.1,0,0,.1,0,8000,10000,2000)
    stronger=a@modal_feedback(x,y,0,.1,0,0,.1,0,8000,10000,2000,8000)
    assert stronger[0]==pytest.approx(normal[0])
    assert stronger[1]==pytest.approx(4*normal[1])
