import numpy as np
from test_preview_control import SCENE


def test_downhill_window_builds_braking_before_contact_and_ignores_the_other_track():
    from ddevsim.strategies.shallow_press.preview import descent_resistance_envelope
    p=np.array([[100.95,-.63],[100.95,.63]])
    load=np.full(2,3300.);radius=np.full(2,.263)
    assert np.allclose(descent_resistance_envelope(p,SCENE,load,radius,1.,0.,[1.,0.]),0.)
    early=descent_resistance_envelope(p,SCENE,load,radius,1.,.26,[1.,0.])
    assert early[0]<-100. and early[1]==0.
    assert abs(early[0])<=3300*.263
    assert np.allclose(descent_resistance_envelope(p+np.array([2.,0.]),SCENE,load,radius,1.,.26,[1.,0.]),0.)
