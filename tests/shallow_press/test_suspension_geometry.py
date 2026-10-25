import numpy as np
import pytest
from ddevsim.strategies.shallow_press.suspension_geometry import seat_kinematics


def test_full_map_segments_endpoint_extrapolation_and_virtual_work():
    g,mu=seat_kinematics(np.array([-.02,.025,-.02,.025]))
    assert g==pytest.approx([-.0130288,.01758897,-.01594,.021519])
    assert mu==pytest.approx([.65144,.781746,.797,.9564])
    displacement=np.full(4,1e-6)
    after,_=seat_kinematics(np.array([-.02,.025,-.02,.025])+displacement)
    assert np.full(4,1000.)*(after-g)==pytest.approx(1000*mu*displacement)
