import numpy as np
import pytest


def test_extension_headroom_uses_settled_operating_point_and_explicit_reserve():
    from ddevsim.strategies.shallow_press.identification import extension_headroom
    result=extension_headroom([.038028,.022204,.043266,.032767],reserve_m=.02)
    assert result['corner_headroom_m']==pytest.approx([.187028,.171204,.192266,.181767])
    assert result['minimum_with_reserve_m']==pytest.approx(.151204)
    assert result['physical_hardware_rating'] is False
    with pytest.raises(ValueError):extension_headroom([0,0,0,np.nan])


def test_stationary_probe_applies_real_ramps_and_always_returns_to_zero():
    from ddevsim.strategies.shallow_press.identification import RateBoundedStrokeProbe
    p=RateBoundedStrokeProbe();previous=np.zeros(4);seen=[]
    for t in np.arange(0,45.0001,.02):
        u=p.step(float(t),np.full(4,.03),np.full(4,3300.),0.,0.)
        assert np.max(abs(u-previous))<=p.rate_n_s*.02+1e-9
        assert np.max(abs(u))<=4000
        if t<3:assert np.all(u==0)
        seen.append(u);previous=u
    assert np.max(seen)>3000 and np.min(seen)<-399
    assert np.allclose(previous,0)


def test_reach_guard_ends_force_increase_but_never_truncates_native_time():
    from ddevsim.strategies.shallow_press.identification import RateBoundedStrokeProbe
    p=RateBoundedStrokeProbe()
    for t in np.arange(0,23,.02):p.step(float(t),np.full(4,.03),np.full(4,3300.),0.,0.)
    before=p.command.copy()
    u=p.step(23.,np.array([.01,-.13,.02,.03]),np.full(4,3300.),0.,0.)
    assert p.guard_reason=='EXTENSION_RESERVE'
    assert np.all(u<=before+1e-9)
    for t in np.arange(23.02,45.0001,.02):p.step(float(t),np.full(4,.03),np.full(4,3300.),0.,0.)
    assert np.allclose(p.command,0)
    assert p.last_t==pytest.approx(45.)
