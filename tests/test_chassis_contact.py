import numpy as np
import pytest

from ddevsim.chassis_contact import transform_points, road_surface, contact_forces, instrument_reference_points, plate_clearance


def scene():
    return dict(start_station_m=10., length_m=.8, width_m=1.8,
                center_y_m=0., depth_m=.2, edge_transition_m=.05)


def test_body_rotation_translates_and_pitches_about_exported_origin():
    points=np.array([[1.,0.,0.],[0.,1.,0.]])
    result=transform_points(points,[10.,20.,30.],[0.,90.,0.])
    assert result == pytest.approx(np.array([[10.,20.,29.],[10.,21.,30.]]))


def test_plate_interior_detects_road_lip_when_corners_all_inside_pit():
    # Broad patch spans the lateral pit rim. Minimum is on physical flat road.
    corners=np.array([[10.1,-1.,.05],[10.7,-1.,.05],[10.7,1.,.05],[10.1,1.,.05]])
    gap,point=plate_clearance(corners[None,:,:],scene())
    assert gap[0] == pytest.approx(.05)
    assert point.shape==(1,3)


def test_exact_plate_minimum_matches_dense_surface_and_handles_tilted_yaw():
    from ddevsim.chassis_contact import sample_plate
    local=np.array([[-.4,-.5,.1],[.4,-.5,.1],[.4,.5,.1],[-.4,.5,.1]])
    dense=sample_plate([-.4,.4,-.5,.5],.1,.003)
    for attitude in [[10.,20.,35.],[-20.,-15.,-40.],[0.,0.,0.]]:
        corners=transform_points(local,[10.4,.2,0.],attitude)
        gap,point=plate_clearance(corners[None,:,:],scene())
        p=transform_points(dense,[10.4,.2,0.],attitude)
        sampled=np.min(p[:,2]-road_surface(p,scene())[0])
        assert gap[0]<=sampled+1e-10
        assert sampled-gap[0]<.006
        assert gap[0] == pytest.approx(point[0,2]-road_surface(point,scene())[0][0])


def test_physical_surface_returns_pit_lip_gradient_and_not_visual_offset():
    z,n=road_surface(np.array([[10.025,0.,0.],[10.4,0.,0.],[11.,0.,0.]]),scene())
    assert z == pytest.approx([-.1,-.2,0.])
    assert n[0] == pytest.approx(np.array([4.,0.,1.])/np.sqrt(17.))


def test_unilateral_contact_has_no_air_force_or_tension_and_friction_dissipates():
    p=np.array([[9.,0.,.1],[9.,0.,-.01]])
    v=np.array([[2.,0.,-1.],[2.,0.,0.]])
    f,gap=contact_forces(p,v,scene(),100000.,1000.,.3,.05)
    assert f[0] == pytest.approx([0.,0.,0.])
    assert f[1,2] == pytest.approx(1000.)
    assert np.dot(f[1,:2],v[1,:2])<0
    assert np.linalg.norm(f[1,:2])<=.3*f[1,2]
    f,_=contact_forces(p,np.array([[0.,0.,0.],[0.,0.,100.]]),scene(),100000.,1000.,.3,.05)
    assert f[1] == pytest.approx([0.,0.,0.])


def test_reference_point_forces_preserve_old_ports_and_use_inertial_axes():
    text='PARSFILE\nIMPORT IMP_FS_L1 Add 0\nEXPORT Xo\nEND\n'
    spec='SIMFILE\nPORTS_IMP 1\nPORTS_EXP 1\nEND\n'
    modified,updated,imports,exports=instrument_reference_points(text,spec,np.array([[0.,0.,.2]]))
    assert 'OPT_RP_FORCE_N(1) 1' in modified
    assert 'Z_RP(1) 200' in modified
    assert imports == ['IMP_FS_L1','IMP_FX_RP_1','IMP_FY_RP_1','IMP_FZ_RP_1']
    assert exports == ['Xo','X_RP1','Y_RP1','Z_RP1']
    assert 'PORTS_IMP 4' in updated and 'PORTS_EXP 4' in updated
    with pytest.raises(ValueError,match='reference points'):
        instrument_reference_points(modified,updated,np.array([[0.,0.,.2]]))
