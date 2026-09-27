import pytest

from ddevsim.static_wheel_lift.mass_study import (
    scale_payload_mass, scale_vehicle_mass, shift_rear_payload,
)


BLOCK = "M_PL 200\nIXX_PL 100\nIYY_PL 100\nIZZ_PL 100\n"


def test_scale_payload_changes_three_mass_inertia_blocks_only():
    original = "VEHICLE_CODE i_i\nM_SU 600\n" + BLOCK * 3 + "TSTEP 0.0005\n"
    changed = scale_payload_mass(original, 100)
    assert changed.count("M_PL 100\nIXX_PL 50\nIYY_PL 50\nIZZ_PL 50\n") == 3
    assert "M_SU 600\n" in changed
    assert changed.endswith("TSTEP 0.0005\n")


def test_scale_payload_rejects_unexpected_model_or_inertia():
    with pytest.raises(ValueError):
        scale_payload_mass("VEHICLE_CODE s_s\n" + BLOCK * 3, 100)
    with pytest.raises(ValueError):
        scale_payload_mass("VEHICLE_CODE i_i\n" + BLOCK * 2, 100)
    with pytest.raises(ValueError):
        scale_payload_mass("VEHICLE_CODE i_i\n" + BLOCK * 2 + BLOCK.replace("IYY_PL 100", "IYY_PL 90"), 100)


def test_uniform_mass_scaling_preserves_geometry_and_nonmass_parameters():
    body = ("M_US 80\n" * 2 + "M_SU 600\nIXX_SU 384.0\nIYY_SU 624.2\n"
            "IZZ_SU 686.9\n" + BLOCK * 3)
    original = "VEHICLE_CODE i_i\nLX_CG_SU 550\n" + body + "K_SPRING 30\n"
    changed = scale_vehicle_mass(original, 0.5)
    assert changed.count("M_US 40\n") == 2
    assert "M_SU 300\n" in changed
    assert changed.count("M_PL 100\n") == 3
    assert "IXX_SU 192\n" in changed
    assert "LX_CG_SU 550\n" in changed
    assert changed.endswith("K_SPRING 30\n")


def test_rear_ballast_shift_preserves_mass_and_changes_only_position():
    rear = ("SET_OFFSET_X -1.57500007481\nSET_OFFSET_Y 0\n"
            "SET_OFFSET_Z 0.950000045123\nH_CG_PL 950\n"
            "LX_CG_PL 1575\nY_CG_PL 0\nM_PL 200\n")
    original = "VEHICLE_CODE i_i\n" + rear + "M_SU 600\n"
    shifted = shift_rear_payload(original, rearward_mm=150, leftward_mm=230)
    assert "LX_CG_PL 1725\nY_CG_PL 230\nM_PL 200\n" in shifted
    assert "SET_OFFSET_X -1.72500007481\nSET_OFFSET_Y 0.23\n" in shifted
    assert shifted.endswith("M_SU 600\n")
    rear_corner = shift_rear_payload(original, rearward_mm=350, leftward_mm=500)
    assert "LX_CG_PL 1925\nY_CG_PL 500\nM_PL 200\n" in rear_corner
    with pytest.raises(ValueError):
        shift_rear_payload(original, rearward_mm=351, leftward_mm=500)
