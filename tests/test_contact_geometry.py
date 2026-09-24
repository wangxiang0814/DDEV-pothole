"""Physical-road tyre clearance and conservative three-contact geometry."""

import math

import pytest

from ddevsim.contact_geometry import (
    GEOMETRY_EXPORTS,
    augment_geometry_exports,
    audit_native_geometry_rows,
    combined_cg_projection_xy,
    native_support_geometry,
    pothole_tyre_envelope_gap_m,
    parse_body_mass_points,
    parse_echo_total_cg_xy,
    pothole_road_height_m,
    support_risk,
    tyre_envelope_gap_m,
)
from ddevsim.pothole_case import PotholeScenario


def _pit():
    return PotholeScenario(
        start_station_m=101.1, length_m=0.8, width_m=0.9,
        depth_m=0.2, edge_transition_m=0.05, center_y_m=-0.63,
    )


def test_physical_road_height_matches_linear_dz_grid():
    pit = _pit()
    center_y = pit.center_y_m
    assert pothole_road_height_m(pit, 101.1, center_y) == pytest.approx(0.0)
    assert pothole_road_height_m(pit, 101.125, center_y) == pytest.approx(-0.1)
    assert pothole_road_height_m(pit, 101.15, center_y) == pytest.approx(-0.2)
    assert pothole_road_height_m(pit, 101.15, pit.lateral_min_m + 0.025) == pytest.approx(-0.1)
    assert pothole_road_height_m(pit, 101.15, 0.5) == pytest.approx(0.0)


def test_tyre_envelope_detects_exit_lip_collision_despite_pit_floor_clearance():
    pit = _pit()
    road = lambda x, y: pothole_road_height_m(pit, x, y)
    assert tyre_envelope_gap_m((101.50, -0.63, 0.30), 0.263, road) > 0.03
    assert tyre_envelope_gap_m((101.85, -0.63, 0.247), 0.263, road) <= 0.0


def test_exact_pothole_tyre_gap_resolves_lip_between_default_samples():
    pit = _pit()
    wheel = (101.678357, -0.63, 0.20)
    road = lambda x, y: pothole_road_height_m(pit, x, y)
    sparse = tyre_envelope_gap_m(wheel, 0.263, road, 21)
    dense = tyre_envelope_gap_m(wheel, 0.263, road, 10001)
    exact = pothole_tyre_envelope_gap_m(pit, wheel, 0.263)
    assert sparse - exact > 0.005
    assert exact == pytest.approx(dense, abs=0.0001)


def test_exact_pothole_gap_matches_dense_reference_across_both_lips_and_sides():
    pit = _pit()
    road = lambda x, y: pothole_road_height_m(pit, x, y)
    for x in (100.9, 101.02, 101.1, 101.2, 101.55, 101.76, 101.9, 102.0):
        for y in (-1.08, -0.63, -0.20, 0.0):
            wheel = (x, y, 0.30)
            dense = tyre_envelope_gap_m(wheel, 0.263, road, 10001)
            exact = pothole_tyre_envelope_gap_m(pit, wheel, 0.263)
            assert exact <= dense + 1e-10
            assert dense - exact <= 0.0001


def test_flat_road_tyres_need_strict_positive_gap_to_be_called_lifted():
    gap = tyre_envelope_gap_m((0.0, 0.0, 0.30), 0.263, lambda _x, _y: 0.0)
    assert gap == pytest.approx(0.037, abs=1e-12)
    contacts = {"FL": (1.0, 1.0), "RL": (-1.0, 1.0), "RR": (-1.0, -1.0)}
    state = _safe_state()
    state.update(target_free=1.0, target_fz_n=0.0, target_gap_m=-0.001)
    assert not support_risk(state, contacts, (-0.2, 0.2), 0.001)["verified"]


def _safe_state():
    return {
        "Fz_FL": 4500.0, "Fz_RL": 1000.0, "Fz_RR": 4500.0,
        "roll_deg": 1.0, "roll_rate_deg_s": 0.0,
        "roll_projection_tau_s": 0.2, "roll_limit_deg": 7.0,
        "support_floor_n": 300.0, "cop_margin_required_m": 0.01,
    }


def test_cg_uncertainty_larger_than_margin_refuses_verification():
    contacts = {"FL": (1.0, 1.0), "RL": (-1.0, 1.0), "RR": (-1.0, -1.0)}
    cg = (0.0, math.sqrt(2.0) * 0.01)
    result = support_risk(_safe_state(), contacts, cg, cg_error_m=0.015)
    assert result["margin_m"] == pytest.approx(0.01, abs=1e-10)
    assert result["conservative_margin_m"] == pytest.approx(-0.005, abs=1e-10)
    assert not result["verified"]


def test_combined_cg_includes_payload_and_unsprung_mass_at_zero_attitude():
    body_masses = ((600.0, -0.55, 0.0, 0.70),
                   (200.0, -1.10, 0.375, 0.95),
                   (200.0, -1.10, -0.375, 0.95),
                   (200.0, -1.575, 0.0, 0.95))
    wheel_xy = {"FL": (99.1, 0.63), "FR": (99.1, -0.63),
                "RL": (97.175, 0.63), "RR": (97.175, -0.63)}
    xy = combined_cg_projection_xy((99.1, 0.0), (0.0, 0.0, 0.0),
                                    body_masses, wheel_xy, 40.0)
    assert xy == pytest.approx((99.1 - 1239.0 / 1360.0, 0.0), abs=1e-12)


def test_frozen_par_yields_all_three_payload_mass_points():
    from pathlib import Path
    par = Path(__file__).resolve().parents[1] / "models" / "corner_module_ddev" / "run_all.par"
    points = parse_body_mass_points(par.read_text(encoding="utf-8", errors="replace"))
    expected = (
        (600.0, -0.55, 0.0, 0.70),
        (200.0, -1.10, 0.375, 0.95),
        (200.0, -1.10, -0.375, 0.95),
        (200.0, -1.575, 0.0, 0.95),
    )
    assert len(points) == len(expected)
    for actual, wanted in zip(points, expected):
        assert actual == pytest.approx(wanted)


def test_echo_total_cg_reference_is_parsed_in_metres():
    echo = "! H_CG_TL 766.0735394 ; mm\n! LX_CG_TL 910.9485463 ; mm\n! Y_CG_TL 0 ; mm"
    assert parse_echo_total_cg_xy(echo) == pytest.approx((-0.9109485463, 0.0))


def test_native_support_geometry_uses_true_contact_points_and_instant_cg():
    row = {
        "exp_XCG_TM": 99.0, "exp_YCG_TM": 0.02,
        "exp_Xctc_L1i": 100.0, "exp_Yctc_L1i": 0.62,
        "exp_Xctc_L2i": 98.0, "exp_Yctc_L2i": 0.63,
        "exp_Xctc_R2i": 98.2, "exp_Yctc_R2i": -0.65,
        "exp_X_L1": 100.1, "exp_Y_L1": 0.60,
    }
    contacts, cg = native_support_geometry(row, ("FL", "RL", "RR"))
    assert contacts == {"FL": (100.0, 0.62), "RL": (98.0, 0.63), "RR": (98.2, -0.65)}
    assert cg == (99.0, 0.02)


def test_native_support_geometry_rejects_missing_or_nonfinite_native_channels():
    with pytest.raises(ValueError, match="missing native geometry"):
        native_support_geometry({"exp_XCG_TM": 0.0, "exp_YCG_TM": 0.0}, ("FL",))
    row = {
        "exp_XCG_TM": 0.0, "exp_YCG_TM": 0.0,
        "exp_Xctc_L1i": float("nan"), "exp_Yctc_L1i": 0.0,
    }
    with pytest.raises(ValueError, match="nonfinite native geometry"):
        native_support_geometry(row, ("FL",))


def test_native_geometry_audit_compares_all_rows_and_loaded_contact_locations():
    row = {"exp_Xo": 100.0, "exp_Yo": 0.0,
           "exp_Roll_E": 0.0, "exp_Pitch": 0.0, "exp_Yaw": 0.0,
           "exp_XCG_TM": 100.0, "exp_YCG_TM": 0.0}
    for corner in ("L1", "R1", "L2", "R2"):
        row.update({"exp_X_" + corner: 100.0,
                    "exp_Y_" + corner: 0.0,
                    "exp_Xctc_" + corner + "i": 100.02,
                    "exp_Yctc_" + corner + "i": 0.0,
                    "exp_Fz_" + corner: 1000.0})
    result = audit_native_geometry_rows([row], ((1000.0, 0.0, 0.0, 0.0),), 1.0)
    assert result["rows"] == 1
    assert result["max_cg_reconstruction_error_m"] == pytest.approx(0.0)
    assert result["max_loaded_contact_wheel_center_offset_m"] == pytest.approx(0.02)


def test_geometry_exports_augment_only_a_generated_case_interface():
    native = "PARSFILE\nEND\nEXPORT AVy_L1\nEXPORT Sta_Road\nEND\n"
    simfile = "SIMFILE\nPORTS_IMP 8\nPORTS_EXP 2\nEND\n"
    modified, spec = augment_geometry_exports(native, simfile)
    assert modified.startswith("PARSFILE\nEND\nEXPORT AVy_L1")
    assert modified.endswith("EXPORT YCG_TM\nEND\n")
    assert spec == "SIMFILE\nPORTS_IMP 8\nPORTS_EXP 12\nEND\n"
    assert len(GEOMETRY_EXPORTS) == 10
    assert modified.count("EXPORT XCG_TM") == 1
    with pytest.raises(ValueError, match="already has geometry exports"):
        augment_geometry_exports(modified, spec)
