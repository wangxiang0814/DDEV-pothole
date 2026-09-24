"""Bounded four-corner reachability searches must fail conservatively."""

import pytest

from ddevsim.reachability import (
    build_profile,
    classify_reachability,
    isolated_rear_start_station,
)


def test_profile_clips_force_and_slew_and_only_unloads_requested_corner():
    profile = build_profile("FR", (50.0, -50.0, 2.0, 1.0),
                            t_preload_s=0.5, t_lift_s=1.0,
                            force_limit_n=10000.0, slew_n_per_s=2000.0)
    at_start = profile(0.0, ())
    at_preload = profile(0.5, ())
    at_lift = profile(1.5, ())
    assert len(at_start) == 8
    assert at_start == (0.0,) * 8
    assert at_preload[5] == 0.0  # FR remains loaded during posture trim
    assert at_lift[5] == pytest.approx(-2000.0)
    assert max(abs(value) for value in at_lift[4:]) <= 10000.0
    for earlier, later, dt in ((at_start, at_preload, 0.5),
                               (at_preload, at_lift, 1.0)):
        assert max(abs(a - b) / dt for a, b in zip(earlier[4:], later[4:])) <= 2000.0


def _row(margin_cg_y=0.0, target_load=0.0):
    row = {"time_s": "3.0", "exp_X_R1": "101.5", "exp_Yo": "0",
           "exp_Roll_E": "0", "exp_Yaw": "0",
           "exp_XCG_TM": "101.0", "exp_YCG_TM": str(margin_cg_y),
           "exp_Fz_R1": str(target_load), "exp_Z_R1": "0.5",
           "exp_Y_R1": "-0.63"}
    for tyre, x, y in (("L1", 101.8, 0.63),
                       ("L2", 99.9, 0.63), ("R2", 99.9, -0.63)):
        row.update({"exp_Xctc_" + tyre + "i": str(x),
                    "exp_Yctc_" + tyre + "i": str(y),
                    "exp_Fz_" + tyre: "3000",
                    "exp_Jnc_" + tyre: "0"})
    row["exp_Jnc_R1"] = "0"
    return row


def _limits():
    return {"pit_start_m": 101.1, "pit_end_m": 101.9,
            "pit_center_y_m": -0.63, "pit_width_m": 0.9,
            "pit_depth_m": 0.2, "edge_transition_m": 0.05,
            "tyre_radius_m": 0.263, "cg_margin_m": 0.01,
            "cg_error_m": 0.001, "support_floor_n": 300.0,
            "target_fz_max_n": 150.0, "roll_max_deg": 7.0,
            "yaw_max_deg": 2.0, "lateral_max_m": 0.15,
            "jounce_min_m": -0.1, "jounce_max_m": 0.16,
            "interior_start_m": 101.45, "interior_end_m": 101.55}


def test_negative_calibrated_margin_rejects_even_with_positive_support_loads():
    row = _row(margin_cg_y=-0.004)
    result = classify_reachability([row], "FR", _limits())
    assert result["status"] == "feasibility_unresolved"
    assert result["binding_constraint"] == "cg_margin"
    assert result["evidence"]["min_support_load_n"] == 3000.0


def test_exhausted_search_cannot_be_reported_as_physical_infeasibility():
    result = classify_reachability([], "RR", _limits())
    assert result["status"] == "feasibility_unresolved"
    assert result["binding_constraint"] == "missing_pit_window"


def test_rebound_stop_contact_is_not_accepted_as_clean_active_lift():
    row = _row(margin_cg_y=0.5)
    row["exp_Jnc_R1"] = "-100.3"
    result = classify_reachability([row], "FR", _limits())
    assert result["status"] == "feasibility_unresolved"
    assert result["binding_constraint"] == "suspension_travel"


def test_well_inside_triangle_clear_lift_is_only_an_admissible_candidate():
    result = classify_reachability([_row(margin_cg_y=0.5)], "FR", _limits())
    assert result["status"] == "admissible_candidate"
    assert result["binding_constraint"] is None


def test_isolated_rear_trial_starts_with_front_tyre_past_exit_envelope():
    station = isolated_rear_start_station(
        pit_exit_m=101.9, tyre_radius_m=0.263,
        front_center_offset_m=0.00788162, clearance_m=0.05,
    )
    assert station == pytest.approx(102.20511838)
    assert station + 0.00788162 - 0.263 - 101.9 >= 0.05 - 1e-10
