import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from run_right_side_full_cycle import (  # noqa: E402
    _enable_steering_import, _scale_lateral_ride_movement,
    _set_pit_geometry, _set_vehicle_start_offset, front_run,
)
from ddevsim.static_wheel_lift.config import rear_speed_trial_config  # noqa: E402
from ddevsim.static_wheel_lift.rear_cycle import (  # noqa: E402
    RearCycleController, assess_rear_gate,
)


def _rear_observation():
    x = {
        "XCG_TM": 0.2, "YCG_TM": 0.1,
        "X_R2": 100.6, "Z_R2": 0.31,
        "Roll_E": 0., "Pitch": 0., "Vx": 2.,
        "Yaw": 3., "Yo": 0.1, "AVz": 0.,
    }
    for wheel, xy, load in zip(
        ("L1", "R1", "L2", "R2"),
        ((1., .6), (1., -.6), (-1., .6), (-1., -.6)),
        (4500., 4500., 4500., 0.),
    ):
        x[f"Xctc_{wheel}i"], x[f"Yctc_{wheel}i"] = xy
        x[f"Fz_{wheel}"] = load
        x[f"Jnc_{wheel}"] = 0.
    return x


def test_rear_gate_uses_three_other_wheels_and_real_clearance():
    x = _rear_observation()
    gate = assess_rear_gate(x, np.array([4500., 4500., 4500., 0.]))
    assert gate.safe
    assert not gate.ready  # The vehicle is already moving.
    assert gate.min_support_n == 4500.
    assert gate.clearance_m > 0.


def test_rear_stop_travel_relief_is_feedback_driven_and_rate_limited():
    controller = RearCycleController(
        scenario={"friction": .7, "start_station_m": 101.1, "length_m": .8},
        rr_gain_per_coupled_force=-.4)
    controller.mode = "RR_STOP"
    controller.preload_start_s = -20.
    controller.lift_start_s = -10.
    x = _rear_observation()
    x["X_R2"] = 102.4
    x["Jnc_R1"] = -147.
    controller(0., x)
    assert controller.stop_travel_relief_n < 0.
    assert abs(controller.stop_travel_relief_n) <= (
        controller.config.stop_travel_relief_slew_n_s * .02)
    for t in np.arange(.02, .42, .02):
        controller(float(t), x)
    assert abs(controller.stop_travel_relief_n) <= controller.config.stop_travel_relief_limit_n
    assert controller.rows[-1]["stop_travel_relief_n"] < 0.
    previous = controller.stop_travel_relief_n
    x["Jnc_R1"] = -130.
    controller(.42, x)
    assert controller.stop_travel_relief_n > previous


def test_rear_stop_travel_relief_does_not_start_over_the_pit():
    controller = RearCycleController(
        scenario={"friction": .7, "start_station_m": 101.1, "length_m": .8},
        rr_gain_per_coupled_force=-.4)
    controller.mode = "RR_STOP"
    x = _rear_observation()
    x["Jnc_R1"] = -147.
    controller(0., x)
    assert controller.stop_travel_relief_n == 0.


def test_rear_abort_recovers_only_after_clearance_and_stationary_dwell():
    controller = RearCycleController(
        scenario={"friction": .7, "start_station_m": 101.1, "length_m": .8},
        rr_gain_per_coupled_force=-.4)
    controller.mode = "RR_ABORT_STOP"
    controller.abort_reason = "RR support suspension rebound limit"
    x = _rear_observation()
    x["Vx"] = 0.
    controller(0., x)
    controller(2., x)
    assert controller.mode == "RR_ABORT_STOP"
    x["X_R2"] = 102.4
    controller(2.02, x)
    assert controller.mode == "RR_ABORT_STOP"
    controller(3.04, x)
    assert controller.mode == "RR_LOWERING"
    assert controller.abort_reason is not None


def test_rear_lift_uses_posture_assistance_without_relaxing_entry_gate():
    from ddevsim.static_wheel_lift.config import ROBUST_REAR_RUN
    controller = RearCycleController(
        scenario={"friction": .7, "start_station_m": 101.1, "length_m": .8},
        rr_gain_per_coupled_force=-.4, config=ROBUST_REAR_RUN)
    controller.mode = "RR_LIFTING"
    controller.preload_start_s = -20.
    controller.lift_start_s = 0.
    x = _rear_observation()
    x["Vx"] = 0.
    x["Z_R2"] = ROBUST_REAR_RUN.tyre_radius_m + .0015
    controller(4.02, x)
    assert controller.mode == "RR_LIFTING"
    assert controller.posture_start_s == 4.02
    x["Z_R2"] += .003
    controller(4.04, x)
    assert controller.mode == "RR_POSTURE"
    assert controller.posture_start_s == 4.02  # No reset of the force ramp.


def test_rear_steering_corrects_positive_lateral_and_yaw_with_rate_limit():
    controller = RearCycleController(
        scenario={"friction": .7, "start_station_m": 101.1, "length_m": .8},
        rr_gain_per_coupled_force=-.4)
    controller.mode = "RR_CRAWL"
    controller.initial_yaw_deg = 0.
    controller.initial_yo_m = 0.
    x = _rear_observation()
    controller(0., x)
    assert controller.steer_deg == -8.
    controller(.02, x)
    assert controller.steer_deg == -16.


def test_steering_import_only_extends_isolated_model(tmp_path):
    model = tmp_path / "model"
    model.mkdir()
    run = model / "run_all.par"
    run.write_text("IMPORT IMP_FS_R2 Add 0.0! 0\nEXPORT Vx\n", encoding="utf-8")
    sim = model / "simfile.sim"
    sim.write_text("PORTS_IMP 8\nPORTS_EXP 1\n", encoding="utf-8")
    _enable_steering_import(model)
    assert "IMPORT IMP_STEER_SW Replace 0.0! 0" in run.read_text(encoding="utf-8")
    assert "PORTS_IMP 9" in sim.read_text(encoding="utf-8")


def test_lateral_ride_scale_changes_only_copied_tables(tmp_path):
    model = tmp_path / "model"
    model.mkdir()
    run = model / "run_all.par"
    table = "SUSP_LAT_TABLE SPLINE\n-70, 4\n0, 0\n70, 10\nENDTABLE\n"
    run.write_text(table * 4, encoding="utf-8")
    _scale_lateral_ride_movement(model, .5)
    content = run.read_text(encoding="utf-8")
    assert content.count("-70, 2\n") == 4
    assert content.count("70, 5\n") == 4


def test_rear_speed_trial_has_explicit_validated_target():
    config = rear_speed_trial_config(3.3, 3.0)
    assert config.crawl_speed_kph == 3.3
    assert config.min_pit_speed_kph == 3.0


def test_vehicle_start_offset_changes_only_the_copied_initial_station(tmp_path):
    model = tmp_path / "model"
    model.mkdir()
    run = model / "run_all.par"
    run.write_text("SSTART 99.1\nSSTART(1) 86.1\nSSTART 99.1\n", encoding="utf-8")
    scenario = {"start_station_m": 101.1, "approach_distance_m": 2.0}
    _set_vehicle_start_offset(model, scenario, 0.3)
    assert run.read_text(encoding="utf-8").count("SSTART 99.4") == 2
    assert "SSTART(1) 86.1" in run.read_text(encoding="utf-8")
    assert abs(scenario["approach_distance_m"] - 1.7) < 1e-9


def test_boundary_pit_geometry_updates_copied_road_and_scenario(tmp_path):
    model, scenario = front_run._prepare_model(tmp_path / "case", crawl=True)
    _set_pit_geometry(model, scenario, width_m=1.1, depth_m=.25)
    text = (model / "run_all.par").read_text(encoding="utf-8")
    road = text.split("ROAD_DZ_CARPET 2D_LINEAR\n", 1)[1].split("ENDTABLE", 1)[0]
    assert "-1.18" in road
    assert "-0.25" in road
    assert "SPEED_TARGET_CONSTANT 0" in text
    assert scenario["width_m"] == 1.1
    assert scenario["depth_m"] == .25
