import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from run_right_side_full_cycle import (  # noqa: E402
    _enable_steering_import, _scale_lateral_ride_movement,
    _set_pit_geometry, _set_vehicle_start_offset, front_run,
    FullRightSideController,
    evaluate_full_cycle,
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


def test_rear_parking_feedback_resists_rotation_and_disables_unloaded_wheel():
    from dataclasses import replace
    from ddevsim.static_wheel_lift.config import ROBUST_REAR_RUN
    controller = RearCycleController(
        scenario={"friction": .7, "start_station_m": 101.1, "length_m": .8},
        rr_gain_per_coupled_force=-.4,
        config=replace(ROBUST_REAR_RUN, parking_damping_nm_per_rpm=100.))
    x = _rear_observation()
    x["Vx"] = 0.
    for wheel, rpm in zip(("L1", "R1", "L2", "R2"), (1., -2., 0., 3.)):
        x[f"AVy_{wheel}"] = rpm
    command = controller(0., x)
    assert command[0] < 0.
    assert command[1] > 0.
    assert command[2] == command[3] == 0.
    assert max(abs(v) for v in command[:4]) <= (
        ROBUST_REAR_RUN.parking_slew_nm_s * ROBUST_REAR_RUN.control_period_s)


def test_adaptive_rear_preload_pauses_reference_without_aborting():
    from dataclasses import replace
    from ddevsim.static_wheel_lift.config import ROBUST_REAR_RUN
    controller = RearCycleController(
        scenario={"friction": .7, "start_station_m": 101.1, "length_m": .8},
        rr_gain_per_coupled_force=-.4, config=replace(ROBUST_REAR_RUN, adaptive_preload=True))
    controller.mode = "RR_PRELOAD"
    controller.preload_start_s = 0.
    controller.start_rr_load_n = 3000.
    x = _rear_observation()
    x["Vx"] = 0.
    x["Fz_R2"] = 3000.
    x["AVz"] = 3.
    controller(0., x)
    controller(.02, x)
    assert controller.preload_reference_s == 0.
    assert controller.mode == "RR_PRELOAD"
    x["AVz"] = 0.
    controller(.04, x)
    assert controller.preload_reference_s > 0.


def test_rear_rl_preload_participates_without_torquing_swing_rr():
    from dataclasses import replace
    from ddevsim.static_wheel_lift.config import ROBUST_REAR_RUN
    controller = RearCycleController(
        scenario={"friction": .7, "start_station_m": 101.1, "length_m": .8},
        rr_gain_per_coupled_force=-.4,
        config=replace(ROBUST_REAR_RUN, preload_rl_force_n=3000.))
    controller.mode = "RR_PRELOAD"
    controller.preload_start_s = -10.
    controller.start_rr_load_n = 3000.
    command = controller(0., _rear_observation())
    assert command[6] == 3000.
    assert command[3] == 0.


def test_parking_to_crawl_seeds_drive_and_lowering_releases_smoothly():
    from dataclasses import replace
    from ddevsim.static_wheel_lift.config import ROBUST_REAR_RUN
    controller = RearCycleController(
        scenario={"friction": .7, "start_station_m": 101.1, "length_m": .8},
        rr_gain_per_coupled_force=-.4,
        config=replace(ROBUST_REAR_RUN, parking_damping_nm_per_rpm=100.))
    controller.parking_torque[:] = (-100., -80., -60., 0.)
    controller._enter("RR_CRAWL", 0.)
    np.testing.assert_allclose(controller.drive.torque_nm, [-60., -100., -80.])
    controller._enter("RR_LOWERING", 0.)
    controller.lower_start_s = 0.
    x = _rear_observation()
    for wheel in ("L1", "R1", "L2"):
        x[f"AVy_{wheel}"] = 1.
    command = controller(0., x)
    np.testing.assert_allclose(command[:3], [-90., -70., -50.])


def test_static_steering_feedback_corrects_preload_drift_without_drive_torque():
    from dataclasses import replace
    from ddevsim.static_wheel_lift.config import ROBUST_REAR_RUN
    controller = RearCycleController(
        scenario={"friction": .7, "start_station_m": 101.1, "length_m": .8},
        rr_gain_per_coupled_force=-.4,
        config=replace(ROBUST_REAR_RUN, static_steering_feedback=True))
    controller.mode = "RR_PRELOAD"
    controller.preload_start_s = 0.
    controller.start_rr_load_n = 3000.
    controller.initial_yo_m = 0.
    controller.initial_yaw_deg = 0.
    x = _rear_observation()
    x["Vx"] = 0.
    x["Fz_R2"] = 3000.
    command = controller(0., x)
    assert controller.steer_deg == -8.
    assert command[:4] == (0., 0., 0., 0.)


def test_full_cycle_preserves_original_path_reference_on_handoff(monkeypatch):
    class Front:
        mode = "COMPLETE"
        abort_reason = None
        initial_yo_m = .001
        initial_yaw_deg = .02

        def __init__(self, **kwargs):
            pass

        def __call__(self, *args):
            return (0.,) * 8

    monkeypatch.setattr(front_run, "LiftCrawlController", Front)
    controller = FullRightSideController(
        scenario={"friction": .7}, rr_gain_per_coupled_force=-.4)
    controller(0., [])
    assert controller.rear.initial_yo_m == .001
    assert controller.rear.initial_yaw_deg == .02
    controller.front.abort_reason = "FR failed"
    controller.rear_started = False
    controller(.02, [])
    assert not controller.rear_started


def test_sequential_preload_adjusts_diagonal_before_other_supports():
    from dataclasses import replace
    from ddevsim.static_wheel_lift.config import ROBUST_REAR_RUN
    controller = RearCycleController(
        scenario={"friction": .7, "start_station_m": 101.1, "length_m": .8},
        rr_gain_per_coupled_force=-.4,
        config=replace(ROBUST_REAR_RUN, sequential_preload=True,
                       preload_rl_force_n=3000., preload_feedback_deadband_n=1e6))
    controller.mode = "RR_PRELOAD"
    controller.preload_start_s = 0.
    controller.start_rr_load_n = 3000.
    first = controller(2., _rear_observation())
    assert first[4] < 0. and first[5] == first[6] == 0.
    second = controller(6., _rear_observation())
    assert second[4] == ROBUST_REAR_RUN.preload_fl_force_n
    assert second[5] > 0. and second[6] > 0.


def test_full_cycle_evaluation_counts_drift_accumulated_before_rr():
    controller = RearCycleController(
        scenario={"friction": .7, "start_station_m": 101.1, "length_m": .8},
        rr_gain_per_coupled_force=-.4)
    controller.initial_yo_m = 0.
    controller.initial_yaw_deg = 0.
    controller.mode = "RR_CRAWL"
    controller(0., _rear_observation())
    result = evaluate_full_cycle({"status": "PASS"}, controller,
                                 controller.scenario, True)
    assert result["rr_max_lateral_m"] == .1
    assert result["rr_local_lateral_max_m"] == 0.
    assert not result["criteria"]["rr_straight"]


def test_front_steering_feedback_uses_original_path_and_rate_limit(monkeypatch):
    class Front:
        mode = "CRAWL"
        abort_reason = None
        initial_yo_m = 0.
        initial_yaw_deg = 0.
        rows = []

        def __init__(self, **kwargs):
            pass

        def __call__(self, *args):
            return (0.,) * 8

    monkeypatch.setattr(front_run, "LiftCrawlController", Front)
    controller = FullRightSideController(
        scenario={"friction": .7}, rr_gain_per_coupled_force=-.4,
        front_steering_feedback=True)
    x = {name: 0. for name in front_run.EXPORTS}
    x["Yo"] = .03
    x["Yaw"] = 1.
    exports = [x[name] for name in front_run.EXPORTS]
    assert controller(0., exports)[-1] == -8.
    assert controller(.0005, exports)[-1] == -8.
    assert controller(.02, exports)[-1] == -16.


def test_front_evaluation_accepts_the_shared_path_reference():
    row = {"mode": "CRAWL", "time_s": 0., "yo_m": .04,
           "yaw_deg": 0., "x_fr_m": 100., "x_rr_m": 98., "vx_kph": 1.,
           "fz_fl_n": 4500., "fz_fr_n": 0., "fz_rl_n": 4500., "fz_rr_n": 4500.}
    result = front_run.evaluate_control_rows(
        [row], crawl=True, scenario={"start_station_m": 101.1, "length_m": .8},
        native_completed=True, abort_reason=None,
        path_reference_yo_m=0., path_reference_yaw_deg=0.)
    assert result["moving_lateral_max_m"] == .04


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
