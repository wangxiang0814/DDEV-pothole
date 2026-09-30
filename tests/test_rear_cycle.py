import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from run_right_side_full_cycle import (  # noqa: E402
    _enable_steering_import, _scale_lateral_ride_movement,
)
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
