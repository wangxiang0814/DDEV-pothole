import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from run_static_fr_closed_loop import (  # noqa: E402
    CFG, EXPORTS, LiftCrawlController, _prepare_model,
)


def test_prepared_crawl_scenario_uses_effective_controller_speed(tmp_path):
    _, scenario = _prepare_model(tmp_path / "crawl", crawl=True)
    assert scenario["target_speed_kph"] == pytest.approx(CFG.crawl_speed_kph)
    assert scenario["controller_period_s"] == pytest.approx(CFG.control_period_s)


def test_prepared_crawl_changes_physical_road_friction_in_copy(tmp_path):
    model, scenario = _prepare_model(tmp_path / "low_mu", crawl=True,
                                     road_friction=0.6)
    text = (model / "run_all.par").read_text(encoding="utf-8")
    assert scenario["friction"] == pytest.approx(0.6)
    assert text.count("MU_ROAD_CONSTANT 0.6") == 1


def test_controller_FR_feedback_has_authority_at_104N_gate():
    controller = LiftCrawlController(crawl=False, scenario={"friction": 0.7})
    controller.fr_feedback.correction_n = -300.
    command = controller.fr_feedback.update(0., measured_n=104.,
                                             reference_n=78.)
    assert command < -300.


def test_preload_stall_smoothly_returns_instead_of_waiting_forever():
    controller = LiftCrawlController(crawl=False, scenario={"friction": 0.7})
    controller.mode = "PRELOAD_SHIFT"
    controller.preload_clock.progress_s = controller.preload_clock.end_s
    controller.preload_clock.last_time_s = 30.
    values = dict.fromkeys(EXPORTS, 0.)
    values.update({
        "Fz_L1": 6400., "Fz_R1": 3000., "Fz_L2": 1300., "Fz_R2": 5600.,
        "Xctc_L1i": 1., "Yctc_L1i": .625,
        "Xctc_R1i": 1., "Yctc_R1i": -.625,
        "Xctc_L2i": -1., "Yctc_L2i": .625,
        "Xctc_R2i": -1., "Yctc_R2i": -.625,
        "XCG_TM": -.3, "YCG_TM": .4, "Z_R1": .263,
    })
    observation = tuple(values[name] for name in EXPORTS)
    controller(31., observation)
    controller(34.02, observation)
    assert controller.mode == "LOWERING"
    assert controller.abort_reason == "preload ready gate not reached"


def test_preload_tracking_stall_has_total_deadline():
    controller = LiftCrawlController(crawl=False, scenario={"friction": 0.7})
    controller.mode = "PRELOAD_SHIFT"
    controller.mode_start_s = 7.
    controller.preload_clock.progress_s = 30.
    controller.preload_clock.last_time_s = 68.
    values = dict.fromkeys(EXPORTS, 0.)
    values.update({
        "Fz_L1": 6400., "Fz_R1": 3000., "Fz_L2": 1300., "Fz_R2": 5600.,
        "Xctc_L1i": 1., "Yctc_L1i": .625,
        "Xctc_R1i": 1., "Yctc_R1i": -.625,
        "Xctc_L2i": -1., "Yctc_L2i": .625,
        "Xctc_R2i": -1., "Yctc_R2i": -.625,
        "XCG_TM": -.3, "YCG_TM": .4, "Z_R1": .263,
    })
    controller(70., tuple(values[name] for name in EXPORTS))
    assert controller.mode == "LOWERING"
    assert controller.abort_reason == "preload tracking deadline exceeded"


def test_near_zero_FR_uses_swing_support_gain_before_liftoff():
    controller = LiftCrawlController(crawl=False, scenario={"friction": 0.7})
    controller.mode = "PRELOAD_SHIFT"
    values = dict.fromkeys(EXPORTS, 0.)
    values.update({
        "Fz_L1": 6400., "Fz_R1": 50., "Fz_L2": 1300., "Fz_R2": 5600.,
        "Xctc_L1i": 1., "Yctc_L1i": .625,
        "Xctc_R1i": 1., "Yctc_R1i": -.625,
        "Xctc_L2i": -1., "Yctc_L2i": .625,
        "Xctc_R2i": -1., "Yctc_R2i": -.625,
        "XCG_TM": -.3, "YCG_TM": .4, "Z_R1": .263,
    })
    controller(0., tuple(values[name] for name in EXPORTS))
    assert (controller.support.gain == controller.swing_gain).all()


def test_support_feedback_can_exceed_old_300N_bound_when_RL_weak():
    controller = LiftCrawlController(crawl=True, scenario={"friction": 0.7})
    controller.support.gain = controller.swing_gain
    controller.support.correction[:] = [300., -300., 300.]
    controller.support.update(0., [6373., 719., 6174.],
                              [6399., 1007., 5853.],
                              travel_mm=[-101., 42., 19.])
    assert controller.support.correction[0] > 300.


def test_control_log_records_actual_force_command_and_geometry():
    controller = LiftCrawlController(crawl=False, scenario={"friction": 0.7})
    values = dict.fromkeys(EXPORTS, 0.)
    values.update({
        "Fz_L1": 6400., "Fz_R1": 3000., "Fz_L2": 1300., "Fz_R2": 5600.,
        "Xctc_L1i": 1., "Yctc_L1i": 0.625,
        "Xctc_R1i": 1., "Yctc_R1i": -0.625,
        "Xctc_L2i": -1., "Yctc_L2i": 0.625,
        "Xctc_R2i": -1., "Yctc_R2i": -0.625,
        "XCG_TM": -0.3, "YCG_TM": 0.4, "Z_R1": 0.263,
    })
    command = controller(0., tuple(values[name] for name in EXPORTS))
    row = controller.rows[-1]
    assert [row[f"fact_{wheel}_n"] for wheel in ("fl", "fr", "rl", "rr")] == pytest.approx(command[4:8])
    assert row["com_x_m"] == pytest.approx(-0.3)
    assert row["com_y_m"] == pytest.approx(0.4)
    # FR still carries load at INIT, so the three-wheel margin may be signed negative.
    assert row["zmp_edge_distance_m"] < 0.
    assert row["fz_fr_filtered_n"] == pytest.approx(3000.)
    assert row["preload_ref_time_s"] == pytest.approx(0.)
    assert row["support_feedback_saturated"] is False
