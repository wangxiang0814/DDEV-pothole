import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from run_static_fr_closed_loop import evaluate_control_rows


def row(time, mode, x=99.1, speed=0., clearance=.04):
    return dict(time_s=time, mode=mode, x_fr_m=x, x_rr_m=x - 1.94,
                vx_kph=speed,
                yo_m=0., yaw_deg=0., fr_top_clearance_m=clearance,
                fz_fl_n=6400., fz_fr_n=0., fz_rl_n=1300., fz_rr_n=5600.,
                zmp_lambda_min=.09, com_lambda_min=.1,
                force_corr_fl_n=0., force_corr_rl_n=0.,
                force_corr_rr_n=0.)


def test_acceptance_requires_hold_crossing_and_four_wheel_return():
    rows = [row(54., "THREE_WHEEL_HOLD"), row(58., "THREE_WHEEL_HOLD"),
            row(59., "CRAWL", x=99.1, speed=.5),
            row(72., "CRAWL", x=101.3, speed=2.2),
            row(77., "CRAWL", x=101.8, speed=2.2),
            row(79., "STOP", x=102.3, speed=.02),
            row(80., "LOWERING", x=102.3, speed=.01),
            row(96., "COMPLETE", x=102.3, speed=0.)]
    rows[-1]["fz_fr_n"] = 3000.
    result = evaluate_control_rows(rows, crawl=True,
                                   scenario={"start_station_m": 101.1,
                                             "length_m": .8},
                                   native_completed=True,
                                   abort_reason=None)
    assert result["status"] == "PASS"
    assert result["hold_duration_s"] >= 5.
    rows[4]["com_lambda_min"] = -.01
    assert evaluate_control_rows(rows, crawl=True,
                                 scenario={"start_station_m": 101.1,
                                           "length_m": .8},
                                 native_completed=True,
                                 abort_reason=None)["status"] == "FAIL"


def test_acceptance_rejects_more_than_five_centimetres_lateral_drift():
    rows = [row(54., "THREE_WHEEL_HOLD"), row(59., "CRAWL", x=101.3, speed=2.2),
            row(60., "CRAWL", x=101.8, speed=2.2),
            row(61., "STOP", x=102.3, speed=0.),
            row(62., "LOWERING", x=102.3, speed=0.),
            row(78., "COMPLETE", x=102.3, speed=0.)]
    rows[0]["time_s"] = 54.
    rows.insert(1, row(58.98, "THREE_WHEEL_HOLD"))
    rows[-1]["fz_fr_n"] = 3000.
    rows[3]["yo_m"] = .06
    result = evaluate_control_rows(rows, crawl=True,
                                   scenario={"start_station_m": 101.1,
                                             "length_m": .8},
                                   native_completed=True, abort_reason=None)
    assert result["criteria"]["straight_path"] is False


def test_acceptance_rejects_stop_with_rear_wheel_in_pit():
    rows = [row(54., "THREE_WHEEL_HOLD"),
            row(58.98, "THREE_WHEEL_HOLD"),
            row(59., "CRAWL", x=101.3, speed=2.2),
            row(60., "CRAWL", x=101.8, speed=2.2),
            row(61., "STOP", x=102.4, speed=0.),
            row(62., "LOWERING", x=102.4, speed=0.),
            row(78., "COMPLETE", x=102.4, speed=0.)]
    rows[-1]["fz_fr_n"] = 3000.
    rows[5]["x_rr_m"] = 101.2
    result = evaluate_control_rows(rows, crawl=True,
                                   scenario={"start_station_m": 101.1,
                                             "length_m": .8},
                                   native_completed=True, abort_reason=None)
    assert result["criteria"]["rear_stopped_before_pit"] is False


def test_acceptance_rejects_sustained_support_correction_saturation():
    rows = [row(54., "THREE_WHEEL_HOLD"),
            row(58.98, "THREE_WHEEL_HOLD")]
    for i in range(35):
        sample = row(59. + i * .02, "CRAWL", x=101.3 + i * .01,
                     speed=2.2)
        sample["support_feedback_saturated"] = True
        rows.append(sample)
    rows += [row(60., "STOP", x=102.3, speed=0.),
             row(61., "LOWERING", x=102.3, speed=0.),
             row(77., "COMPLETE", x=102.3, speed=0.)]
    rows[-1]["fz_fr_n"] = 3000.
    result = evaluate_control_rows(rows, crawl=True,
                                   scenario={"start_station_m": 101.1,
                                             "length_m": .8},
                                   native_completed=True, abort_reason=None)
    assert result["criteria"]["no_sustained_support_saturation"] is False
