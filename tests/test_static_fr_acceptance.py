import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from run_static_fr_closed_loop import evaluate_control_rows


def row(time, mode, x=99.1, speed=0., clearance=.04):
    return dict(time_s=time, mode=mode, x_fr_m=x, vx_kph=speed,
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
