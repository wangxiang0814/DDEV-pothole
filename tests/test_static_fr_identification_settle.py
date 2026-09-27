import math

from ddevsim.static_wheel_lift.identification_settle import (
    IdentificationSettleLimits, assess_identification_settle,
)


def _rows(*, drift_n_s=0.):
    rows = []
    for j in range(201):
        t = 44. + j * 0.01
        wobble = math.sin(2 * math.pi * 40 * t)
        row = {"time_s": t, "exp_Roll_E": 0.002 * wobble,
               "exp_Pitch": 0.002 * wobble,
               "exp_AVy": 0.25 * math.cos(2 * math.pi * 40 * t),
               "exp_XCG_TM": 98.1, "exp_YCG_TM": 0.08,
               "exp_Vx": 0.0}
        for wheel in ("L1", "R1", "L2", "R2"):
            row[f"exp_Fz_{wheel}"] = 3000. + 5. * wobble + drift_n_s * (t - 44.)
            row[f"exp_Jnc_{wheel}"] = 20. + 0.01 * wobble
            row[f"exp_AVy_{wheel}"] = 0.0
        rows.append(row)
    return rows


def test_small_fast_oscillation_can_be_usable_for_average_gain():
    result = assess_identification_settle(_rows(), start_s=44.,
                                            limits=IdentificationSettleLimits())
    assert result.status == "SETTLED"


def test_drifting_wheel_load_cannot_be_used_as_static_gain():
    result = assess_identification_settle(_rows(drift_n_s=100.), start_s=44.,
                                            limits=IdentificationSettleLimits())
    assert result.status == "NOT_SETTLED"
    assert result.metrics["max_mean_fz_change_n"] > 20.
