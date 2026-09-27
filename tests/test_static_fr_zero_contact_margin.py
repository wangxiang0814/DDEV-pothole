import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from run_static_fr_small_unload import _support_margin


def _sample():
    points = {"L1": (-1., 1.), "R1": (1., 1.),
              "L2": (-1., -1.), "R2": (1., -1.)}
    result = {f"Fz_{wheel}": 1000. for wheel in points}
    result["Fz_R1"] = 0.
    for wheel, (x, y) in points.items():
        result[f"Xctc_{wheel}i"] = x
        result[f"Yctc_{wheel}i"] = y
    result["XCG_TM"] = -1. / 3.
    result["YCG_TM"] = -1. / 3.
    return result


def test_fr_zero_contact_is_valid_three_wheel_support():
    assert abs(_support_margin(_sample()) - 1. / 3.) < 1e-12


def test_support_wheel_zero_contact_is_invalid():
    sample = _sample()
    sample["Fz_L2"] = 0.
    assert _support_margin(sample) == float("-inf")
