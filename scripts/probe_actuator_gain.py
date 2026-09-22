"""Measure the static actuator effectiveness matrix of the TruckSim model.

Why this is necessary
---------------------
``IMP_FS_*`` injects a force at the *spring seat*, not at the tyre contact patch.
The wheel-normal-load change per newton of command is therefore well below 1, and
on this vehicle it is roughly 0.3.  The deep-pothole strategy is a force
feedforward, so using a wrong gain (the previous default was 1.0) makes the
feedforward 3x too small and drives the sliding-mode trim permanently into
saturation, which is exactly what the first expert run showed.

The gain is also cross-coupled: this vehicle has rigid axles, so a force at one
corner loads the opposite corner of the same axle and unloads the body elsewhere.
A single scalar gain is not enough, so this probe measures the full 4x4
command-to-load matrix.

Method
------
For each of the eight channels, hold a constant command for ``hold_s`` seconds on
straight, flat ground with a matched baseline run, then compare the settled tail.
The result is written to ``runs/_actuator_gain/gain_matrix.json``.

Usage
-----
    $env:PYTHONPATH='src'; python scripts\\probe_actuator_gain.py
"""

from __future__ import annotations

import csv
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Sequence

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from ddevsim.cosim import run_stepwise  # noqa: E402
from ddevsim.interface_validation import EXPORT_NAMES, IMPORT_NAMES  # noqa: E402
from ddevsim.pothole_case import (  # noqa: E402
    SCENARIO_EXPORTS,
    PotholeScenario,
    build_single_wheel_pothole_case,
)

CORNERS = ("FL", "FR", "RL", "RR")
SUFFIX = {"FL": "L1", "FR": "R1", "RL": "L2", "RR": "R2"}
FORCE_AMPLITUDE_N = 20000.0
HOLD_S = 5.0
LOG_DECIMATION = 100  # 0.5 ms step -> 50 ms CSV


def _command(port_index: int, amplitude: float, start_s: float = 0.5):
    def command(time_s, _exports):
        values = [0.0] * 8
        if time_s >= start_s:
            values[port_index] = amplitude
        return tuple(values)

    return command


def _settled_tail(csv_path: Path, fraction: float = 0.2) -> Dict[str, float]:
    with Path(csv_path).open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError("no samples in %s" % csv_path)
    tail = rows[-max(1, int(len(rows) * fraction)):]
    keys = [key for key in rows[0] if key != "time_s"]
    return {
        key: sum(float(row[key]) for row in tail) / len(tail) for key in keys
    }


def main() -> int:
    out_dir = ROOT / "runs" / "_actuator_gain"
    out_dir.mkdir(parents=True, exist_ok=True)

    # The probe must measure the actuator, not road geometry: if the vehicle were
    # rolling it would reach the pothole during the settle window (as it did on the
    # first attempt, which put the "baseline" inside the hole and unbalanced the
    # rear axle).  Holding the vehicle stationary at the default pothole station
    # keeps every sample on flat ground and gives a true static gain.
    probe_model = out_dir / "probe_model"
    build_single_wheel_pothole_case(
        ROOT / "models" / "hd_utility_ddev" / "run_all.par",
        ROOT / "models" / "hd_utility_ddev" / "simfile.sim",
        probe_model,
        scenario=PotholeScenario(target_speed_kph=0.0, stop_s=HOLD_S + 0.5),
    )
    simfile = probe_model / "simfile.sim"

    export_names = list(EXPORT_NAMES) + list(SCENARIO_EXPORTS)
    reports: List[Dict[str, object]] = []

    # Baseline: identical run with all eight channels at zero.
    baseline = run_stepwise(
        simfile, _command(0, 0.0), out_dir / "baseline.csv",
        IMPORT_NAMES, export_names, log_decimation=LOG_DECIMATION, stop_at_s=HOLD_S,
    )
    if baseline["status"] != "COMPLETED":
        raise RuntimeError("baseline run failed: %s" % baseline.get("error_message"))
    base = _settled_tail(out_dir / "baseline.csv")
    print("baseline settled: " + " ".join(
        "Fz_%s=%.1f" % (w, base["exp_Fz_" + w]) for w in SUFFIX.values()))

    gain_matrix: Dict[str, Dict[str, float]] = {c: {} for c in CORNERS}
    deflection_matrix: Dict[str, Dict[str, float]] = {c: {} for c in CORNERS}
    roll_response: Dict[str, float] = {}

    for index, corner in enumerate(CORNERS):
        port = 4 + index  # IMP_FS order is FL, FR, RL, RR
        case = run_stepwise(
            simfile, _command(port, FORCE_AMPLITUDE_N),
            out_dir / ("force_%s.csv" % corner),
            IMPORT_NAMES, export_names, log_decimation=LOG_DECIMATION, stop_at_s=HOLD_S,
        )
        if case["status"] != "COMPLETED":
            raise RuntimeError("%s run failed: %s" % (corner, case.get("error_message")))
        settled = _settled_tail(out_dir / ("force_%s.csv" % corner))

        for other in CORNERS:
            wheel = SUFFIX[other]
            delta_load = settled["exp_Fz_" + wheel] - base["exp_Fz_" + wheel]
            delta_defl = settled["exp_CmpS_" + wheel] - base["exp_CmpS_" + wheel]
            gain_matrix[corner][other] = delta_load / FORCE_AMPLITUDE_N
            deflection_matrix[corner][other] = delta_defl / FORCE_AMPLITUDE_N
        roll_response[corner] = settled["exp_Roll_E"] - base["exp_Roll_E"]

        print("+%.0f N at %-3s -> " % (FORCE_AMPLITUDE_N, corner) + " ".join(
            "dFz_%s=%+8.0f" % (o, gain_matrix[corner][o] * FORCE_AMPLITUDE_N)
            for o in CORNERS) + "  dRoll=%+.3f deg" % roll_response[corner])

        reports.append({"corner": corner, "settled": settled})

    # A single scalar "how effective is a corner's own actuator" number, used as
    # the controller's feedforward gain.
    diagonal = [gain_matrix[c][c] for c in CORNERS]
    scalar_gain = sum(diagonal) / len(diagonal)

    payload = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "simfile": str(simfile),
        "force_amplitude_n": FORCE_AMPLITUDE_N,
        "hold_s": HOLD_S,
        "baseline_settled": base,
        "gain_matrix_command_to_load": gain_matrix,
        "deflection_matrix_command_to_cmps_mm_per_n": deflection_matrix,
        "roll_response_deg": roll_response,
        "diagonal_own_corner_gain": diagonal,
        "scalar_feedforward_gain": scalar_gain,
        "note": (
            "gain_matrix_command_to_load[cmd_corner][loaded_corner] is the change in "
            "that corner's tyre normal load per newton of IMP_FS command. Rigid axles "
            "make this matrix strongly off-diagonal."
        ),
    }
    (out_dir / "gain_matrix.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    print()
    print("own-corner gain (dFz_own / F_cmd): " + " ".join(
        "%s=%.3f" % (c, gain_matrix[c][c]) for c in CORNERS))
    print("scalar feedforward gain = %.4f" % scalar_gain)
    print("required command to unload one wheel completely (Fz/|gain|):")
    for corner in CORNERS:
        own = abs(gain_matrix[corner][corner])
        print("   %s: %.1f kN command for a %.1f kN static load" % (
            corner, base["exp_Fz_" + SUFFIX[corner]] / own / 1000.0,
            base["exp_Fz_" + SUFFIX[corner]] / 1000.0))
    print()
    print("report -> %s" % (out_dir / "gain_matrix.json"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
