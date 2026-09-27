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
import hashlib
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
from ddevsim.suspension_actuator import signed_probe_cases  # noqa: E402

CORNERS = ("FL", "FR", "RL", "RR")
SUFFIX = {"FL": "L1", "FR": "R1", "RL": "L2", "RR": "R2"}
#: The probe amplitude is resolved at run time from the vehicle's settled static corner
#: load (``--amplitude-n`` overrides it); see the argument help for why the fixed
#: 20 000 N default was removed.
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
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model", choices=("corner_module",), default="corner_module",
        help="I_I corner-module control object",
    )
    parser.add_argument("--hold-s", type=float, default=HOLD_S)
    parser.add_argument(
        "--amplitude-n", type=float, default=None,
        help="spring-seat command amplitude.  Default (None) scales it from the "
             "vehicle's own settled static corner load (see --amplitude-scale), which "
             "keeps the probe inside the suspension's travel on every model.  The old "
             "fixed 20000 N default drove the corner-module front suspension straight "
             "onto its jounce stop and measured a limit cycle instead of a gain.",
    )
    parser.add_argument(
        "--amplitude-scale", type=float, default=1.0,
        help="multiple of the settled static corner load used when --amplitude-n is "
             "not given",
    )
    parser.add_argument("--out-dir", type=Path, default=None)
    args = parser.parse_args()

    # Must be absolute: run_stepwise changes into the simfile's directory, so a
    # relative output path would be resolved against the wrong place mid-run.
    out_dir = (args.out_dir or (ROOT / "runs" / "_actuator_gain")).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    # Both vehicles need a purpose-built STATIONARY case.  Using the vehicle's own
    # rolling scenario (as the first corner-module attempt did) leaves the settle
    # window inside a transient: the measured static loads summed to 13.7 kN against
    # a 9.4 kN vehicle, so the "gain" was measuring dynamics, not the actuator.
    # Holding the vehicle at zero speed on the flat approach keeps every sample in
    # static equilibrium.
    base = ROOT / "models" / "corner_module_ddev"
    if not (base / "simfile.sim").exists():
        raise SystemExit(
            "model %s is missing; run build_corner_module_ddev.py first" % base
        )
    probe_model = out_dir / "probe_model"
    run_all = base / "run_all.par"
    build_single_wheel_pothole_case(
        run_all,
        base / "simfile.sim",
        probe_model,
        scenario=PotholeScenario(target_speed_kph=0.0, stop_s=args.hold_s + 0.5),
    )
    simfile = probe_model / "simfile.sim"

    export_names = list(EXPORT_NAMES) + list(SCENARIO_EXPORTS)
    reports: List[Dict[str, object]] = []

    # Baseline: identical run with all eight channels at zero.
    baseline = run_stepwise(
        simfile, _command(0, 0.0), out_dir / "baseline.csv",
        IMPORT_NAMES, export_names, log_decimation=LOG_DECIMATION, stop_at_s=args.hold_s,
    )
    if baseline["status"] != "COMPLETED":
        raise RuntimeError("baseline run failed: %s" % baseline.get("error_message"))
    base = _settled_tail(out_dir / "baseline.csv")
    print("baseline settled: " + " ".join(
        "Fz_%s=%.1f" % (w, base["exp_Fz_" + w]) for w in SUFFIX.values()))

    # Resolve the probe amplitude against the vehicle's own settled static load, so the
    # same script stays inside the travel of an 8.9 t truck and a 1.36 t corner module.
    if args.amplitude_n is not None:
        amplitude = float(args.amplitude_n)
    else:
        reference = max(base["exp_Fz_" + w] for w in SUFFIX.values())
        if not (reference > 0.0):
            raise RuntimeError(
                "baseline settled with no wheel load; the probe model is not resting on "
                "the ground, so no gain can be measured"
            )
        amplitude = args.amplitude_scale * reference
    print("probe amplitude: %.0f N per channel" % amplitude)

    gain_matrix: Dict[str, Dict[str, float]] = {c: {} for c in CORNERS}
    deflection_matrix: Dict[str, Dict[str, float]] = {c: {} for c in CORNERS}
    jounce_matrix: Dict[str, Dict[str, float]] = {c: {} for c in CORNERS}
    roll_response: Dict[str, float] = {}

    settled_by_corner: Dict[str, Dict[int, Dict[str, float]]] = {
        corner: {} for corner in CORNERS
    }
    for corner, signed_amplitude in signed_probe_cases(amplitude):
        index = CORNERS.index(corner)
        port = 4 + index  # IMP_FS order is FL, FR, RL, RR
        sign = 1 if signed_amplitude > 0 else -1
        csv_path = out_dir / ("force_%s_%s.csv" % (corner, "pos" if sign > 0 else "neg"))
        case = run_stepwise(
            simfile, _command(port, signed_amplitude), csv_path,
            IMPORT_NAMES, export_names, log_decimation=LOG_DECIMATION, stop_at_s=args.hold_s,
        )
        if case["status"] != "COMPLETED":
            raise RuntimeError("%s run failed: %s" % (corner, case.get("error_message")))
        settled = _settled_tail(csv_path)
        settled_by_corner[corner][sign] = settled
        reports.append({
            "corner": corner, "sign": sign, "command_n": signed_amplitude,
            "csv": str(csv_path), "settled": settled,
            "delta_own_load_n": settled["exp_Fz_" + SUFFIX[corner]] - base["exp_Fz_" + SUFFIX[corner]],
            "delta_own_jounce_mm": settled["exp_Jnc_" + SUFFIX[corner]] - base["exp_Jnc_" + SUFFIX[corner]],
            "delta_roll_deg": settled["exp_Roll_E"] - base["exp_Roll_E"],
        })

        print("%+.0f N at %-3s -> dFz_own=%+.0f N dJnc_own=%+.2f mm dRoll=%+.3f deg" % (
            signed_amplitude, corner, reports[-1]["delta_own_load_n"],
            reports[-1]["delta_own_jounce_mm"], reports[-1]["delta_roll_deg"],
        ))

    for corner in CORNERS:
        positive = settled_by_corner[corner][1]
        negative = settled_by_corner[corner][-1]

        for other in CORNERS:
            wheel = SUFFIX[other]
            gain_matrix[corner][other] = (
                positive["exp_Fz_" + wheel] - negative["exp_Fz_" + wheel]
            ) / (2.0 * amplitude)
            deflection_matrix[corner][other] = (
                positive["exp_CmpS_" + wheel] - negative["exp_CmpS_" + wheel]
            ) / (2.0 * amplitude)
            jounce_matrix[corner][other] = (
                positive["exp_Jnc_" + wheel] - negative["exp_Jnc_" + wheel]
            ) / (2.0 * amplitude)
        roll_response[corner] = (positive["exp_Roll_E"] - negative["exp_Roll_E"]) / 2.0

    # A single scalar "how effective is a corner's own actuator" number, used as
    # the controller's feedforward gain.
    diagonal = [gain_matrix[c][c] for c in CORNERS]
    scalar_gain = sum(diagonal) / len(diagonal)

    payload = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "simfile": str(simfile),
        "model_run_all_sha256": hashlib.sha256(run_all.read_bytes()).hexdigest(),
        "force_amplitude_n": amplitude,
        "hold_s": args.hold_s,
        "baseline_settled": base,
        "experiments": reports,
        "gain_matrix_command_to_load": gain_matrix,
        "deflection_matrix_command_to_cmps_mm_per_n": deflection_matrix,
        "jounce_matrix_command_to_jnc_mm_per_n": jounce_matrix,
        "roll_response_deg": roll_response,
        "diagonal_own_corner_gain": diagonal,
        "scalar_feedforward_gain": scalar_gain,
        "note": (
            "gain_matrix_command_to_load[cmd_corner][loaded_corner] is the change in "
            "that corner's tyre normal load per newton of IMP_FS command, estimated "
            "by central positive/negative finite differences on the I_I independent "
            "suspension plant. Body and tyre dynamics still cause cross-coupling."
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
