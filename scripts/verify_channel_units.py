"""Empirically verify the unit of every TruckSim export used by the platform.

Rationale
---------
TruckSim 2019 does not document the units of an ``EXPORT`` variable in the
generated files, and the units are NOT all SI.  The platform's own logical
channel names (``wheel_speed_radps``, ``speed_mps``) were therefore wrong for
two channels.  Guessing units silently corrupts every controller and every
training set, so this script *measures* them instead.

Method
------
A short diagnostic case is built from the real single-wheel pothole model.  It
exports deliberately redundant channels, so that each unknown can be compared
against an independently defined quantity:

* ``Rot_*`` is defined by TruckSim as *revolutions* (``.vs`` header), so
  ``d(Rot_*)/dt`` is an unambiguous wheel angular rate -> pins ``AVy_*``.
* ``Xo`` is defined in *metres* (validated: the pothole lip station 101.1 m
  matches the measured wheel-load loss), so ``d(Xo)/dt`` pins ``Vx``.
* ``Z_L*``/``Z_R*`` (wheel-centre height) are defined in *metres*, so
  ``d(Z_*)/dt`` pins ``Vz_Wc_*``.
* ``Fz_*`` is pinned by static equilibrium: the four static loads must sum to
  ``m*g``.
* ``Roll_E``/``Pitch`` are declared in *degrees* in the ``.vs`` header.

Every comparison is made at full solver resolution (no decimation) so that
fast transients are not aliased.

Usage
-----
    $env:PYTHONPATH='src'; python scripts\verify_channel_units.py

Writes ``runs/_unit_probe/`` (CSV, JSON report) and prints a table.
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from ddevsim.cosim import run_stepwise  # noqa: E402
from ddevsim.pothole_case import SCENARIO_EXPORTS  # noqa: E402
from ddevsim.pothole_controller import WheelLiftController  # noqa: E402

# Channels re-exported purely to pin the units of the contract channels.
PROBE_EXPORTS = ("Rot_L1", "Rot_R1", "Rot_L2", "Rot_R2", "Z_L1", "Z_R1", "Z_L2", "Z_R2")

CONTRACT_IMPORTS = (
    "IMP_MYUSM_L1", "IMP_MYUSM_R1", "IMP_MYUSM_L2", "IMP_MYUSM_R2",
    "IMP_FS_L1", "IMP_FS_R1", "IMP_FS_L2", "IMP_FS_R2",
)
CONTRACT_EXPORTS = (
    "AVy_L1", "AVy_R1", "AVy_L2", "AVy_R2",
    "Fz_L1", "Fz_R1", "Fz_L2", "Fz_R2",
    "CmpS_L1", "CmpS_R1", "CmpS_L2", "CmpS_R2",
    "Vz_Wc_L1", "Vz_Wc_R1", "Vz_Wc_L2", "Vz_Wc_R2",
)

WHEELS = ("L1", "R1", "L2", "R2")


def build_probe_case(source_dir: Path, target_dir: Path, stop_s: float) -> Path:
    """Copy the pothole case, extend its export list, and retarget the simfile."""
    target_dir.mkdir(parents=True, exist_ok=True)
    (target_dir / "output").mkdir(parents=True, exist_ok=True)

    run_all = (source_dir / "run_all.par").read_text(encoding="utf-8", errors="replace")
    patterns = [(r"(?m)^TSTOP\s+[-+0-9.eE]+\s*$", "TSTOP %s" % stop_s)]
    for pattern, replacement in patterns:
        run_all, count = re.subn(pattern, replacement, run_all, count=1)
        if count != 1:
            raise ValueError("source run_all.par must declare exactly one TSTOP")

    final_end = run_all.rfind("\nEND")
    if final_end < 0:
        raise ValueError("source run_all.par has no final END")
    extra = "\n".join("EXPORT " + name for name in PROBE_EXPORTS)
    run_all = run_all[:final_end] + "\n! unit-probe redundant exports\n" + extra + run_all[final_end:]
    (target_dir / "run_all.par").write_text(run_all, encoding="utf-8")

    simfile = (source_dir / "simfile.sim").read_text(encoding="ascii")
    total = len(CONTRACT_EXPORTS) + len(SCENARIO_EXPORTS) + len(PROBE_EXPORTS)
    simfile, count = re.subn(r"(?m)^PORTS_EXP\s+\d+\s*$", "PORTS_EXP %d" % total, simfile)
    if count != 1:
        raise ValueError("source simfile must declare exactly one PORTS_EXP")
    simfile = simfile.replace("single_wheel_deep_pothole", "unit_probe")
    simfile = simfile.replace("hd_utility_ddev", "unit_probe")
    (target_dir / "simfile.sim").write_text(simfile, encoding="ascii")
    return target_dir / "simfile.sim"


def _read_csv(path: Path):
    import csv

    with path.open("r", encoding="utf-8", newline="") as stream:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]


def _central_diff(series, dt):
    out = []
    for i in range(len(series)):
        if i == 0:
            out.append((series[1] - series[0]) / dt)
        elif i == len(series) - 1:
            out.append((series[-1] - series[-2]) / dt)
        else:
            out.append((series[i + 1] - series[i - 1]) / (2.0 * dt))
    return out


def _median(values):
    ordered = sorted(values)
    if not ordered:
        return float("nan")
    return ordered[len(ordered) // 2]


def _scale_estimate(measured, reference, floor):
    """Median ratio measured/reference where the reference is well conditioned."""
    ratios = [m / r for m, r in zip(measured, reference) if abs(r) > floor]
    return _median(ratios), len(ratios)


def analyze(rows, log_decimation: int):
    if len(rows) < 5:
        raise ValueError("diagnostic CSV too short to analyse")
    dt = rows[1]["time_s"] - rows[0]["time_s"]
    times = [r["time_s"] for r in rows]
    present = set(rows[0])

    report = {"sample_dt_s": dt, "samples": len(rows), "decimation": log_decimation, "checks": []}

    def add(name, claimed, reference, scale, n, note):
        report["checks"].append(
            {
                "channel": name,
                "independent_reference": reference,
                "estimated_scale": scale,
                "reference_samples": n,
                "declared_unit": claimed,
                "note": note,
            }
        )

    # ---- AVy_* : compare against d(Rot_*)/dt, Rot is in revolutions -------
    for wheel in WHEELS:
        av = "exp_AVy_" + wheel
        rot = "exp_Rot_" + wheel
        if av not in present or rot not in present:
            continue
        omega_rev_per_s = _central_diff([r[rot] for r in rows], dt)
        # AVy is rpm if  dRot/dt [rev/s] * 60 == AVy
        rpm = [w * 60.0 for w in omega_rev_per_s]
        scale, n = _scale_estimate([r[av] for r in rows], rpm, 0.5)
        add(av, "60*d(Rot_%s)/dt  [rpm]" % wheel, "rpm", scale, n,
            "scale ~1.0 confirms AVy is rpm, not rad/s")

    # ---- Vx : compare against d(Xo)/dt, Xo is in metres -------------------
    if "exp_Vx" in present and "exp_Xo" in present:
        v_ms = _central_diff([r["exp_Xo"] for r in rows], dt)
        kph = [v * 3.6 for v in v_ms]
        scale, n = _scale_estimate([r["exp_Vx"] for r in rows], kph, 0.05)
        add("exp_Vx", "3.6*d(Xo)/dt  [km/h]", "km/h", scale, n,
            "scale ~1.0 confirms Vx is km/h, not m/s")

    # ---- Vz_Wc_* : compare against d(Z_*)/dt, Z is in metres -------------
    for wheel in WHEELS:
        vz = "exp_Vz_Wc_" + wheel
        z = "exp_Z_" + wheel
        if vz not in present or z not in present:
            continue
        dz = _central_diff([r[z] for r in rows], dt)
        scale, n = _scale_estimate([r[vz] for r in rows], dz, 0.05)
        add(vz, "d(Z_%s)/dt  [m/s]" % wheel, "m/s", scale, n,
            "scale ~1.0 confirms Vz_Wc is m/s")

    # ---- Fz_* : static total must equal m*g -----------------------------
    fz_cols = ["exp_Fz_" + w for w in WHEELS if "exp_Fz_" + w in present]
    if fz_cols:
        total_static = sum(rows[0][c] for c in fz_cols)
        report["static_total_vertical_load_n"] = total_static
        add("Fz_*", "sum(Fz) at t~0 == m*g", "N", None, len(fz_cols),
            "pins the force unit to newtons (no scaling possible for a sum)")

    # ---- CmpS_* : travel must lie inside the declared suspension travel --
    for wheel in WHEELS:
        cmp_col = "exp_CmpS_" + wheel
        if cmp_col not in present:
            continue
        v = [r[cmp_col] for r in rows]
        report.setdefault("cmps_range_mm", {})[wheel] = [min(v), max(v)]

    # ---- angles -----------------------------------------------------------
    for col in ("exp_Roll_E", "exp_Pitch"):
        if col in present:
            v = [r[col] for r in rows]
            report.setdefault("angle_range", {})[col] = [min(v), max(v)]

    report["times_s"] = [times[0], times[-1]]
    return report


def main() -> int:
    source_dir = ROOT / "models" / "hd_utility_ddev" / "single_wheel_deep_pothole"
    probe_dir = ROOT / "runs" / "_unit_probe"
    stop_s = 9.0
    log_decimation = 1  # full solver resolution: no aliasing of fast transients

    simfile = build_probe_case(source_dir, probe_dir, stop_s)
    exports = list(CONTRACT_EXPORTS) + list(SCENARIO_EXPORTS) + list(PROBE_EXPORTS)
    result = run_stepwise(
        simfile,
        WheelLiftController(),
        probe_dir / "unit_probe.csv",
        CONTRACT_IMPORTS,
        exports,
        log_decimation=log_decimation,
    )
    rows = _read_csv(probe_dir / "unit_probe.csv")
    report = analyze(rows, log_decimation)
    report["solver"] = result

    (probe_dir / "unit_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    print("solver status: %s  final t=%.3f s  rows=%d" % (
        result["status"], result["final_time_s"], result["rows_written"]))
    print("sample dt = %.6g s" % report["sample_dt_s"])
    print()
    print("%-12s %-28s %10s %8s  %s" % ("channel", "reference", "scale", "n", "verdict"))
    print("-" * 96)
    for check in report["checks"]:
        scale = check["estimated_scale"]
        if scale is None:
            verdict = "n/a"
            shown = "-"
        else:
            shown = "%.4f" % scale
            verdict = "MATCHES" if abs(scale - 1.0) < 0.15 else "DIFFERS"
        print("%-12s %-28s %10s %8d  %s" % (
            check["channel"], check["independent_reference"], shown,
            check["reference_samples"], verdict))
    print()
    print("static total load at t~0: %.1f N" % report.get("static_total_vertical_load_n", float("nan")))
    print("CmpS range (mm):", json.dumps(report.get("cmps_range_mm", {})))
    print("angle range:", json.dumps(report.get("angle_range", {})))
    print()
    print("report -> %s" % (probe_dir / "unit_report.json"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
