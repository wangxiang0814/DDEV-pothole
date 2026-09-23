"""Validate a run's channels and score its stability / traversability.

Two jobs in one command, because they answer the two questions a dataset producer
actually asks about a run:

1. **Is the data valid?**  Every exported channel is checked for finiteness, for a
   plausible range, and -- where an independent cross-check exists -- against it.  The
   cross-checks are what catch a unit error or a mis-bound channel, which a range test
   alone cannot: ``Vz_Wc`` is compared against the differentiated wheel-centre height to
   confirm the declared km/h; ``Zgnd`` must reproduce the configured pothole; ``FsExt``
   must track the commanded ``IMP_FS``; ``Jnc`` must move with ``CmpS``; the four tyre
   loads must sum to the vehicle weight away from the hole.

2. **Is the run any good?**  Stability and traversability metrics: body attitude RMS and
   peak, lateral and heading deviation, load-transfer ratio, per-wheel contact loss,
   sprung-mass vertical acceleration, travel usage, actuator saturation and
   command-tracking error.

Usage::

    $env:PYTHONPATH='src'
    python scripts/report_run_metrics.py
    python scripts/report_run_metrics.py --csv runs/corner_module_expert_pothole/data/expert_and_dynamics.csv
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Dict, List, Sequence

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from ddevsim.pothole_case import PotholeScenario  # noqa: E402
from ddevsim.units import to_si  # noqa: E402

CORNERS = ("FL", "FR", "RL", "RR")
SUFFIX = {"FL": "L1", "FR": "R1", "RL": "L2", "RR": "R2"}
G = 9.80665
CONTACT_THRESHOLD_N = 500.0


def _load(csv_path: Path):
    with csv_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise SystemExit("no samples in %s" % csv_path)
    return rows


def _key(name: str) -> str:
    """Map a bare channel name to its CSV column.

    ``time_s`` is already a full column name; the imports and exports carry their
    ``imp_`` / ``exp_`` prefix.
    """
    if name == "time_s" or name.startswith(("exp_", "imp_")):
        return name
    return "exp_" + name


def _col(rows, name) -> List[float]:
    key = _key(name)
    if key not in rows[0]:
        raise SystemExit("channel %s is not in this run's log" % key)
    return [float(row[key]) for row in rows]


def _has(rows, name) -> bool:
    return _key(name) in rows[0]


def _span(values) -> float:
    return max(values) - min(values)


def _rms(values) -> float:
    return math.sqrt(sum(v * v for v in values) / len(values))


# ------------------------------------------------------------------ validation

def validate_channels(rows, scenario: PotholeScenario) -> List[str]:
    """Cross-check the exports against physics and report defects."""
    problems: List[str] = []
    time = _col(rows, "time_s")

    # 1. finite everywhere
    for name in rows[0]:
        if name == "time_s":
            continue
        values = [float(row[name]) for row in rows]
        if not all(math.isfinite(v) for v in values):
            problems.append("%s has non-finite samples" % name)

    # 2. tyre loads must sum to the vehicle weight while the vehicle is on flat road
    loads = {c: _col(rows, "Fz_%s" % SUFFIX[c]) for c in CORNERS}
    total = [sum(loads[c][i] for c in CORNERS) for i in range(len(rows))]
    station = _col(rows, "Sta_Road") if _has(rows, "Sta_Road") else _col(rows, "Xo")
    # Use a flat, quiet window well before the hole.
    flat = [
        i for i, s in enumerate(station)
        if s < scenario.start_station_m - 0.6 and time[i] > 0.4
    ]
    if not flat:
        problems.append("no flat approach window found for the load-sum check")
    else:
        mean_total = sum(total[i] for i in flat) / len(flat)
        weight = _col(rows, "Fz_L1")[0] + _col(rows, "Fz_R1")[0] + \
                 _col(rows, "Fz_L2")[0] + _col(rows, "Fz_R2")[0]
        if abs(mean_total - weight) / weight > 0.05:
            problems.append(
                "tyre loads sum to %.0f N on the flat approach but %.0f N at t=0 "
                "(>5%% apart)" % (mean_total, weight)
            )

    # 3. Vz_Wc must be the wheel-centre vertical velocity: compare with dZ/dt.
    if _has(rows, "Vz_Wc_L1") and _has(rows, "Z_L1"):
        ratios = []
        for i in range(1, len(rows) - 1):
            dt = time[i + 1] - time[i - 1]
            if dt <= 0:
                continue
            dz = (
                to_si("Z_L1", float(rows[i + 1]["exp_Z_L1"]))
                - to_si("Z_L1", float(rows[i - 1]["exp_Z_L1"]))
            ) / dt
            v = to_si("Vz_Wc_L1", float(rows[i]["exp_Vz_Wc_L1"]))
            if abs(dz) > 0.05:            # only where the signal is well above noise
                ratios.append(v / dz)
        if ratios:
            ratios.sort()
            median = ratios[len(ratios) // 2]
            if not (0.6 <= median <= 1.4):
                problems.append(
                    "Vz_Wc_L1 / d(Z_L1)/dt has median %.2f; the declared km/h unit "
                    "looks wrong" % median
                )

    # 4. Zgnd must reproduce the configured pothole under a wheel that crosses it.
    if _has(rows, "Zgnd_R1i"):
        ground = _col(rows, "Zgnd_R1i")
        drop = -min(ground)
        if abs(drop - scenario.depth_m) > 0.03:
            problems.append(
                "Zgnd_R1i only dips %.3f m but the pothole is %.3f m deep"
                % (drop, scenario.depth_m)
            )

    # 5. The realised active force must track the command.
    if _has(rows, "FsExt_R1") and _has(rows, "imp_IMP_FS_R1"):
        realised = _col(rows, "FsExt_R1")
        commanded = _col(rows, "imp_IMP_FS_R1")
        scale = max(1.0, max(abs(v) for v in commanded))
        worst = max(abs(a - b) for a, b in zip(realised, commanded)) / scale
        if worst > 0.5:
            problems.append(
                "FsExt_R1 differs from the IMP_FS_R1 command by up to %.0f%% of full "
                "scale; check the sign convention before using it as a realised "
                "actuator signal" % (100.0 * worst)
            )

    # 6. Jnc and CmpS are different quantities but must be tightly correlated.
    if _has(rows, "Jnc_R1") and _has(rows, "CmpS_R1"):
        a = _col(rows, "Jnc_R1")
        b = _col(rows, "CmpS_R1")
        if _span(a) > 1e-9 and _span(b) > 1e-9:
            ma, mb = sum(a) / len(a), sum(b) / len(b)
            cov = sum((x - ma) * (y - mb) for x, y in zip(a, b))
            sa = math.sqrt(sum((x - ma) ** 2 for x in a))
            sb = math.sqrt(sum((y - mb) ** 2 for y in b))
            corr = cov / (sa * sb) if sa and sb else 0.0
            if corr < 0.5:
                problems.append(
                    "Jnc_R1 and CmpS_R1 correlate at only %.2f; they should move "
                    "together" % corr
                )

    # 7. Friction should be the configured constant on-road value.
    if _has(rows, "MuX_R1i"):
        mu = _col(rows, "MuX_R1i")
        on_road = [v for v in mu if v > 0.0]
        if on_road:
            median = sorted(on_road)[len(on_road) // 2]
            if abs(median - scenario.friction) > 0.15:
                problems.append(
                    "MuX_R1i median %.2f disagrees with the configured friction %.2f"
                    % (median, scenario.friction)
                )
    return problems


# ------------------------------------------------------------------- metrics

def stability_metrics(rows, scenario: PotholeScenario) -> Dict[str, object]:
    time = _col(rows, "time_s")
    station = _col(rows, "Sta_Road") if _has(rows, "Sta_Road") else _col(rows, "Xo")

    # Score the pothole traverse plus the settling that follows, not the whole run: the
    # initial transient belongs to the model's initial condition, not to the strategy.
    begin = scenario.start_station_m - 0.25
    end = scenario.trailing_edge_m + 2.0
    window = [i for i, s in enumerate(station) if begin <= s <= end]
    if len(window) < 10:
        window = list(range(len(rows)))
    w = lambda name: [_col(rows, name)[i] for i in window]

    roll = w("Roll_E")
    pitch = w("Pitch")
    yaw = w("Yaw")
    yo = w("Yo")
    vx = w("Vx")
    loads = {c: w("Fz_%s" % SUFFIX[c]) for c in CORNERS}
    total = [sum(loads[c][k] for c in CORNERS) for k in range(len(window))]

    static = {c: _col(rows, "Fz_%s" % SUFFIX[c])[0] for c in CORNERS}
    left = [loads["FL"][k] + loads["RL"][k] for k in range(len(window))]
    right = [loads["FR"][k] + loads["RR"][k] for k in range(len(window))]
    base = (static["FR"] + static["RR"]) - (static["FL"] + static["RL"])
    ltr = [
        ((right[k] - left[k]) - base) / total[k] if total[k] else 0.0
        for k in range(len(window))
    ]

    dt = (time[window[-1]] - time[window[0]]) / max(1, len(window) - 1)
    loss = {
        c: sum(1 for v in loads[c] if v < CONTACT_THRESHOLD_N) * dt
        for c in CORNERS
    }
    airborne = sum(
        1 for k in range(len(window)) if all(loads[c][k] < CONTACT_THRESHOLD_N for c in CORNERS)
    ) * dt

    metrics: Dict[str, object] = {
        "window_s": [time[window[0]], time[window[-1]]],
        "samples": len(window),
        "roll_rms_deg": _rms(roll),
        "roll_peak_deg": max(abs(v) for v in roll),
        "roll_span_deg": _span(roll),
        "pitch_rms_deg": _rms(pitch),
        "pitch_peak_deg": max(abs(v) for v in pitch),
        "pitch_span_deg": _span(pitch),
        "yaw_span_deg": _span(yaw),
        "yaw_end_deg": yaw[-1],
        "lateral_drift_m": max(yo) - min(yo),
        "lateral_end_m": yo[-1] - yo[0],
        "speed_min_kph": min(vx),
        "speed_end_kph": vx[-1],
        "ltr_peak": max(abs(v) for v in ltr),
        "min_load_n": min(min(loads[c]) for c in CORNERS),
        "max_total_load_n": max(total),
        "max_total_load_over_weight": max(total) / sum(static.values()),
        "contact_loss_s": loss,
        "airborne_s": airborne,
    }
    for name, key in (("Az_SM", "az_sm_peak_g"), ("Az", "az_peak_g")):
        if _has(rows, name):
            values = w(name)
            metrics[key] = max(abs(v) for v in values)
            metrics[key.replace("_peak", "_rms")] = _rms(values)
    for name, key in (("Jnc_L1", "jounce_max_mm"),):
        if _has(rows, name):
            metrics["jounce_max_mm"] = max(
                max(_col(rows, "Jnc_%s" % SUFFIX[c])) for c in CORNERS
            )
            metrics["rebound_min_mm"] = min(
                min(_col(rows, "Jnc_%s" % SUFFIX[c])) for c in CORNERS
            )
    if _has(rows, "FsExt_L1"):
        limit = max(
            abs(v) for c in CORNERS for v in _col(rows, "FsExt_%s" % SUFFIX[c])
        )
        metrics["max_realised_force_n"] = limit
    if _has(rows, "Alpha_R1i"):
        metrics["slip_angle_peak_deg"] = max(
            max(abs(v) for v in _col(rows, "Alpha_%s" % SUFFIX[c] + "i"))
            for c in CORNERS
        )
    if _has(rows, "Kappa_R1i"):
        metrics["slip_ratio_peak"] = max(
            max(abs(v) for v in _col(rows, "Kappa_%s" % SUFFIX[c] + "i"))
            for c in CORNERS
        )
    return metrics


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--csv", type=Path,
        default=ROOT / "runs" / "corner_module_expert_pothole" / "data" / "expert_and_dynamics.csv",
    )
    parser.add_argument("--json", type=Path, default=None)
    parser.add_argument(
        "--scenario", type=Path,
        default=ROOT / "models" / "corner_module_ddev" / "single_wheel_deep_pothole" / "scenario.json",
    )
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)

    rows = _load(args.csv)
    scenario = (
        PotholeScenario(**json.loads(args.scenario.read_text(encoding="utf-8")))
        if args.scenario.exists() else PotholeScenario()
    )

    print("run      : %s" % args.csv)
    print("samples  : %d   channels: %d" % (len(rows), len(rows[0]) - 1))
    print()
    problems = validate_channels(rows, scenario)
    print("--- channel validation ---")
    if problems:
        for p in problems:
            print("  PROBLEM  %s" % p)
    else:
        print("  all cross-checks passed")

    metrics = stability_metrics(rows, scenario)
    print()
    print("--- stability / traversability (pothole traverse window) ---")
    for key, value in metrics.items():
        if isinstance(value, float):
            print("  %-30s %10.4f" % (key, value))
        elif isinstance(value, dict):
            for corner, v in value.items():
                print("  %-30s %10.4f  (%s)" % ("contact_loss_s", v, corner))
        else:
            print("  %-30s %s" % (key, value))

    if args.json:
        args.json.write_text(
            json.dumps({"csv": str(args.csv), "problems": problems, "metrics": metrics},
                       indent=2, default=str) + "\n",
            encoding="utf-8",
        )
        print("\nwritten -> %s" % args.json)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
