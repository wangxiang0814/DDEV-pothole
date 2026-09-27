"""Replay a validated near-zero FR preload, try a smooth FR lift, then land."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import re
import shutil
import sys
from dataclasses import asdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

from ddevsim.cosim import run_stepwise
from ddevsim.interface_validation import IMPORT_NAMES, EXPORT_NAMES
from ddevsim.pothole_case import SCENARIO_EXPORTS
from ddevsim.contact_geometry import GEOMETRY_EXPORTS
from ddevsim.static_wheel_lift.command_trace import load_command_trace
from ddevsim.static_wheel_lift.config import COUPLED_PROBE, LIFT_PROBE, PRELOAD_GATE
from ddevsim.static_wheel_lift.quintic_trajectory import quintic_step
from run_static_fr_small_unload import _support_margin
from run_static_fr_m3 import response


WHEELS = ("L1", "R1", "L2", "R2")
EXPORTS = EXPORT_NAMES + SCENARIO_EXPORTS + GEOMETRY_EXPORTS


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--m1", type=Path, required=True)
    parser.add_argument("--preload", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--rl-support-n", type=float, default=0.,
                        help="additional RL suspension force during three-wheel hold")
    parser.add_argument("--fl-support-n", type=float, default=0.)
    parser.add_argument("--rr-support-n", type=float, default=0.)
    parser.add_argument("--fr-lift-n", type=float,
                        default=LIFT_PROBE.extra_fr_force_n)
    parser.add_argument("--fr-support-comp-n", type=float, default=0.,
                        help="FR force compensation synchronized with support adjustment")
    parser.add_argument("--support-start-s", type=float, default=None,
                        help="start measured support redistribution before or during lift")
    args = parser.parse_args()
    support_request = np.array([args.fl_support_n, args.rl_support_n,
                                args.rr_support_n], dtype=float)
    if (not np.isfinite(support_request).all() or
            np.max(np.abs(support_request)) > LIFT_PROBE.max_support_adjust_n):
        raise ValueError("support adjustment exceeds simulation envelope")
    if not np.isfinite(args.fr_lift_n) or args.fr_lift_n >= 0:
        raise ValueError("FR lift force must be finite and negative")
    if (not np.isfinite(args.fr_support_comp_n) or
            abs(args.fr_support_comp_n) > LIFT_PROBE.max_support_adjust_n):
        raise ValueError("FR support compensation exceeds simulation envelope")
    support_start_s = (LIFT_PROBE.start_s + LIFT_PROBE.ramp_s
                       if args.support_start_s is None else args.support_start_s)
    if not np.isfinite(support_start_s) or not 40. <= support_start_s <= 52.:
        raise ValueError("support start must follow the validated near-zero preload")
    args.output = args.output.resolve()
    m1 = json.loads((args.m1 / "result.json").read_text(encoding="utf-8"))
    prior = json.loads(args.preload.read_text(encoding="utf-8"))
    source = args.m1 / "model"
    digest = hashlib.sha256((source / "run_all.par").read_bytes()).hexdigest()
    if (m1["M1_status"] != "PASS" or m1["model_sha256"] != digest or
            prior["model_sha256"] != digest or
            prior["native"]["status"] != "COMPLETED" or
            prior["abort_time_s"] is not None or
            prior["observed"]["Fz_n"][1] > LIFT_PROBE.near_zero_fr_n or
            prior["settled"]["metrics"]["min_lambda"] < PRELOAD_GATE.lambda_safe):
        raise RuntimeError("requires safe completed near-zero FR preload")
    trace_path = Path(prior["native"]["csv"])
    if not trace_path.is_absolute():
        trace_path = args.preload.parent / trace_path
    trace = load_command_trace(trace_path, IMPORT_NAMES)
    if args.output.exists():
        raise FileExistsError(args.output)
    shutil.copytree(source, args.output / "model", ignore=shutil.ignore_patterns("output"))
    (args.output / "model/output").mkdir()
    end_s = (LIFT_PROBE.start_s + LIFT_PROBE.ramp_s + LIFT_PROBE.hold_s +
             LIFT_PROBE.lower_s + LIFT_PROBE.return_to_four_s + 2.)
    par = args.output / "model/run_all.par"
    old_end = m1["metrics"]["window_end_s"]
    content, count = re.subn(rf"(?m)^TSTOP {old_end:g}$", f"TSTOP {end_s:g}",
                             par.read_text(encoding="utf-8", errors="replace"))
    if count != 2:
        raise RuntimeError("unexpected source duration")
    par.write_text(content, encoding="utf-8")
    radii_mm = [float(value) for value in re.findall(
        r"(?m)^R0\s+([0-9.]+)\s*$", content)]
    if len(radii_mm) != 4 or max(radii_mm) != min(radii_mm):
        raise RuntimeError("four equal tyre unloaded radii required")
    tyre_radius_m = radii_mm[0] / 1000.
    lift_end = LIFT_PROBE.start_s + LIFT_PROBE.ramp_s
    lower_start = lift_end + LIFT_PROBE.hold_s
    return_start = lower_start + LIFT_PROBE.lower_s
    state = {"abort_time_s": None, "abort_reason": None,
             "abort_extra_n": 0., "abort_support_n": np.zeros(3),
             "min_margin_after_start": float("inf")}

    def command(t, values):
        x = dict(zip(EXPORTS, values))
        base = np.asarray(trace.at(t), dtype=float)
        if t < min(LIFT_PROBE.start_s, support_start_s):
            return tuple(base)
        margin = _support_margin(x)
        state["min_margin_after_start"] = min(state["min_margin_after_start"], margin)
        loads = [x[f"Fz_{wheel}"] for wheel in WHEELS]
        travels = [x[f"Jnc_{wheel}"] for wheel in WHEELS]
        healthy = (min(loads[i] for i in (0, 2, 3)) >=
                   COUPLED_PROBE.min_wheel_load_n and
                   (loads[1] > LIFT_PROBE.near_zero_fr_n or
                    margin >= PRELOAD_GATE.lambda_safe) and
                   min(travels) >= COUPLED_PROBE.min_travel_mm and
                   max(travels) <= COUPLED_PROBE.max_travel_mm and
                   max(abs(x["Roll_E"]), abs(x["Pitch"])) <=
                   COUPLED_PROBE.max_attitude_deg and
                   max(abs(x["AVx"]), abs(x["AVy"])) <=
                   LIFT_PROBE.max_transition_rate_deg_s and
                   abs(x["Vx"]) <= LIFT_PROBE.max_transition_vx_kph)
        if t < lift_end:
            extra = args.fr_lift_n * quintic_step(
                t, LIFT_PROBE.start_s, LIFT_PROBE.ramp_s)[0]
        elif t < lower_start:
            extra = args.fr_lift_n
        else:
            extra = args.fr_lift_n * (1. - quintic_step(
                t, lower_start, LIFT_PROBE.lower_s)[0])
        if t < lower_start:
            support_fraction = quintic_step(
                t, support_start_s, LIFT_PROBE.support_adjust_ramp_s)[0]
        else:
            support_fraction = 1. - quintic_step(
                t, lower_start, LIFT_PROBE.lower_s)[0]
        support_extra = support_request * support_fraction
        extra += args.fr_support_comp_n * support_fraction
        if state["abort_time_s"] is None and not healthy:
            state["abort_time_s"] = t
            state["abort_extra_n"] = extra
            state["abort_support_n"] = support_extra.copy()
            state["abort_reason"] = {
                "loads_n": loads, "travel_mm": travels,
                "margin": margin, "pitch_deg": x["Pitch"],
                "roll_deg": x["Roll_E"], "pitch_rate_deg_s": x["AVy"],
                "roll_rate_deg_s": x["AVx"], "vx_kph": x["Vx"]}
        if state["abort_time_s"] is not None:
            fraction = quintic_step(t, state["abort_time_s"],
                                    LIFT_PROBE.recovery_s)[0]
            extra = state["abort_extra_n"] * (1. - fraction)
            support_extra = state["abort_support_n"] * (1. - fraction)
            scale = 1. - quintic_step(
                t, state["abort_time_s"] + LIFT_PROBE.recovery_s,
                LIFT_PROBE.return_to_four_s)[0]
        else:
            scale = 1. - quintic_step(t, return_start,
                                      LIFT_PROBE.return_to_four_s)[0]
        base *= scale
        base[IMPORT_NAMES.index("IMP_FS_R1")] += extra
        for wheel, delta in zip(("L1", "L2", "R2"), support_extra):
            base[IMPORT_NAMES.index(f"IMP_FS_{wheel}")] += delta
        return tuple(base)

    csv_path = args.output / "native_5ms.csv"
    native = run_stepwise(args.output / "model/simfile.sim", command, csv_path,
                          IMPORT_NAMES, EXPORTS, log_decimation=10)
    with csv_path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    samples = []
    early_liftoff_s = None
    for row in rows:
        t = float(row["time_s"])
        if support_start_s <= t < LIFT_PROBE.start_s and (
                float(row["exp_Z_R1"]) - float(row["exp_Zgnd_R1i"]) -
                tyre_radius_m >= LIFT_PROBE.clearance_positive_m and
                float(row["exp_Fz_R1"]) <= LIFT_PROBE.near_zero_fr_n and
                early_liftoff_s is None):
            early_liftoff_s = t
        if t < LIFT_PROBE.start_s:
            continue
        z = float(row["exp_Z_R1"])
        ground = float(row["exp_Zgnd_R1i"])
        samples.append((t, z - ground - tyre_radius_m,
                        float(row["exp_Fz_R1"])))
    lifted = [s for s in samples if s[1] >= LIFT_PROBE.clearance_positive_m and
              s[2] <= LIFT_PROBE.near_zero_fr_n]
    target = [s for s in samples if s[1] >= LIFT_PROBE.clearance_target_m and
              s[2] <= LIFT_PROBE.near_zero_fr_n]
    hold = [s for s in target if lift_end <= s[0] <= lower_start]
    result = {
        "status": ("EARLY_LIFTOFF" if early_liftoff_s is not None else
                   "LIFT_HOLD_PASS" if state["abort_time_s"] is None and
                   hold and hold[-1][0] - hold[0][0] >= 5. else
                   "NO_VERIFIED_LIFT"),
        "model_sha256": digest, "config": asdict(LIFT_PROBE),
        "fr_lift_force_n": args.fr_lift_n,
        "support_start_s": support_start_s,
        "early_liftoff_s": early_liftoff_s,
        "fr_support_comp_n": args.fr_support_comp_n,
        "support_delta_n": support_request.tolist(),
        "tyre_unloaded_radius_m": tyre_radius_m,
        "native": native, "abort_time_s": state["abort_time_s"],
        "abort_reason": state["abort_reason"],
        "min_margin_after_start": state["min_margin_after_start"],
        "max_clearance_m": max((s[1] for s in samples), default=None),
        "first_positive_clearance_s": lifted[0][0] if lifted else None,
        "first_target_clearance_s": target[0][0] if target else None,
        "target_clearance_hold_s": hold[-1][0] - hold[0][0] if hold else 0.,
        "near_zero_preload": prior["observed"],
        "post_return": response(csv_path, end_s - 2., end_s - 1.),
    }
    (args.output / "result.json").write_text(json.dumps(result, indent=2),
                                               encoding="utf-8")
    print(json.dumps({k: result[k] for k in
                      ("status", "abort_time_s", "abort_reason",
                       "max_clearance_m", "first_target_clearance_s",
                       "target_clearance_hold_s", "post_return")}, indent=2))


if __name__ == "__main__":
    main()
