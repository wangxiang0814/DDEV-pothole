"""Guarded one-step I_I FR unloading from a measured safe static baseline."""

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
from ddevsim.static_wheel_lift.config import (
    COUPLED_PROBE, FOLLOWUP_UNLOAD, PRELOAD_GATE, SMALL_UNLOAD,
)
from ddevsim.static_wheel_lift.quintic_trajectory import quintic_step
from ddevsim.static_wheel_lift.staged_unload import (
    IncrementalUnloadController, UnloadSample, WheelLoadLowPass,
)
from ddevsim.static_wheel_lift.suspension_allocator import plan_local_unload_step
from run_static_fr_m1_m2 import analyze
from run_static_fr_m3 import response


WHEELS = ("L1", "R1", "L2", "R2")
EXPORTS = EXPORT_NAMES + SCENARIO_EXPORTS + GEOMETRY_EXPORTS


def _support_margin(export):
    support = np.asarray([[export[f"Xctc_{w}i"], export[f"Yctc_{w}i"]]
                          for w in ("L1", "L2", "R2")], dtype=float)
    fz = np.asarray([export[f"Fz_{w}"] for w in WHEELS], dtype=float)
    frame = np.vstack(((support - support[0]).T, np.ones(3)))
    if (fz[1] < 0 or min(fz[i] for i in (0, 2, 3)) <= 0 or
            abs(np.linalg.det(frame)) < 1e-9):
        return float("-inf")
    contacts = np.asarray([[export[f"Xctc_{w}i"], export[f"Yctc_{w}i"]]
                           for w in WHEELS], dtype=float)
    zmp = (fz @ contacts) / sum(fz)
    com = np.asarray([export["XCG_TM"], export["YCG_TM"]])
    return min(float(np.min(np.linalg.solve(frame, np.r_[xy - support[0], 1.])))
               for xy in (zmp, com))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--m1", type=Path,
                        default=ROOT / "runs/fr_static_ii_ballast_rear350_left500_v2")
    parser.add_argument("--m3", type=Path,
                        default=ROOT / "runs/fr_static_ii_ballast_rear350_left500_v2/m3_slow/gain_matrix.json")
    parser.add_argument("--output", type=Path,
                        default=ROOT / "runs/fr_static_ii_ballast_rear350_left500_v2/small_unload_150n_fsm")
    parser.add_argument("--force-bound-n", type=float,
                        default=SMALL_UNLOAD.force_step_bound_n)
    parser.add_argument("--fr-drop-n", type=float,
                        default=SMALL_UNLOAD.fr_drop_target_n)
    args = parser.parse_args()
    args.output = args.output.resolve()
    source = args.m1 / "model"
    m1 = json.loads((args.m1 / "result.json").read_text(encoding="utf-8"))
    m3 = json.loads(args.m3.read_text(encoding="utf-8"))
    allowed_bound = min(
        SMALL_UNLOAD.force_step_bound_n,
        m3["amplitude_n"] * (SMALL_UNLOAD.validated_online_gain_multiplier
                              if m3.get("updates_since_full_id", 0) > 0 else 1.))
    if (not 0 < args.force_bound_n <= allowed_bound or
            not 0 < args.fr_drop_n <= SMALL_UNLOAD.fr_drop_target_n):
        raise ValueError("requested step exceeds the configured simulation envelope")
    followup = "base_force_n" in m3
    timing = FOLLOWUP_UNLOAD if followup else SMALL_UNLOAD
    base_force = np.asarray(m3["base_force_n"] if followup else [0.] * 4,
                            dtype=float)
    digest = hashlib.sha256((source / "run_all.par").read_bytes()).hexdigest()
    if (m1["M1_status"] != "PASS" or m3["status"] != "PASS" or
            m1["model_sha256"] != digest or
            m3.get("model_sha256", m3.get("source_model_sha256")) != digest or
            m1["metrics"]["min_lambda"] < PRELOAD_GATE.lambda_safe or
            base_force.shape != (4,) or not np.isfinite(base_force).all()):
        raise RuntimeError("requires same safe I_I native M1 and M3")
    if args.force_bound_n * 1.875 / SMALL_UNLOAD.ramp_s > COUPLED_PROBE.max_slew_n_s:
        raise RuntimeError("configured force slew exceeds diagnostic limit")
    contact_csv = (Path(m3["contact_csv"]) if "contact_csv" in m3 else
                   args.m3.parent / "base/native_5ms.csv" if followup else
                   args.m1 / "native_5ms.csv")
    with contact_csv.open(encoding="utf-8", newline="") as stream:
        rows = [r for r in csv.DictReader(stream)
                if m3["settled_window_s"][0] <= float(r["time_s"]) <=
                m3["settled_window_s"][1]]
    if len(rows) < 50:
        raise RuntimeError("insufficient measured baseline contacts")
    support = np.asarray([[np.mean([float(r[f"exp_{axis}ctc_{wheel}i"]) for r in rows])
                           for axis in ("X", "Y")]
                          for wheel in ("L1", "L2", "R2")])
    base = m3["baseline_response"]
    gain = m3["gains"]
    support_frame = np.vstack(((support - support[0]).T, np.ones(3)))
    baseline_margin = min(
        float(np.min(np.linalg.solve(
            support_frame, np.r_[np.asarray(base[key]) - support[0], 1.])))
        for key in ("CoM_xy_m", "ZMP_xy_m"))
    target_margin = max(PRELOAD_GATE.lambda_safe + SMALL_UNLOAD.margin_reserve,
                        baseline_margin + SMALL_UNLOAD.margin_gain_target)
    step = plan_local_unload_step(
        support, base["CoM_xy_m"], gain["CoM_xy_m"],
        base["ZMP_xy_m"], gain["ZMP_xy_m"], base["Fz_n"], gain["Fz_n"],
        base["travel_m"], gain["travel_m"], base["attitude_rad"],
        gain["attitude_rad"], force_bound_n=args.force_bound_n,
        travel_lower_m=[SMALL_UNLOAD.planning_min_travel_mm / 1000] * 4,
        travel_upper_m=[COUPLED_PROBE.max_travel_mm / 1000] * 4,
        attitude_bound_rad=np.deg2rad(COUPLED_PROBE.max_attitude_deg),
        support_floor_n=COUPLED_PROBE.min_wheel_load_n,
        lambda_safe=PRELOAD_GATE.lambda_safe,
        margin_reserve=SMALL_UNLOAD.margin_reserve,
        fr_drop_target_n=args.fr_drop_n,
        stability_margin_target=target_margin)
    if step.status != "OPTIMAL":
        print(json.dumps(asdict(step), indent=2))
        return
    if args.output.exists():
        raise FileExistsError(args.output)
    shutil.copytree(source, args.output / "model", ignore=shutil.ignore_patterns("output"))
    (args.output / "model/output").mkdir()
    par = args.output / "model/run_all.par"
    source_duration = m1["metrics"]["window_end_s"]
    if source_duration != timing.duration_s:
        text, count = re.subn(rf"(?m)^TSTOP {source_duration:g}$",
                              f"TSTOP {timing.duration_s:g}",
                              par.read_text(encoding="utf-8", errors="replace"))
        if count != 2:
            raise RuntimeError("unexpected source duration")
        par.write_text(text, encoding="utf-8")
    supervisor = IncrementalUnloadController(
        target_force_n=step.force_n, baseline_fr_n=base["Fz_n"][1],
        fr_drop_target_n=args.fr_drop_n,
        earliest_start_s=(timing.step_earliest_s if followup else
                          SMALL_UNLOAD.start_s),
        settle_s=SMALL_UNLOAD.settle_s, ramp_s=SMALL_UNLOAD.ramp_s,
        hold_s=SMALL_UNLOAD.hold_s, recovery_s=COUPLED_PROBE.recovery_s,
        lambda_safe=PRELOAD_GATE.lambda_safe,
        support_floor_n=COUPLED_PROBE.min_wheel_load_n,
        travel_lower_mm=COUPLED_PROBE.min_travel_mm,
        travel_upper_mm=COUPLED_PROBE.max_travel_mm,
        max_attitude_deg=COUPLED_PROBE.max_attitude_deg,
        max_rate_deg_s=COUPLED_PROBE.max_rate_deg_s,
        max_vx_kph=COUPLED_PROBE.max_vx_kph,
        max_wheel_rpm=COUPLED_PROBE.max_wheel_rpm,
        fr_tolerance_n=SMALL_UNLOAD.fr_tolerance_n)
    load_filter = WheelLoadLowPass(cutoff_hz=SMALL_UNLOAD.load_filter_cutoff_hz)
    state = {"abort_time_s": None, "abort_reason": None,
             "abort_force_n": None, "last_total_force_n": np.zeros(4),
             "min_margin": float("inf"), "status": "WAIT_STABLE"}

    def command(t, values):
        x = dict(zip(EXPORTS, values))
        raw_loads = tuple(x[f"Fz_{w}"] for w in WHEELS)
        filtered_loads = load_filter.update(t, raw_loads)
        margin = _support_margin(x)
        if t >= SMALL_UNLOAD.start_s and state["abort_time_s"] is None:
            state["min_margin"] = min(state["min_margin"], margin)
        sample = UnloadSample(
            time_s=t, fz_filtered_n=filtered_loads, lambda_min=margin,
            travel_mm=tuple(x[f"Jnc_{w}"] for w in WHEELS),
            roll_deg=x["Roll_E"], pitch_deg=x["Pitch"],
            roll_rate_deg_s=x["AVx"], pitch_rate_deg_s=x["AVy"],
            vx_kph=x["Vx"],
            wheel_speed_rpm=tuple(x[f"AVy_{w}"] for w in WHEELS))
        control = supervisor.update(sample)
        total_force = (base_force * quintic_step(t, FOLLOWUP_UNLOAD.base_start_s,
                       FOLLOWUP_UNLOAD.base_ramp_s)[0] if followup else
                       np.zeros(4)) + np.asarray(control.force_n)
        if (followup and t >= FOLLOWUP_UNLOAD.base_start_s and
                state["abort_time_s"] is None and not supervisor._safe(sample)):
            state["abort_time_s"] = t
            state["abort_force_n"] = state["last_total_force_n"].copy()
            state["abort_reason"] = {
                "reason": "baseline_or_step_safety_limit",
                "loads_raw_n": raw_loads, "loads_filtered_n": filtered_loads,
                "travel_mm": sample.travel_mm, "margin": margin,
                "roll_deg": sample.roll_deg, "pitch_deg": sample.pitch_deg,
                "roll_rate_deg_s": sample.roll_rate_deg_s,
                "pitch_rate_deg_s": sample.pitch_rate_deg_s}
        if control.status == "ABORT_RECOVERY" and state["abort_time_s"] is None:
            state["abort_time_s"] = t
            state["abort_reason"] = {
                "reason": control.reason, "loads_raw_n": raw_loads,
                "loads_filtered_n": filtered_loads,
                "travel_mm": sample.travel_mm, "margin": margin,
                "roll_deg": sample.roll_deg, "pitch_deg": sample.pitch_deg,
                "roll_rate_deg_s": sample.roll_rate_deg_s,
                "pitch_rate_deg_s": sample.pitch_rate_deg_s}
        if state["abort_time_s"] is not None and followup:
            fraction = quintic_step(t, state["abort_time_s"],
                                    COUPLED_PROBE.recovery_s)[0]
            total_force = state["abort_force_n"] * (1. - fraction)
        state["status"] = control.status
        state["last_total_force_n"] = np.asarray(total_force)
        return (0.0,) * 4 + tuple(total_force)

    csv_path = args.output / "native_5ms.csv"
    native = run_stepwise(args.output / "model/simfile.sim", command, csv_path,
                          IMPORT_NAMES, EXPORTS, log_decimation=10)
    settled = analyze(csv_path, settle_start_s=timing.settled_start_s,
                      lambda_safe=PRELOAD_GATE.lambda_safe)
    settled["legacy_four_wheel_gate_status"] = settled.pop("M1_status")
    settled["phase"] = "CONTROL_HOLD; use near-zero hold validation for M6"
    result = {
        "scope": "one guarded M3-sized FR unloading step; no M4 or lift",
        "model_sha256": digest, "allocator": asdict(step),
        "step_force_bound_n": args.force_bound_n,
        "step_fr_drop_target_n": args.fr_drop_n,
        "baseline_margin": baseline_margin,
        "planned_margin_target": target_margin,
        "safety_limits": asdict(COUPLED_PROBE),
        "base_force_n": base_force.tolist(), "followup": followup,
        "native": native, "abort_time_s": state["abort_time_s"],
        "abort_reason": state["abort_reason"],
        "supervisor_status": state["status"],
        "min_online_margin": state["min_margin"], "settled": settled,
        "observed": response(csv_path, timing.settled_start_s,
                             timing.settled_end_s),
        "baseline": base,
    }
    (args.output / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps({key: result[key] for key in
                      ("allocator", "abort_time_s", "abort_reason",
                       "min_online_margin", "settled")}, indent=2))


if __name__ == "__main__":
    main()
