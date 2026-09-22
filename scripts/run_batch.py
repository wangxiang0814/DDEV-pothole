"""Run a batch of deep-pothole cases and export an ML-ready dataset.

Usage
-----
    $env:PYTHONPATH='src'
    python scripts\\run_batch.py                  # default sweep, 1 worker
    python scripts\\run_batch.py --workers 4      # process-level parallelism
    python scripts\\run_batch.py --name sweep_a --log-decimation 20

Each case gets its own model directory, native TruckSim history and ``dataset.npz``.
``dataset_index.csv`` collects the per-case quality gates; the batch manifest records
parameters and provenance.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from ddevsim.batch import BatchCase, default_cases, run_batch  # noqa: E402
from ddevsim.calibration import load_or_measure  # noqa: E402
from ddevsim.interface_validation import EXPORT_NAMES, IMPORT_NAMES  # noqa: E402
from ddevsim.pothole_case import SCENARIO_EXPORTS  # noqa: E402
from ddevsim.vehicle_params import load_vehicle  # noqa: E402

CORNERS = ("FL", "FR", "RL", "RR")


def read_tyre_reference_load_n(run_all_par: Path) -> float:
    """Read the tyre dataset's load rating ``FZ_REF`` from the generated model.

    The gate must use the model's own rating: it is a model parameter (this
    platform deliberately raises it above the stock 20 kN because the vehicle's
    static corner load is 22.4 kN), and a hard-coded value would silently make the
    gate either vacuous or impossible after a rebuild.
    """
    import re

    text = Path(run_all_par).read_text(encoding="utf-8", errors="replace")
    match = re.search(r"(?m)^FZ_REF\s+([-+0-9.eE]+)\s*$", text)
    if not match:
        raise ValueError("%s declares no FZ_REF" % run_all_par)
    return float(match.group(1))


def load_gain_matrix(path: Path):
    if not Path(path).exists():
        return None
    table = json.loads(Path(path).read_text(encoding="utf-8"))["gain_matrix_command_to_load"]
    return [[table[j][i] for i in CORNERS] for j in CORNERS]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", default="sweep", help="batch directory name")
    parser.add_argument("--workers", type=int, default=1,
                        help="parallel worker processes (one TruckSim solver each)")
    parser.add_argument("--log-decimation", type=int, default=10,
                        help="CSV decimation; 10 gives 5 ms samples")
    parser.add_argument("--include-unverified", action="store_true",
                        help="include channels whose unit is not established (not advised)")
    parser.add_argument("--baseline-only", action="store_true",
                        help="run just the retained baseline scenario")
    args = parser.parse_args(argv)

    base_model = ROOT / "models" / "hd_utility_ddev"
    base_run_all = base_model / "run_all.par"
    base_simfile = base_model / "simfile.sim"
    model_dir = ROOT / "models" / "hd_utility_ddev" / "single_wheel_deep_pothole"
    export_names = list(EXPORT_NAMES) + list(SCENARIO_EXPORTS)

    calibration = load_or_measure(
        model_dir / "calibration.json", model_dir / "simfile.sim",
        IMPORT_NAMES, export_names,
        model_label="HD Utility DDEV 4x4 Active Suspension",
    )
    vehicle = load_vehicle(base_run_all, static_wheel_load_n=calibration.wheel_load_n)
    gain_matrix = load_gain_matrix(ROOT / "runs" / "_actuator_gain" / "gain_matrix.json")
    if gain_matrix is None:
        print("WARNING: no measured gain matrix; run scripts\\probe_actuator_gain.py first")

    cases = [BatchCase(name="baseline", scenario=default_cases()[0].scenario)]
    if not args.baseline_only:
        cases = default_cases()

    tyre_reference_load_n = read_tyre_reference_load_n(base_run_all)
    print("tyre load reference (FZ_REF) = %.0f N" % tyre_reference_load_n)

    manifest = run_batch(
        cases=cases,
        base_run_all=base_run_all,
        base_simfile=base_simfile,
        run_root=ROOT / "runs" / ("batch_" + args.name),
        vehicle=vehicle,
        calibration=calibration,
        gain_matrix=gain_matrix,
        tyre_reference_load_n=tyre_reference_load_n,
        workers=max(1, args.workers),
        log_decimation=args.log_decimation,
        include_unverified=args.include_unverified,
        batch_name=args.name,
    )

    print("batch      : %s" % manifest["batch_name"])
    print("cases      : %d (%d usable)" % (manifest["case_count"], manifest["usable_count"]))
    print("wall time  : %.1f s for %.1f simulated seconds" % (
        manifest["wall_time_s"], manifest["simulated_seconds"]))
    print("workers    : %d" % manifest["workers"])
    print()
    print("%-14s %-7s %-42s %8s %8s" % ("case", "usable", "failures", "peakFz/x", "roll deg"))
    print("-" * 90)
    for item in manifest["results"]:
        print("%-14s %-7s %-42s %8.2f %8.2f" % (
            item["case"], item["usable"], ",".join(item.get("failures") or [])[:42],
            item.get("peak_load_over_tyre_reference", 0.0), item.get("peak_roll_deg", 0.0)))
    print()
    print("index    -> %s" % manifest["dataset_index"])
    print("manifest -> %s" % (ROOT / "runs" / ("batch_" + args.name) / "batch_manifest.json"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
