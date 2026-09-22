"""Measure the model's static operating point instead of assuming it.

The expert strategy needs two baseline quantities per corner:

* the **static wheel load** (the feedforward is a *change* from this baseline), and
* the **static suspension deflection** (the travel guard needs to know where the
  corner sits inside its +-150 mm travel).

Both are measured by running the real TruckSim model with zero control input for a
short settle interval and reading the final sample.  Nothing is taken from a
datasheet, and a model rebuild cannot silently invalidate the baseline.
"""

from __future__ import annotations

import csv
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Optional, Sequence

from .cosim import run_stepwise
from .units import require_verified, to_si

CORNERS = ("FL", "FR", "RL", "RR")


def _suffix(corner: str) -> str:
    return {"FL": "L1", "FR": "R1", "RL": "L2", "RR": "R2"}[corner]


@dataclass
class StaticCalibration:
    """Measured static operating point of the vehicle model."""

    wheel_load_n: Dict[str, float]
    deflection_m: Dict[str, float]
    settle_s: float
    source_csv: str
    model: str = ""
    #: Relative change in total vertical load between the initial state and the end of
    #: the settle window.  Large values mean the model is not actually at rest.
    total_load_drift_fraction: float = 0.0
    #: True when the drift is small enough that the baseline can be trusted.
    quiet_at_rest: bool = True

    def load(self, corner: str) -> float:
        return float(self.wheel_load_n[corner])

    def deflection(self, corner: str) -> float:
        return float(self.deflection_m[corner])

    def to_json(self) -> str:
        return json.dumps(asdict(self), ensure_ascii=False, indent=2) + "\n"

    @classmethod
    def from_json(cls, path: Path) -> "StaticCalibration":
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls(**payload)


def _zero_command(_time_s, _exports):
    return (0.0,) * 8


def measure_static_state(
    simfile: Path,
    import_names: Sequence[str],
    export_names: Sequence[str],
    target_dir: Path,
    settle_s: float = 0.05,
    model_label: str = "",
) -> StaticCalibration:
    """Run the model with zero input and return its settled static state."""
    simfile = Path(simfile)
    for channel in ("Fz_L1", "CmpS_L1"):
        require_verified(channel)

    # Must be absolute: run_stepwise changes into the simfile's directory, so a
    # relative output path would be resolved against the wrong place and the CSV open
    # would fail.
    target_dir = Path(target_dir).resolve()
    target_dir.mkdir(parents=True, exist_ok=True)
    csv_path = target_dir / "static_calibration.csv"

    result = run_stepwise(
        simfile, _zero_command, csv_path, import_names, export_names,
        log_decimation=1, stop_at_s=settle_s,
    )
    if result["status"] != "COMPLETED":
        raise RuntimeError(
            "static calibration run did not complete: %s" % result.get("error_message", "")
        )

    with csv_path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError("static calibration produced no samples")

    def _mean_of(subset, column: str) -> float:
        return sum(float(row[column]) for row in subset) / len(subset)

    def _values(column: str):
        return [float(row[column]) for row in rows]

    # The static operating point is the *initial* state, not the tail.
    #
    # TruckSim initialises every run by solving for static equilibrium, so sample 0
    # already satisfies sum(Fz) == m*g.  For a model whose suspension rings at rest
    # the tail is a transient and averaging it gives nonsense: on the corner-module
    # control object the tail reads 6488 N against a 13337 N vehicle because the
    # front wheels have lifted by t = 50 ms.  Using the head keeps the measured
    # baseline equal to the solver's own equilibrium.
    head = rows[: max(1, min(3, len(rows)))]
    settled = rows[-max(1, len(rows) // 10):]

    wheel_load = {c: _mean_of(head, "exp_Fz_%s" % _suffix(c)) for c in CORNERS}
    deflection = {
        c: to_si("CmpS_%s" % _suffix(c), _mean_of(head, "exp_CmpS_%s" % _suffix(c)))
        for c in CORNERS
    }

    # Flag a rest state that is not actually at rest: if the total vertical load
    # drifts materially across the settle window the model is ringing or the
    # suspension is outside its tables, and the caller should know before trusting
    # the baseline.
    total_head = sum(wheel_load.values())
    total_tail = sum(_mean_of(settled, "exp_Fz_%s" % _suffix(c)) for c in CORNERS)
    drift = abs(total_tail - total_head) / total_head if total_head else 0.0

    elapsed = float(rows[-1]["time_s"])
    if elapsed < settle_s * 0.5:
        raise RuntimeError(
            "calibration settle window too short (%.4f s); check the model TSTOP"
            % elapsed
        )

    return StaticCalibration(
        wheel_load_n=wheel_load,
        deflection_m=deflection,
        settle_s=elapsed,
        source_csv=str(csv_path),
        model=model_label,
        total_load_drift_fraction=drift,
        quiet_at_rest=drift < 0.05,
    )


def load_or_measure(
    calibration_path: Path, simfile: Path, import_names, export_names, model_label: str = ""
) -> StaticCalibration:
    """Load a stored calibration, or measure and store one if absent."""
    calibration_path = Path(calibration_path)
    if calibration_path.exists():
        return StaticCalibration.from_json(calibration_path)
    calibration = measure_static_state(
        simfile, import_names, export_names, calibration_path.parent,
        model_label=model_label,
    )
    calibration_path.write_text(calibration.to_json(), encoding="utf-8")
    return calibration
