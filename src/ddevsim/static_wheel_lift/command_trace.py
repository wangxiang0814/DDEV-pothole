"""Replay the measured input history needed to reach a local operating point."""

from __future__ import annotations

import csv
import gzip
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class CommandTrace:
    time_s: np.ndarray
    values: np.ndarray

    def at(self, time_s: float) -> tuple[float, ...]:
        if not np.isfinite(time_s):
            raise ValueError("finite simulation time required")
        return tuple(float(np.interp(time_s, self.time_s, self.values[:, i]))
                     for i in range(self.values.shape[1]))


def load_command_trace(csv_path: str | Path, import_names) -> CommandTrace:
    names = tuple(import_names)
    if not names:
        raise ValueError("at least one import channel required")
    path = Path(csv_path)
    opener = gzip.open if path.suffix == ".gz" else Path.open
    with opener(path, "rt", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 2:
        raise ValueError("command trace needs at least two samples")
    times = np.asarray([float(r["time_s"]) for r in rows], dtype=float)
    values = np.asarray([[float(r[f"imp_{name}"]) for name in names]
                         for r in rows], dtype=float)
    if (not np.isfinite(times).all() or not np.isfinite(values).all() or
            np.any(np.diff(times) <= 0)):
        raise ValueError("invalid command trace")
    return CommandTrace(times, values)
