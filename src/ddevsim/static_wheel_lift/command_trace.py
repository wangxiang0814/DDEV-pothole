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


@dataclass(frozen=True)
class HoldCommandTrace(CommandTrace):
    def at(self, time_s: float) -> tuple[float, ...]:
        if not np.isfinite(time_s):
            raise ValueError('finite simulation time required')
        index = int(np.clip(np.searchsorted(self.time_s, time_s, side='right') - 1,
                            0, len(self.time_s) - 1))
        return tuple(self.values[index])


def cycle_hold_trace(rows, lifted_corner: str, *, hold_age_s: float):
    """Replay logged commands through a hold point, then freeze all nine inputs."""
    modes = {'FR': 'THREE_WHEEL_HOLD', 'RR': 'RR_HOLD'}
    if lifted_corner not in modes or not np.isfinite(hold_age_s) or hold_age_s < 0:
        raise ValueError('invalid identification contact mode')
    held = [r for r in rows if r['mode'] == modes[lifted_corner]]
    if not held:
        raise ValueError('reference has no requested hold')
    start = float(held[0]['time_s'])
    sample = next((r for r in held if float(r['time_s']) >= start + hold_age_s), None)
    if sample is None:
        raise ValueError('reference hold too short')
    freeze_s = float(sample['time_s'])
    selected = [r for r in rows if float(r['time_s']) <= freeze_s]
    values = []
    for row in selected:
        torques = []
        for c in ('fl', 'fr', 'rl', 'rr'):
            key = f'torque_{c}_nm'
            # FR logs omit its torque; this wheel remains undriven throughout FR.
            if key in row:
                torques.append(float(row[key]))
            elif c == 'fr' and not row['mode'].startswith('RR_'):
                torques.append(0.)
            else:
                raise ValueError(f'missing recorded torque: {key}')
        values.append(torques + [float(row[f'fact_{c}_n'])
                                for c in ('fl', 'fr', 'rl', 'rr')] +
                      [float(row['steer_sw_deg'])])
    times, commands = np.asarray([float(r['time_s']) for r in selected]), np.asarray(values)
    if not np.isfinite(commands).all() or not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError('invalid cycle command history')
    return HoldCommandTrace(times, commands), freeze_s


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
