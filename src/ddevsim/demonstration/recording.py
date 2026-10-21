"""Read-only integration-boundary tap with causal action alignment."""
import json
import math
from pathlib import Path
import struct

import numpy as np

from ddevsim.units import CHANNEL_UNITS, si_factor, si_name

SENSOR_NAMES = (
    'Vx', 'Vy', 'Roll_E', 'Pitch', 'Yaw', 'AVx', 'AVy', 'AVz', 'Ax', 'Ay', 'Az',
    *(f'AVy_{w}' for w in ('L1', 'R1', 'L2', 'R2')),
    *(f'Jnc_{w}' for w in ('L1', 'R1', 'L2', 'R2')),
    *(f'JncR_{w}' for w in ('L1', 'R1', 'L2', 'R2')),
    *(f'FsExt_{w}' for w in ('L1', 'R1', 'L2', 'R2')),
)


def unit_info(name):
    if name == 'IMP_STEER_SW':
        return {'native_unit': 'deg', 'si_unit': 'rad', 'si_factor': math.pi / 180}
    return {'native_unit': CHANNEL_UNITS[name], 'si_unit': si_name(name),
            'si_factor': si_factor(name)}


class TransitionRecorder:
    def __init__(self, directory, imports, exports):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.imports = list(imports)
        self.sensors = [n for n in SENSOR_NAMES if n in exports]
        self.indices = [list(exports).index(n) for n in self.sensors]
        self.schema = {
            'version': 1, 'dtype': '<f8',
            'columns': ['observation_time_s', 'action_end_time_s'] + self.sensors + self.imports,
            'sensors': self.sensors, 'actions': self.imports,
            'units': {n: unit_info(n) for n in self.sensors + self.imports},
            'unavailable_sensors': [n for n in SENSOR_NAMES if n not in exports],
            'observation_source': 'plant_truth_sensor_like_channels; no added sensor noise',
            'action_semantics': 'actual command submitted to vs_integrate_io over (observation_time, action_end_time]',
            'policy_excluded': 'global CoM/contact geometry/wheel loads/FSM/scene ground truth',
            'wheel_order': ['FL', 'FR', 'RL', 'RR'],
            'complete': False,
        }
        self.stream = (self.directory / 'transitions.bin').open('xb')
        self.pack = struct.Struct('<%dd' % len(self.schema['columns']))
        self.count = 0
        self.last_end = None
        try:
            self._write_schema()
        except Exception:
            self.stream.close()
            raise

    def _write_schema(self):
        (self.directory / 'transitions.json').write_text(
            json.dumps(self.schema, indent=2), encoding='utf-8')

    def append(self, before, after, observations, actions):
        row = [before, after] + [float(observations[i]) for i in self.indices] + list(actions)
        if (len(actions) != len(self.imports) or not all(math.isfinite(v) for v in row)
                or after <= before or (self.last_end is not None and abs(before-self.last_end) > 1e-8)):
            raise ValueError('invalid/non-contiguous transition')
        self.stream.write(self.pack.pack(*row))
        self.count += 1
        self.last_end = after

    def finish(self, time_s, observations):
        if self.last_end is None or abs(time_s-self.last_end) > 1e-8:
            raise ValueError('final observation must match final transition')
        state = [float(observations[i]) for i in self.indices]
        if not all(math.isfinite(x) for x in state):
            raise ValueError('nonfinite final observation')
        self.schema.update(complete=True, count=self.count, final_time_s=time_s,
                           final_observation=state)
        self.close()

    def close(self):
        self.stream.close()
        self._write_schema()


class TransitionReader:
    def __init__(self, directory):
        directory = Path(directory)
        self.schema = json.loads((directory / 'transitions.json').read_text(encoding='utf-8'))
        if not self.schema['complete']:
            raise ValueError('incomplete transition log')
        self.rows = np.memmap(directory / 'transitions.bin', dtype='<f8', mode='r').reshape(
            -1, len(self.schema['columns']))
        if len(self.rows) != self.schema['count']:
            raise ValueError('truncated transition log')
        self.times = self.rows[:, 0]
        self.final_time = self.schema['final_time_s']
        self.ns = len(self.schema['sensors'])

    def sample(self, time_s):
        if not math.isfinite(time_s) or time_s < self.times[0] or time_s > self.final_time:
            raise ValueError('image time is outside recorded simulation')
        i = min(len(self.rows)-1, int(np.searchsorted(self.times, time_s, side='right'))-1)
        row = self.rows[i]
        fraction = (time_s-row[0])/(row[1]-row[0])
        left = row[2:2+self.ns]
        right = (self.rows[i+1, 2:2+self.ns] if i+1 < len(self.rows)
                 else np.asarray(self.schema['final_observation']))
        state = left + fraction * (right-left)
        units = self.schema['units']
        return {
            'time_s': float(time_s),
            'state_si': {n: float(v*units[n]['si_factor'])
                         for n, v in zip(self.schema['sensors'], state)},
            'action_si': {n: float(v*units[n]['si_factor'])
                          for n, v in zip(self.schema['actions'], row[2+self.ns:])},
            'state_bracket_s': [float(row[0]), float(row[1])],
            'state_interpolation_fraction': float(fraction),
            'action_interval_s': [float(row[0]), float(row[1])],
        }
