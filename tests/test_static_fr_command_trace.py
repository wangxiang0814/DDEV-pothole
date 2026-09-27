import csv
import gzip

import numpy as np

from ddevsim.static_wheel_lift.command_trace import load_command_trace


def test_command_trace_interpolates_and_holds_last_force(tmp_path):
    path = tmp_path / "trace.csv"
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=("time_s", "imp_A", "imp_B"))
        writer.writeheader()
        writer.writerow({"time_s": 0., "imp_A": 0., "imp_B": 0.})
        writer.writerow({"time_s": 1., "imp_A": 100., "imp_B": -20.})
    trace = load_command_trace(path, ("A", "B"))
    np.testing.assert_allclose(trace.at(0.5), [50., -10.])
    np.testing.assert_allclose(trace.at(2.), [100., -20.])


def test_command_trace_reads_compressed_replay(tmp_path):
    path = tmp_path / "trace.csv.gz"
    with gzip.open(path, "wt", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=("time_s", "imp_A"))
        writer.writeheader()
        writer.writerow({"time_s": 0., "imp_A": 0.})
        writer.writerow({"time_s": 2., "imp_A": 8.})
    trace = load_command_trace(path, ("A",))
    np.testing.assert_allclose(trace.at(1.), [4.])
