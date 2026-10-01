import unittest

import pytest

from ddevsim.cosim import signed_pulse
from ddevsim import cosim


def test_solver_is_terminated_when_controller_raises(monkeypatch, tmp_path):
    calls = []

    class Function:
        def __init__(self, handler):
            self.handler = handler

        def __call__(self, *args):
            return self.handler(*args)

    def configure(_name, n_import, n_export, t_start, t_stop, t_step):
        n_import._obj.value = n_export._obj.value = 1
        t_start._obj.value = 0.0
        t_stop._obj.value = 1.0
        t_step._obj.value = 0.1

    class Solver:
        vs_set_opt_error_dialog = Function(lambda *_: None)
        vs_get_error_message = Function(lambda: b"")
        vs_read_configuration = Function(configure)
        vs_copy_export_vars = Function(lambda exports: None)
        vs_integrate_io = Function(lambda *_: 0)
        vs_terminate_run = Function(lambda time: calls.append(time.value))

    monkeypatch.setattr(cosim.ctypes.cdll, "LoadLibrary", lambda _: Solver())
    monkeypatch.setattr(cosim, "parse_simfile", lambda _: {
        "DLLFILE": str(tmp_path / "dummy.dll"),
        "PORTS_IMP": "1", "PORTS_EXP": "1",
    })
    monkeypatch.setattr(cosim, "solver_input_name", lambda _: b"simfile.sim")

    def broken_controller(_time, _exports):
        raise RuntimeError("controller failed")

    with pytest.raises(RuntimeError, match="controller failed"):
        cosim.run_stepwise(tmp_path / "simfile.sim", broken_controller,
                           tmp_path / "log.csv", ("in",), ("out",))
    assert calls == [0.1]


class CosimTests(unittest.TestCase):
    def test_signed_pulse_is_one_corner_only_and_has_half_open_window(self):
        self.assertEqual(signed_pulse(0.1, 2, -1500.0, 0.2, 0.4), (0.0, 0.0, 0.0, 0.0))
        self.assertEqual(signed_pulse(0.2, 2, -1500.0, 0.2, 0.4), (0.0, 0.0, -1500.0, 0.0))
        self.assertEqual(signed_pulse(0.399, 2, -1500.0, 0.2, 0.4), (0.0, 0.0, -1500.0, 0.0))
        self.assertEqual(signed_pulse(0.4, 2, -1500.0, 0.2, 0.4), (0.0, 0.0, 0.0, 0.0))

    def test_bad_corner_is_rejected(self):
        with self.assertRaises(ValueError):
            signed_pulse(0.3, 4, 1000.0, 0.2, 0.4)


if __name__ == "__main__":
    unittest.main()
