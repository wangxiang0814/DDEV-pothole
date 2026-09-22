from __future__ import annotations

import csv
import ctypes
import os
from pathlib import Path
from typing import Any, Callable, Dict, Sequence, Tuple

from .solver_api import _decode_message, parse_simfile, solver_input_name


Vector4 = Tuple[float, float, float, float]


def signed_pulse(
    time_s: float,
    corner_index: int,
    amplitude: float,
    start_s: float,
    stop_s: float,
) -> Vector4:
    if corner_index not in range(4):
        raise ValueError("corner_index must be 0..3 in FL,FR,RL,RR order")
    values = [0.0, 0.0, 0.0, 0.0]
    if start_s <= time_s < stop_s:
        values[corner_index] = float(amplitude)
    return tuple(values)  # type: ignore[return-value]


def run_stepwise(
    simfile: Path,
    command: Callable[[float, Sequence[float]], Sequence[float]],
    csv_path: Path,
    import_names: Sequence[str],
    export_names: Sequence[str],
    log_decimation: int = 20,
    stop_at_s: float | None = None,
) -> Dict[str, Any]:
    """Integrate a TruckSim case step by step with an external controller.

    ``stop_at_s`` caps the loop below the model's configured ``TSTOP`` (used by
    short diagnostic runs such as the static calibration).  The solver is always
    terminated through ``vs_terminate_run`` so a capped run leaves a valid,
    animatable history file.
    """
    simfile = Path(simfile).resolve()
    parsed = parse_simfile(simfile)
    dll_path = Path(parsed["DLLFILE"])
    if len(import_names) != int(parsed.get("PORTS_IMP", len(import_names))):
        raise ValueError("import_names length does not match PORTS_IMP")
    if len(export_names) != int(parsed.get("PORTS_EXP", len(export_names))):
        raise ValueError("export_names length does not match PORTS_EXP")
    csv_path = Path(csv_path)
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    previous_directory = Path.cwd()
    t_current = 0.0
    status = 1
    rows_written = 0
    try:
        dll = ctypes.cdll.LoadLibrary(str(dll_path))
        dll.vs_set_opt_error_dialog.argtypes = [ctypes.c_int]
        dll.vs_set_opt_error_dialog(0)
        dll.vs_get_error_message.argtypes = []
        dll.vs_get_error_message.restype = ctypes.c_char_p
        dll.vs_read_configuration.argtypes = [
            ctypes.c_char_p,
            ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
        ]
        dll.vs_copy_export_vars.argtypes = [ctypes.POINTER(ctypes.c_double)]
        dll.vs_integrate_io.argtypes = [
            ctypes.c_double, ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double)
        ]
        dll.vs_integrate_io.restype = ctypes.c_int
        dll.vs_terminate_run.argtypes = [ctypes.c_double]

        n_import, n_export = ctypes.c_int(), ctypes.c_int()
        t_start, t_stop, t_step = ctypes.c_double(), ctypes.c_double(), ctypes.c_double()
        os.chdir(str(simfile.parent))
        dll.vs_read_configuration(
            solver_input_name(simfile), ctypes.byref(n_import), ctypes.byref(n_export),
            ctypes.byref(t_start), ctypes.byref(t_stop), ctypes.byref(t_step)
        )
        if n_import.value != len(import_names) or n_export.value != len(export_names):
            dll.vs_terminate_run(t_start)
            raise ValueError(
                "solver I/O sizes %d/%d do not match contract %d/%d"
                % (n_import.value, n_export.value, len(import_names), len(export_names))
            )
        imports = (ctypes.c_double * n_import.value)()
        exports = (ctypes.c_double * n_export.value)()
        dll.vs_copy_export_vars(exports)
        t_current = t_start.value
        limit_s = t_stop.value if stop_at_s is None else min(t_stop.value, float(stop_at_s))
        if limit_s <= t_start.value:
            raise ValueError("stop_at_s must be greater than the configured TSTART")
        fieldnames = ["time_s"] + ["imp_" + name for name in import_names] + ["exp_" + name for name in export_names]
        with csv_path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            step_index = 0
            status = 0
            while status == 0 and t_current < limit_s:
                t_current = min(limit_s, t_current + t_step.value)
                values = tuple(float(value) for value in command(t_current, tuple(exports)))
                if len(values) != n_import.value:
                    raise ValueError("command returned %d values, expected %d" % (len(values), n_import.value))
                for index, value in enumerate(values):
                    imports[index] = value
                status = int(dll.vs_integrate_io(ctypes.c_double(t_current), imports, exports))
                if step_index % max(1, log_decimation) == 0 or status != 0:
                    row = {"time_s": "%.9g" % t_current}
                    row.update({"imp_" + name: "%.9g" % imports[i] for i, name in enumerate(import_names)})
                    row.update({"exp_" + name: "%.9g" % exports[i] for i, name in enumerate(export_names)})
                    writer.writerow(row)
                    rows_written += 1
                step_index += 1
        dll.vs_terminate_run(ctypes.c_double(t_current))
        reached_limit = t_current >= limit_s - 1.5 * t_step.value
        reached_configured = t_current >= t_stop.value - 1.5 * t_step.value
        return {
            # Reaching the integration limit is success.  When the loop is capped
            # early the solver has not signalled its own end, so the raw solver
            # status alone cannot be the completion criterion.
            "status": "COMPLETED" if reached_limit else "TERMINATED",
            "solver_status": status,
            "solver_signalled_end": status != 0,
            "final_time_s": t_current,
            "configured_stop_s": t_stop.value,
            "capped_at_s": stop_at_s,
            "reached_configured_stop": reached_configured,
            "rows_written": rows_written,
            "csv": str(csv_path),
            "error_message": _decode_message(dll.vs_get_error_message()),
        }
    finally:
        os.chdir(str(previous_directory))
