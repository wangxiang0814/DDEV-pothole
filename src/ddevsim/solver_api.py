from __future__ import annotations

import ctypes
import os
from pathlib import Path
from typing import Any, Dict


def solver_input_name(path: Path) -> bytes:
    """Return the ANSI-safe argument expected by TruckSim 2019.

    Callers change into ``path.parent`` before invoking the DLL. Passing an
    absolute UTF-8 path crashes this legacy ANSI API when project directories
    contain Chinese characters; the ASCII simfile basename is safe.
    """
    return Path(path).name.encode("ascii")


def parse_simfile(path: Path) -> Dict[str, str]:
    values: Dict[str, str] = {}
    for raw_line in Path(path).read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("!") or " " not in line:
            continue
        key, value = line.split(None, 1)
        values[key.upper()] = value.strip()
    return values


def _decode_message(value) -> str:
    if not value:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace").strip()
    return str(value).strip()


def probe_solver(simfile: Path) -> Dict[str, Any]:
    simfile = Path(simfile).resolve()
    if not simfile.exists():
        return {
            "status": "BLOCKED",
            "reason_code": "SIMFILE_NOT_FOUND",
            "simfile": str(simfile),
        }
    parsed = parse_simfile(simfile)
    dll_value = parsed.get("DLLFILE")
    if not dll_value:
        return {
            "status": "BLOCKED",
            "reason_code": "DLLFILE_NOT_DECLARED",
            "simfile": str(simfile),
        }
    dll_path = Path(dll_value)
    if not dll_path.is_absolute():
        dll_path = (simfile.parent / dll_path).resolve()
    if not dll_path.exists():
        return {
            "status": "BLOCKED",
            "reason_code": "SOLVER_DLL_NOT_FOUND",
            "simfile": str(simfile),
            "dll_path": str(dll_path),
        }

    result: Dict[str, Any] = {
        "status": "ERROR",
        "reason_code": "SOLVER_PROBE_FAILED",
        "simfile": str(simfile),
        "dll_path": str(dll_path),
    }
    previous_directory = Path.cwd()
    try:
        dll = ctypes.cdll.LoadLibrary(str(dll_path))
        dll.vs_set_opt_error_dialog.argtypes = [ctypes.c_int]
        dll.vs_set_opt_error_dialog.restype = None
        dll.vs_set_opt_error_dialog(0)
        dll.vs_get_error_message.argtypes = []
        dll.vs_get_error_message.restype = ctypes.c_char_p
        dll.vs_error_occurred.argtypes = []
        dll.vs_error_occurred.restype = ctypes.c_int
        dll.vs_read_configuration.argtypes = [
            ctypes.c_char_p,
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
            ctypes.POINTER(ctypes.c_double),
        ]
        dll.vs_read_configuration.restype = None
        dll.vs_terminate_run.argtypes = [ctypes.c_double]
        dll.vs_terminate_run.restype = None

        n_import = ctypes.c_int()
        n_export = ctypes.c_int()
        t_start = ctypes.c_double()
        t_stop = ctypes.c_double()
        t_step = ctypes.c_double()
        os.chdir(str(simfile.parent))
        dll.vs_read_configuration(
            solver_input_name(simfile),
            ctypes.byref(n_import),
            ctypes.byref(n_export),
            ctypes.byref(t_start),
            ctypes.byref(t_stop),
            ctypes.byref(t_step),
        )
        error_message = _decode_message(dll.vs_get_error_message())
        error_flag = bool(dll.vs_error_occurred())
        configuration = {
            "n_import": n_import.value,
            "n_export": n_export.value,
            "t_start_s": t_start.value,
            "t_stop_s": t_stop.value,
            "t_step_s": t_step.value,
        }
        if error_flag or t_step.value <= 0.0:
            reason = "TRUCKSIM_LICENSE_OR_CONFIGURATION_BLOCKED"
            if "license" in error_message.lower() or "tslm" in error_message.lower():
                reason = "TRUCKSIM_LICENSE_NOT_SELECTED"
            result.update(
                {
                    "status": "BLOCKED",
                    "reason_code": reason,
                    "error_message": error_message,
                    "configuration": configuration,
                }
            )
        else:
            result.update(
                {
                    "status": "READY",
                    "reason_code": "NONE",
                    "error_message": error_message,
                    "configuration": configuration,
                }
            )
        dll.vs_terminate_run(ctypes.c_double(t_start.value))
    except (OSError, AttributeError, ValueError) as error:
        result["exception"] = "%s: %s" % (type(error).__name__, error)
    finally:
        os.chdir(str(previous_directory))
    return result


def run_native(simfile: Path) -> Dict[str, Any]:
    """Run a complete native VehicleSim case through ``vs_run``.

    This path is used for solver/history smoke tests. Closed-loop external
    commands use the stepwise API only after channel binding is verified.
    """
    simfile = Path(simfile).resolve()
    if not simfile.exists():
        return {"status": "BLOCKED", "reason_code": "SIMFILE_NOT_FOUND", "simfile": str(simfile)}
    parsed = parse_simfile(simfile)
    dll_value = parsed.get("DLLFILE")
    if not dll_value:
        return {"status": "BLOCKED", "reason_code": "DLLFILE_NOT_DECLARED", "simfile": str(simfile)}
    dll_path = Path(dll_value)
    if not dll_path.is_absolute():
        dll_path = (simfile.parent / dll_path).resolve()
    if not dll_path.exists():
        return {
            "status": "BLOCKED",
            "reason_code": "SOLVER_DLL_NOT_FOUND",
            "simfile": str(simfile),
            "dll_path": str(dll_path),
        }

    previous_directory = Path.cwd()
    try:
        dll = ctypes.cdll.LoadLibrary(str(dll_path))
        dll.vs_set_opt_error_dialog.argtypes = [ctypes.c_int]
        dll.vs_set_opt_error_dialog.restype = None
        dll.vs_set_opt_error_dialog(0)
        dll.vs_get_error_message.argtypes = []
        dll.vs_get_error_message.restype = ctypes.c_char_p
        dll.vs_run.argtypes = [ctypes.c_char_p]
        dll.vs_run.restype = ctypes.c_int
        os.chdir(str(simfile.parent))
        return_code = int(dll.vs_run(solver_input_name(simfile)))
        message = _decode_message(dll.vs_get_error_message())
        return {
            "status": "COMPLETED" if return_code == 0 else "FAILED",
            "reason_code": "NONE" if return_code == 0 else "VS_RUN_ERROR",
            "return_code": return_code,
            "error_message": message,
            "simfile": str(simfile),
            "dll_path": str(dll_path),
        }
    except (OSError, AttributeError, ValueError) as error:
        return {
            "status": "ERROR",
            "reason_code": "SOLVER_RUN_EXCEPTION",
            "exception": "%s: %s" % (type(error).__name__, error),
            "simfile": str(simfile),
            "dll_path": str(dll_path),
        }
    finally:
        os.chdir(str(previous_directory))
