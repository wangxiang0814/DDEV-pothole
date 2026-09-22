from __future__ import annotations

import hashlib
import json
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Iterable, Tuple


TORQUE_IMPORTS: Tuple[str, ...] = (
    "IMPORT IMP_MYUSM_L1 Add 0.0! 0",
    "IMPORT IMP_MYUSM_R1 Add 0.0! 0",
    "IMPORT IMP_MYUSM_L2 Add 0.0! 0",
    "IMPORT IMP_MYUSM_R2 Add 0.0! 0",
)

ACTIVE_FORCE_IMPORTS: Tuple[str, ...] = (
    "IMPORT IMP_FS_L1 Add 0.0! 0",
    "IMPORT IMP_FS_R1 Add 0.0! 0",
    "IMPORT IMP_FS_L2 Add 0.0! 0",
    "IMPORT IMP_FS_R2 Add 0.0! 0",
)

DDEV_EXPORTS: Tuple[str, ...] = tuple(
    "EXPORT " + name
    for name in (
        "AVy_L1", "AVy_R1", "AVy_L2", "AVy_R2",
        "Fz_L1", "Fz_R1", "Fz_L2", "Fz_R2",
        "CmpS_L1", "CmpS_R1", "CmpS_L2", "CmpS_R2",
        "Vz_Wc_L1", "Vz_Wc_R1", "Vz_Wc_L2", "Vz_Wc_R2",
    )
)

PROTECTED_KEYS = {
    "M_SU", "M_PL", "M_US", "IXX_SU", "IYY_SU", "IZZ_SU", "IXZ_SU",
    "L_AXLE", "L_TRACK", "R_TIRE", "K_S", "C_S", "K_ARB",
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def parse_protected_parameters(text: str) -> Tuple[str, ...]:
    """Return protected physical parameter lines in source order.

    Exact normalized lines are used so the builder can prove that the DDEV
    interface transformation did not silently re-parameterize the vehicle.
    """
    result = []
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line or line.startswith(("!", "#")):
            continue
        key = line.split(None, 1)[0].upper()
        if key in PROTECTED_KEYS:
            result.append(line)
    return tuple(result)


def _replace_exact_count(pattern: str, replacement: str, text: str, count: int, label: str) -> str:
    updated, actual = re.subn(pattern, replacement, text, flags=re.MULTILINE)
    if actual != count:
        raise ValueError("expected exactly %d %s entries, found %d" % (count, label, actual))
    return updated


def transform_hd_ddev_run(
    source: str,
    stop_s: float = 1.0,
    tyre_load_reference_n: float | None = None,
) -> str:
    """Transform a stock TruckSim run into the 8-actuator DDEV case.

    ``tyre_load_reference_n`` rescales the tyre dataset's load rating (``FZ_REF``).
    This is a **model-validity correction, not a tuning knob**, and it is recorded in
    the source manifest.  The stock model labels its tyre ``2000 kg Rating`` with
    ``FZ_REF 20000``, but this vehicle's own static corner load is 22 415 N
    (8900 kg / 4 = 2.2 t per corner), so the tyre is under-rated at standstill and
    the solver extrapolates its load axis from about 1.96 x FZ_REF = 39.3 kN
    upwards.  Every dynamic load transfer then leaves the table, which is what made
    the first deep-pothole runs unusable.  Raising the rating makes the model valid
    across the real operating range; set it to ``None`` to keep the stock value.
    """
    if stop_s <= 0.0:
        raise ValueError("stop_s must be positive")
    if any(line.startswith("IMPORT ") for line in source.splitlines()):
        raise ValueError("source already contains IMPORT entries")
    if any(line.startswith("EXPORT ") for line in source.splitlines()):
        raise ValueError("source already contains EXPORT entries")

    protected_before = parse_protected_parameters(source)
    text = _replace_exact_count(r"^OPT_PT\s+3\s*$", "OPT_PT 0", source, 2, "OPT_PT 3")
    if tyre_load_reference_n is not None:
        value = float(tyre_load_reference_n)
        if value <= 0.0:
            raise ValueError("tyre_load_reference_n must be positive")
        text, reference_count = re.subn(
            r"(?m)^FZ_REF\s+[-+0-9.eE]+\s*$", "FZ_REF %.9g" % value, text
        )
        if reference_count == 0:
            raise ValueError("source declares no FZ_REF entry to rescale")
    text, tstop_count = re.subn(
        r"^TSTOP\s+[-+0-9.eE]+\s*$",
        "TSTOP %.9g" % float(stop_s),
        text,
        count=1,
        flags=re.MULTILINE,
    )
    if tstop_count not in (0, 1):
        raise ValueError("expected at most one TSTOP entry")

    final_end = text.rfind("\nEND")
    if final_end < 0:
        raise ValueError("source run has no final END")
    interface_block = "\n".join(TORQUE_IMPORTS + ACTIVE_FORCE_IMPORTS + DDEV_EXPORTS)
    transformed = text[:final_end] + "\n\n! HD Utility DDEV external actuator contract\n" + interface_block + text[final_end:]
    if parse_protected_parameters(transformed) != protected_before:
        raise ValueError("protected vehicle parameters changed during DDEV transformation")
    return transformed


def _simfile_text(program_dir: Path, data_dir: Path) -> str:
    program_dir = Path(program_dir).resolve()
    data_dir = Path(data_dir).resolve()
    dll_path = program_dir / "Programs" / "solvers" / "trucksim_64.dll"
    return """SIMFILE
FILEBASE output\\hd_utility_ddev
INPUT run_all.par
INPUTARCHIVE output\\hd_utility_ddev_all.par
ECHO output\\hd_utility_ddev_echo.par
FINAL output\\hd_utility_ddev_end.par
LOGFILE output\\hd_utility_ddev_log.txt
ERDFILE output\\hd_utility_ddev.erd
PROGDIR {program}\\
DATADIR {data}\\
RESOURCEDIR {resources}\\
PRODUCT_ID TruckSim
PRODUCT_VER 2019.0
VEHICLE_CODE S_S
EXT_MODEL_STEP 0.01
PORTS_IMP 8
PORTS_EXP 16
DLLFILE {dll}
END
""".format(
        program=str(program_dir),
        data=str(data_dir),
        resources=str(program_dir / "Resources"),
        dll=str(dll_path),
    )


def _extract_parameter_values(text: str, names: Iterable[str]) -> Dict[str, list]:
    result: Dict[str, list] = {name: [] for name in names}
    for raw_line in text.splitlines():
        parts = raw_line.strip().split(None, 1)
        if len(parts) == 2 and parts[0].upper() in result:
            result[parts[0].upper()].append(parts[1].strip())
    return result


def build_hd_ddev_case(
    source_run_all: Path,
    target_dir: Path,
    program_dir: Path,
    data_dir: Path,
    stop_s: float = 1.0,
    tyre_load_reference_n: float | None = None,
) -> Dict[str, Path]:
    source_run_all = Path(source_run_all).resolve()
    target_dir = Path(target_dir).resolve()
    target_dir.mkdir(parents=True, exist_ok=True)
    output_dir = target_dir / "output"
    output_dir.mkdir(parents=True, exist_ok=True)

    source_text = source_run_all.read_text(encoding="utf-8", errors="replace")
    transformed = transform_hd_ddev_run(
        source_text, stop_s=stop_s, tyre_load_reference_n=tyre_load_reference_n
    )
    run_all = target_dir / "run_all.par"
    run_all.write_text(transformed, encoding="utf-8")
    simfile = target_dir / "simfile.sim"
    simfile.write_text(_simfile_text(program_dir, data_dir), encoding="ascii")

    original_reference = re.search(
        r"(?m)^FZ_REF\s+([-+0-9.eE]+)\s*$", source_text
    )
    applied_reference = re.search(r"(?m)^FZ_REF\s+([-+0-9.eE]+)\s*$", transformed)

    contract = {
        "schema_version": "2.0",
        "vehicle": "HD Utility DDEV 4x4 Active Suspension",
        "wheel_order": ["FL", "FR", "RL", "RR"],
        "imports": [line.split()[1] for line in TORQUE_IMPORTS + ACTIVE_FORCE_IMPORTS],
        "import_units": ["N-m"] * 4 + ["N"] * 4,
        "import_modes": ["ADD"] * 8,
        "exports": [line.split()[1] for line in DDEV_EXPORTS],
    }
    contract_path = target_dir / "interface_contract.json"
    contract_path.write_text(json.dumps(contract, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "source_run_all": str(source_run_all),
        "source_sha256": _sha256(source_run_all),
        "generated_run_all": str(run_all),
        "generated_sha256": _sha256(run_all),
        "program_dir": str(Path(program_dir).resolve()),
        "data_dir": str(Path(data_dir).resolve()),
        "protected_parameters": list(parse_protected_parameters(transformed)),
        "parameter_values": _extract_parameter_values(
            transformed,
            ("M_SU", "M_PL", "M_US", "IXX_SU", "IYY_SU", "IZZ_SU", "IXZ_SU", "L_AXLE", "L_TRACK", "TSTEP"),
        ),
        "powertrain": "disabled (OPT_PT 0)",
        "tyre_load_reference_n": {
            "source_value": float(original_reference.group(1)) if original_reference else None,
            "applied_value": float(applied_reference.group(1)) if applied_reference else None,
            "reason": (
                "model-validity correction: the stock 2000 kg rating (FZ_REF 20000 N) is "
                "below this vehicle's 22415 N static corner load, so the solver "
                "extrapolates the tyre load axis above ~39.3 kN"
            ),
        },
        "ports_import": 8,
        "ports_export": 16,
    }
    manifest_path = target_dir / "source_manifest.json"
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return {
        "run_all": run_all,
        "simfile": simfile,
        "contract": contract_path,
        "manifest": manifest_path,
        "output_dir": output_dir,
    }
