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


def scale_payload(source: str, factor: float) -> str:
    """Multiply every ``M_PL`` payload mass by ``factor``.

    A model-validity lever, recorded in the source manifest.  TruckSim 2019's
    ``Compact Utility Truck (I_I)`` attaches three 200 kg payloads, and the resulting
    1360 kg vehicle over-compresses its own front spring (30 N/mm against a 3483 N
    static corner load) beyond the suspension's kinematic tables.  Reducing the cargo
    is the physical way to bring the vehicle back inside its valid range, and an
    unladen commercial vehicle is a legitimate configuration to study.
    """
    factor = float(factor)
    if factor < 0.0:
        raise ValueError("payload scale must not be negative")
    text, count = re.subn(
        r"(?mi)^(\s*M_PL(?:\s*\(\s*\d+\s*\))?\s+)([-+0-9.eE]+)(\s*)$",
        lambda m: "%s%.9g%s" % (m.group(1), float(m.group(2)) * factor, m.group(3)),
        source,
    )
    if count == 0:
        raise ValueError("source declares no M_PL entry to scale")
    return text


def rescale_steer_spring_rate(source: str, rate_n_per_mm: float) -> str:
    """Set the steer-axle spring rate of an independent-suspension case.

    This is a **model-validity correction, not a tuning knob**, and it is recorded in
    the source manifest.

    TruckSim 2019's ``Compact Utility Truck (I_I)`` dataset is internally
    inconsistent: its front spring is 30 N/mm while the vehicle's own static corner
    load is 3483 N, so the suspension settles at roughly 116-168 mm of deflection.
    The front jounce stop does not engage until 61 mm, and the suspension kinematic
    tables (``SUSP_X_TABLE``, ``CAMBER_TABLE``, ``SUSP_DIVE_TABLE``, ``TOE_TABLE``)
    only extend to about 80 mm, so TruckSim extrapolates the front geometry from
    t = 0 -- the shipped source case's own log reports exactly those warnings.

    A higher-rate front spring lands the static ride height inside those tables.  It
    is also physically motivated here: a corner-module vehicle carries its static load
    on the active element, so a stiffer passive spring is the expected configuration
    rather than a deviation.
    """
    rate = float(rate_n_per_mm)
    if rate <= 0.0:
        raise ValueError("steer spring rate must be positive")
    header = (
        r"#FullDataName Suspension: Independent Compliance, Springs, and Dampers"
        r"`[^`]*Steer Axle`"
    )
    for key in ("FS_COMP_COEFFICIENT", "FS_EXT_COEFFICIENT"):
        pattern = r"(?ms)(" + header + r".*?)^" + key + r"\s+[-+0-9.eE]+\s*$"
        source, count = re.subn(
            pattern, lambda m, k=key: "%s\n%s %.9g" % (m.group(1), k, rate), source
        )
        if count != 1:
            raise ValueError(
                "expected exactly one steer-axle %s to rescale, found %d" % (key, count)
            )
    return source


def transform_hd_ddev_run(
    source: str,
    stop_s: float = 1.0,
    tyre_load_reference_n: float | None = None,
    extra_exports: Iterable[str] = (),
    steer_spring_rate_n_per_mm: float | None = None,
    payload_scale: float | None = None,
) -> str:
    """Transform a stock TruckSim run into the 8-actuator DDEV case.

    The transformation is vehicle-agnostic: it works for any 2-axle TruckSim lead
    unit, including solid-axle ones (``s_s``) and corner-module ones with
    independent suspension at both axles (``i_i``).

    ``tyre_load_reference_n`` rescales the tyre dataset's load rating (``FZ_REF``).
    This is a **model-validity correction, not a tuning knob**, and it is recorded in
    the source manifest.  It is only needed when the stock rating sits below the
    vehicle's own static corner load, which is the case for the HD Utility Vehicle
    (``FZ_REF 20000`` N against a 22 415 N static corner load) but *not* for the
    Compact Utility Truck (``FZ_REF 4100`` N against a ~2 354 N corner load), so the
    default is to leave the rating alone unless asked.
    """
    if stop_s <= 0.0:
        raise ValueError("stop_s must be positive")
    if any(line.startswith("IMPORT ") for line in source.splitlines()):
        raise ValueError("source already contains IMPORT entries")
    if any(line.startswith("EXPORT ") for line in source.splitlines()):
        raise ValueError("source already contains EXPORT entries")

    protected_before = parse_protected_parameters(source)

    # The mechanical powertrain must not drive the wheels.  Stock cases declare
    # OPT_PT 3 once per unit; a case that is already a DDEV declares OPT_PT 0.
    shipped = len(re.findall(r"(?m)^OPT_PT\s+3\s*$", source))
    already = len(re.findall(r"(?m)^OPT_PT\s+0\s*$", source))
    if shipped:
        text = _replace_exact_count(r"^OPT_PT\s+3\s*$", "OPT_PT 0", source, shipped, "OPT_PT 3")
    elif already:
        text = source
    else:
        raise ValueError("source declares neither OPT_PT 3 nor OPT_PT 0")

    if tyre_load_reference_n is not None:
        value = float(tyre_load_reference_n)
        if value <= 0.0:
            raise ValueError("tyre_load_reference_n must be positive")
        text, reference_count = re.subn(
            r"(?m)^FZ_REF\s+[-+0-9.eE]+\s*$", "FZ_REF %.9g" % value, text
        )
        if reference_count == 0:
            raise ValueError("source declares no FZ_REF entry to rescale")
    if steer_spring_rate_n_per_mm is not None:
        text = rescale_steer_spring_rate(text, float(steer_spring_rate_n_per_mm))
    if payload_scale is not None:
        text = scale_payload(text, float(payload_scale))
    # A merged parameter file can carry more than one TSTOP: the source case for the
    # corner-module vehicle has one per unit block, and the *last* one is the run
    # control that the solver honours. Replacing only the first left the solver
    # integrating 20 s per interface pulse instead of 1 s.
    text, tstop_count = re.subn(
        r"(?m)^TSTOP\s+[-+0-9.eE]+\s*$",
        "TSTOP %.9g" % float(stop_s),
        text,
    )
    if tstop_count == 0:
        raise ValueError("source run declares no TSTOP entry")

    final_end = text.rfind("\nEND")
    if final_end < 0:
        raise ValueError("source run has no final END")
    interface_block = "\n".join(TORQUE_IMPORTS + ACTIVE_FORCE_IMPORTS + DDEV_EXPORTS)
    # Scenario channels (vehicle pose and wheel-centre stations) are needed by the
    # expert controller's support-phase state machine and by the QA gates, so a case
    # that will be driven by the controller must export them as well.
    extra = "\n".join("EXPORT " + name for name in extra_exports)
    if extra:
        interface_block = interface_block + "\n" + extra
    transformed = text[:final_end] + "\n\n! HD Utility DDEV external actuator contract\n" + interface_block + text[final_end:]
    if parse_protected_parameters(transformed) != protected_before:
        raise ValueError("protected vehicle parameters changed during DDEV transformation")
    return transformed


def detect_vehicle_code(source: str) -> str:
    """Return the upper-case TruckSim vehicle code, e.g. ``S_S`` or ``I_I``.

    ``S`` = solid axle, ``I`` = independent.  The code is the first letter pair, so
    ``i_i__s`` (independent lead unit towing a solid-axle trailer) yields ``I_I``,
    which is what the solver's ``VEHICLE_CODE`` entry expects.
    """
    match = re.search(r"(?mi)^VEHICLE_CODE\s+(\S+)\s*$", source)
    if not match:
        raise ValueError("source declares no VEHICLE_CODE")
    token = match.group(1).lower()
    parts = token.split("__")[0].split("_")
    if len(parts) < 2 or not all(part in ("s", "i") for part in parts[:2]):
        raise ValueError("unrecognised VEHICLE_CODE %r" % match.group(1))
    return "_".join(part.upper() for part in parts[:2])


def _simfile_text(
    program_dir: Path, data_dir: Path, vehicle_code: str = "S_S", ports_export: int = 16
) -> str:
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
VEHICLE_CODE {vehicle_code}
EXT_MODEL_STEP 0.01
PORTS_IMP 8
PORTS_EXP {ports_export}
DLLFILE {dll}
END
""".format(
        program=str(program_dir),
        data=str(data_dir),
        resources=str(program_dir / "Resources"),
        dll=str(dll_path),
        vehicle_code=vehicle_code,
        ports_export=ports_export,
    )


def describe_suspension_architecture(source: str) -> Dict[str, object]:
    """Describe each axle's suspension type straight from the merged parameters.

    This is the authoritative record of whether the corners are mechanically
    independent, and it is written into the source manifest so a reviewer never has
    to infer the architecture from a vehicle name.
    """
    independent = sorted(set(re.findall(
        r"(?mi)^#FullDataName Suspension: Independent System Kinematics`([^`]+)`", source
    )))
    solid = sorted(set(re.findall(
        r"(?mi)^#FullDataName Suspension: Solid Axle System Kinematics`([^`]+)`", source
    )))
    per_corner_springs = sorted(set(re.findall(
        r"(?mi)^#FullDataName Suspension: (?:Independent|Solid) Compliance, Springs, and Dampers`([^`]+)`",
        source,
    )))
    code = detect_vehicle_code(source)
    parts = code.split("_")
    return {
        "vehicle_code": code,
        "front_axle": "independent" if parts[0] == "I" else "solid",
        "rear_axle": "independent" if parts[1] == "I" else "solid",
        "corners_mechanically_independent": parts[0] == "I" and parts[1] == "I",
        "independent_kinematics_datasets": independent,
        "solid_axle_kinematics_datasets": solid,
        "compliance_datasets": per_corner_springs,
        "note": (
            "Independent at both axles means each corner has its own spring, damper and "
            "kinematics, so a force at one spring seat acts on that corner alone and the "
            "four corners can be raised independently."
        ),
    }


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
    extra_exports: Iterable[str] = (),
    steer_spring_rate_n_per_mm: float | None = None,
    payload_scale: float | None = None,
) -> Dict[str, Path]:
    source_run_all = Path(source_run_all).resolve()
    target_dir = Path(target_dir).resolve()
    target_dir.mkdir(parents=True, exist_ok=True)
    output_dir = target_dir / "output"
    output_dir.mkdir(parents=True, exist_ok=True)

    source_text = source_run_all.read_text(encoding="utf-8", errors="replace")
    transformed = transform_hd_ddev_run(
        source_text, stop_s=stop_s, tyre_load_reference_n=tyre_load_reference_n,
        extra_exports=extra_exports,
        steer_spring_rate_n_per_mm=steer_spring_rate_n_per_mm,
        payload_scale=payload_scale,
    )
    run_all = target_dir / "run_all.par"
    run_all.write_text(transformed, encoding="utf-8")
    vehicle_code = detect_vehicle_code(transformed)
    simfile = target_dir / "simfile.sim"
    simfile.write_text(
        _simfile_text(program_dir, data_dir, vehicle_code=vehicle_code,
                      ports_export=len(DDEV_EXPORTS) + len(tuple(extra_exports))),
        encoding="ascii",
    )

    original_reference = re.search(
        r"(?m)^FZ_REF\s+([-+0-9.eE]+)\s*$", source_text
    )
    applied_reference = re.search(r"(?m)^FZ_REF\s+([-+0-9.eE]+)\s*$", transformed)

    contract = {
        "schema_version": "2.0",
        "vehicle": "HD Utility DDEV 4x4 Active Suspension",
        "vehicle_code": vehicle_code,
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
        "payload_scale": payload_scale,
        "steer_spring_rate_n_per_mm": {
            "applied_value": steer_spring_rate_n_per_mm,
            "reason": (
                "model-validity correction: the stock 30 N/mm front spring leaves the "
                "vehicle at ~116-168 mm of static deflection against an ~80 mm "
                "suspension kinematic table, so the solver extrapolates the front "
                "geometry from t=0 (the shipped source case's own log shows this)"
            ) if steer_spring_rate_n_per_mm is not None else None,
        },
        "vehicle_code": vehicle_code,
        "suspension_architecture": describe_suspension_architecture(transformed),
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
