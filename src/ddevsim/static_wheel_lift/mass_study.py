"""Controlled payload scaling for isolated I_I TruckSim sensitivity cases."""

from __future__ import annotations

import math
import re


_PAYLOAD_BLOCK = "M_PL 200\nIXX_PL 100\nIYY_PL 100\nIZZ_PL 100\n"


def scale_payload_mass(model_text: str, new_mass_kg: float) -> str:
    """Scale the three 200 kg payloads and their inertias, preserving geometry."""
    if ("VEHICLE_CODE i_i\n" not in model_text or
            not math.isfinite(new_mass_kg) or not 0 < new_mass_kg <= 200 or
            model_text.count(_PAYLOAD_BLOCK) != 3):
        raise ValueError("expected I_I model with three known 200 kg payload blocks")
    inertia = new_mass_kg / 2
    replacement = (f"M_PL {new_mass_kg:g}\nIXX_PL {inertia:g}\n"
                   f"IYY_PL {inertia:g}\nIZZ_PL {inertia:g}\n")
    return model_text.replace(_PAYLOAD_BLOCK, replacement)


_UNIFORM_FIELDS = {
    "M_US": (80.0, 2), "M_SU": (600.0, 1),
    "IXX_SU": (384.0, 1), "IYY_SU": (624.2, 1), "IZZ_SU": (686.9, 1),
    "M_PL": (200.0, 3), "IXX_PL": (100.0, 3),
    "IYY_PL": (100.0, 3), "IZZ_PL": (100.0, 3),
}


def scale_vehicle_mass(model_text: str, factor: float) -> str:
    """Uniformly scale known I_I masses and nonzero inertias in a copy."""
    if "VEHICLE_CODE i_i\n" not in model_text or not math.isfinite(factor) or not 0 < factor <= 1:
        raise ValueError("expected I_I model and mass factor in (0, 1]")
    modified = model_text
    for key, (expected, count) in _UNIFORM_FIELDS.items():
        pattern = rf"(?m)^{key} ([-+\d.eE]+)$"
        observed = re.findall(pattern, modified)
        if len(observed) != count or any(float(value) != expected for value in observed):
            raise ValueError(f"unexpected {key} declarations")
        modified = re.sub(pattern, lambda match: f"{key} {float(match.group(1)) * factor:g}", modified)
    return modified


_REAR_PAYLOAD_POSITION = (
    "SET_OFFSET_X -1.57500007481\nSET_OFFSET_Y 0\n"
    "SET_OFFSET_Z 0.950000045123\nH_CG_PL 950\n"
    "LX_CG_PL 1575\nY_CG_PL 0\nM_PL 200\n"
)


def shift_rear_payload(model_text: str, *, rearward_mm: float, leftward_mm: float) -> str:
    """Move only the existing 200 kg rear payload in an isolated I_I copy."""
    if ("VEHICLE_CODE i_i\n" not in model_text or
            not all(math.isfinite(v) for v in (rearward_mm, leftward_mm)) or
            not 0 <= rearward_mm <= 350 or not 0 <= leftward_mm <= 500 or
            model_text.count(_REAR_PAYLOAD_POSITION) != 1):
        raise ValueError("unexpected I_I rear payload or offset")
    replacement = (
        f"SET_OFFSET_X {-1.57500007481 - rearward_mm / 1000:.11f}\n"
        f"SET_OFFSET_Y {leftward_mm / 1000:g}\n"
        "SET_OFFSET_Z 0.950000045123\nH_CG_PL 950\n"
        f"LX_CG_PL {1575 + rearward_mm:g}\n"
        f"Y_CG_PL {leftward_mm:g}\nM_PL 200\n"
    )
    return model_text.replace(_REAR_PAYLOAD_POSITION, replacement)
