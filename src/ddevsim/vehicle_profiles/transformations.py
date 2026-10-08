"""Pure vehicle transformations, used by both expert strategies."""
from __future__ import annotations
from pathlib import Path
import re

def configure_crawl_text(content: str, tyre_low_speed_slip_m_s: float = .2) -> str:
    # The stock Pacejka low-speed regularizer is 2 m/s: at a sub-1 km/h
    # crawl it suppresses usable longitudinal tyre force.  This isolated
    # simulation-only copy resolves slip at the intended speed instead.
    for key in ("VLOW_KAPPA", "VLOW_ALPHA"):
        content, changed = re.subn(rf"(?m)^{key} 2$",
                                   f"{key} {tyre_low_speed_slip_m_s:g}",
                                   content)
        if changed != 4:
            raise ValueError(f"expected four {key} tyre parameters")
    # The isolated I_I suspension geometry is simulation-configurable.  The
    # original -100 mm rebound stop is hit during a 3 km/h launch; extend the
    # stop table for this moving study while keeping the static case intact.
    old_rebound = "F_REB_STOP_TABLE LINEAR\n-100, -7000\n-99, 0\n-90, 0"
    new_rebound = "F_REB_STOP_TABLE LINEAR\n-150, -7000\n-149, 0\n-140, 0"
    if content.count(old_rebound) != 4:
        raise ValueError("expected four I_I rebound-stop tables")
    content = content.replace(old_rebound, new_rebound)
    return content


def enable_steering_import(model: Path) -> None:
    """Extend only this copied run with TruckSim's steering wheel angle input."""
    path = model / "run_all.par"
    content = path.read_text(encoding="utf-8")
    anchor = "IMPORT IMP_FS_R2 Add 0.0! 0\n"
    if content.count(anchor) != 1:
        raise ValueError("expected one final suspension import")
    path.write_text(content.replace(anchor, anchor +
                                    "IMPORT IMP_STEER_SW Replace 0.0! 0\n"),
                    encoding="utf-8")
    simfile = model / "simfile.sim"
    content = simfile.read_text(encoding="utf-8")
    if content.count("PORTS_IMP 8") != 1:
        raise ValueError("expected eight original TruckSim imports")
    simfile.write_text(content.replace("PORTS_IMP 8", "PORTS_IMP 9"),
                       encoding="utf-8")


def clamp_kinematics_at_original_bounds(model: Path) -> None:
    """Diagnostic: avoid extrapolating the original ±70 mm ride tables."""
    path = model / "run_all.par"
    content = path.read_text(encoding="utf-8")
    names = ("SUSP_DIVE_TABLE", "SUSP_X_TABLE", "CAMBER_TABLE",
             "SUSP_LAT_TABLE", "TOE_TABLE")
    changed = 0
    for name in names:
        pattern = rf"(?m)^({name} SPLINE\n)((?:[-+0-9.eE]+, [-+0-9.eE]+\n)+)(ENDTABLE)$"

        def extend(match):
            nonlocal changed
            lines = match.group(2).splitlines()
            if len(lines) < 2 or not lines[0].startswith("-70, ") or not lines[-1].startswith("70, "):
                raise ValueError(f"unexpected {name} endpoints")
            changed += 1
            lower = lines[0].split(", ", 1)[1]
            upper = lines[-1].split(", ", 1)[1]
            return (match.group(1) + f"-150, {lower}\n" + match.group(2) +
                    f"150, {upper}\n" + match.group(3))

        content = re.sub(pattern, extend, content)
    if changed != 20:
        raise ValueError(f"expected 20 independent ride tables, found {changed}")
    path.write_text(content, encoding="utf-8")


def scale_lateral_ride_movement(model: Path, factor: float) -> None:
    """Scale lateral wheel-centre movement in the copied I_I suspension."""
    if not 0.0 <= factor <= 1.0:
        raise ValueError("lateral ride scale must be within [0, 1]")
    path = model / "run_all.par"
    content = path.read_text(encoding="utf-8")
    pattern = r"(?m)^(SUSP_LAT_TABLE SPLINE\n)((?:[-+0-9.eE]+, [-+0-9.eE]+\n)+)(ENDTABLE)$"

    def scale(match):
        points = "".join(
            f"{line.split(', ', 1)[0]}, {float(line.split(', ', 1)[1]) * factor:.9g}\n"
            for line in match.group(2).splitlines())
        return match.group(1) + points + match.group(3)

    content, count = re.subn(pattern, scale, content)
    if count != 4:
        raise ValueError(f"expected four lateral kinematic tables, found {count}")
    path.write_text(content, encoding="utf-8")


