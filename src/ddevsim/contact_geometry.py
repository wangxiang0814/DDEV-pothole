"""Conservative tyre-road clearance and three-support geometry.

The physical road is TruckSim's bilinear S-L dZ grid, not the separately
rendered Animator mesh. All positions are metres in the straight-road frame.
"""

from __future__ import annotations

import math
import re
from typing import Callable, Mapping, Sequence

from .support_geometry import support_triangle_margin_m


_CORNER_TO_TYRE = {"FL": "L1", "FR": "R1", "RL": "L2", "RR": "R2"}
GEOMETRY_EXPORTS = tuple(
    axis + "ctc_" + tyre + "i"
    for axis in ("X", "Y")
    for tyre in ("L1", "R1", "L2", "R2")
) + ("XCG_TM", "YCG_TM")


def augment_geometry_exports(run_all_text: str, simfile_text: str) -> tuple[str, str]:
    """Instrument an isolated generated case without changing its plant.

    Callers must preserve the Task-1 frozen model and write these returned
    strings only to a fresh probe copy. The model's physical parameters and
    input channels are unchanged; only ten native outputs are appended.
    """
    declared = re.search(r"(?m)^PORTS_EXP\s+(\d+)\s*$", simfile_text)
    exports = re.findall(r"(?m)^EXPORT\s+(\S+)\s*$", run_all_text)
    if declared is None or int(declared.group(1)) != len(exports):
        raise ValueError("generated case export count mismatch")
    if any(name in exports for name in GEOMETRY_EXPORTS):
        raise ValueError("already has geometry exports")
    terminal_end = re.search(r"(?m)^END[ \t]*(?:\r?\n)?\Z", run_all_text)
    if terminal_end is None:
        raise ValueError("generated run_all.par has no END")
    newline = "\r\n" if "\r\n" in run_all_text else "\n"
    augmented = (
        run_all_text[:terminal_end.start()]
        + "".join("EXPORT " + name + newline for name in GEOMETRY_EXPORTS)
        + run_all_text[terminal_end.start():]
    )
    updated_simfile = (
        simfile_text[:declared.start(1)] + str(len(exports) + len(GEOMETRY_EXPORTS))
        + simfile_text[declared.end(1):]
    )
    return augmented, updated_simfile


def native_support_geometry(
    exports: Mapping[str, float], support_corners: Sequence[str]
) -> tuple[dict[str, tuple[float, float]], tuple[float, float]]:
    """Read TruckSim's actual tyre contact XY and instant total-vehicle CG.

    `X_L1/Y_L1` are *wheel centres*, not the tyre contact patches.  The
    contact coordinates and `XCG_TM/YCG_TM` must be explicitly exported in
    the native model.  No wheel-centre fallback is permitted for a safety
    decision, especially near a pit lip.
    """
    names = ["exp_XCG_TM", "exp_YCG_TM"]
    if len(set(support_corners)) != len(support_corners):
        raise ValueError("duplicate support corner")
    for corner in support_corners:
        if corner not in _CORNER_TO_TYRE:
            raise ValueError("unknown support corner: " + corner)
        tyre = _CORNER_TO_TYRE[corner]
        names.extend(("exp_Xctc_" + tyre + "i", "exp_Yctc_" + tyre + "i"))
    missing = [name for name in names if name not in exports]
    if missing:
        raise ValueError("missing native geometry: " + ", ".join(missing))
    values = {name: float(exports[name]) for name in names}
    if not all(math.isfinite(value) for value in values.values()):
        raise ValueError("nonfinite native geometry")
    contacts = {
        corner: (
            values["exp_Xctc_" + _CORNER_TO_TYRE[corner] + "i"],
            values["exp_Yctc_" + _CORNER_TO_TYRE[corner] + "i"],
        )
        for corner in support_corners
    }
    return contacts, (values["exp_XCG_TM"], values["exp_YCG_TM"])


def audit_native_geometry_rows(
    rows: Sequence[Mapping[str, float]],
    body_masses: Sequence[tuple[float, float, float, float]],
    unsprung_mass_per_corner_kg: float,
    load_floor_n: float = 300.0,
) -> dict:
    """Compare independent mass reconstruction with native instant CG.

    Contact-versus-wheel-centre offset is measured only for load-bearing
    tyres; `Xctc/Yctc` is not a valid support vertex when that tyre is free.
    """
    if not rows:
        raise ValueError("native geometry audit requires rows")
    max_cg_error = 0.0
    max_contact_offset = 0.0
    loaded_contacts = 0
    for row in rows:
        _, native_cg = native_support_geometry(row, tuple(_CORNER_TO_TYRE))
        wheels = {
            corner: (float(row["exp_X_" + tyre]), float(row["exp_Y_" + tyre]))
            for corner, tyre in _CORNER_TO_TYRE.items()
        }
        reconstructed_cg = combined_cg_projection_xy(
            (float(row["exp_Xo"]), float(row["exp_Yo"])),
            (float(row["exp_Roll_E"]), float(row["exp_Pitch"]), float(row["exp_Yaw"])),
            body_masses,
            wheels,
            unsprung_mass_per_corner_kg,
        )
        max_cg_error = max(max_cg_error, math.dist(reconstructed_cg, native_cg))
        contacts, _ = native_support_geometry(row, tuple(_CORNER_TO_TYRE))
        for corner, tyre in _CORNER_TO_TYRE.items():
            if float(row["exp_Fz_" + tyre]) > load_floor_n:
                max_contact_offset = max(max_contact_offset,
                                         math.dist(contacts[corner], wheels[corner]))
                loaded_contacts += 1
    return {
        "rows": len(rows),
        "loaded_contact_samples": loaded_contacts,
        "max_cg_reconstruction_error_m": max_cg_error,
        "max_loaded_contact_wheel_center_offset_m": max_contact_offset,
    }


def _parameter(text: str, name: str) -> float:
    match = re.search(r"(?m)^\s*%s\s+([-+0-9.eE]+)(?:\s|$)" % re.escape(name), text)
    if match is None:
        raise ValueError("missing vehicle mass parameter: " + name)
    return float(match.group(1))


def parse_body_mass_points(text: str) -> tuple[tuple[float, float, float, float], ...]:
    """Read the actual sprung and per-payload mass centres from `run_all.par`.

    Local +X is forward and `LX_CG_*` is a positive distance behind the
    sprung-mass origin, so local X is its negative in metres.
    """
    points = [(
        _parameter(text, "M_SU"),
        -_parameter(text, "LX_CG_SU") / 1000.0,
        _parameter(text, "Y_CG_SU") / 1000.0,
        _parameter(text, "H_CG_SU") / 1000.0,
    )]
    blocks = re.findall(
        r"(?s)ENTER_PARSFILE Payloads\\[^\n]*\n(.*?)EXIT_PARSFILE Payloads\\[^\n]*",
        text,
    )
    for block in blocks:
        if not re.search(r"(?m)^M_PL\s+", block):
            continue
        points.append((
            _parameter(block, "M_PL"),
            -_parameter(block, "LX_CG_PL") / 1000.0,
            _parameter(block, "Y_CG_PL") / 1000.0,
            _parameter(block, "H_CG_PL") / 1000.0,
        ))
    return tuple(points)


def parse_echo_total_cg_xy(echo_text: str) -> tuple[float, float]:
    """Initial laden-vehicle CG in local XY from the solver's Echo file."""
    x = _parameter(echo_text.replace("! ", ""), "LX_CG_TL")
    y = _parameter(echo_text.replace("! ", ""), "Y_CG_TL")
    return -x / 1000.0, y / 1000.0


def _ramp(value: float, start: float, end: float) -> float:
    if end <= start:
        raise ValueError("road transition must have positive width")
    return max(0.0, min(1.0, (value - start) / (end - start)))


def pothole_road_height_m(scenario, station_m: float, lateral_m: float) -> float:
    """Evaluate the generated `ROAD_DZ_CARPET 2D_LINEAR` pit height."""
    s0, s1 = scenario.leading_edge_m, scenario.trailing_edge_m
    y0, y1 = scenario.lateral_min_m, scenario.lateral_max_m
    edge = float(scenario.edge_transition_m)
    if not 0.0 < 2.0 * edge < min(s1 - s0, y1 - y0):
        raise ValueError("road edge transition must fit inside the pit")
    longitudinal = min(_ramp(station_m, s0, s0 + edge),
                       1.0 - _ramp(station_m, s1 - edge, s1))
    lateral = min(_ramp(lateral_m, y0, y0 + edge),
                  1.0 - _ramp(lateral_m, y1 - edge, y1))
    return -float(scenario.depth_m) * longitudinal * lateral


def pothole_tyre_envelope_gap_m(
    scenario, wheel_xyz: tuple[float, float, float], radius_m: float
) -> float:
    """Exact minimum clearance over the generated piecewise-linear pit road.

    On each longitudinal road segment, the lower circular tyre envelope minus
    the linear road height has at most one interior minimum.  Checking that
    point and every segment endpoint avoids the centimetre-scale optimistic
    error caused by sparse samples at the short exit ramp.
    """
    if not math.isfinite(radius_m) or radius_m <= 0.0:
        raise ValueError("tyre radius must be positive and finite")
    xc, yc, zc = (float(value) for value in wheel_xyz)
    left, right = xc - radius_m, xc + radius_m
    s0, s1 = scenario.leading_edge_m, scenario.trailing_edge_m
    edge = float(scenario.edge_transition_m)
    knots = [left] + sorted(
        x for x in (s0, s0 + edge, s1 - edge, s1) if left < x < right
    ) + [right]
    road = lambda x: pothole_road_height_m(scenario, x, yc)

    def gap(x: float) -> float:
        dx = x - xc
        tyre_low = zc - math.sqrt(max(0.0, radius_m * radius_m - dx * dx))
        return tyre_low - road(x)

    candidates = list(knots)
    for a, b in zip(knots, knots[1:]):
        slope = (road(b) - road(a)) / (b - a)
        stationary = xc + radius_m * slope / math.sqrt(1.0 + slope * slope)
        if a < stationary < b:
            candidates.append(stationary)
    return min(gap(x) for x in candidates)


def tyre_envelope_gap_m(
    wheel_xyz: tuple[float, float, float],
    radius_m: float,
    road_height: Callable[[float, float], float],
    fore_aft_samples: int = 21,
) -> float:
    """Minimum lower tyre-circle gap to the physical road under its footprint.

    Unloaded radius is deliberately conservative for a separated tyre. A
    positive value means the whole sampled fore-aft envelope clears the road;
    `Fz≈0` without this clearance is not a successful lift.
    """
    if not math.isfinite(radius_m) or radius_m <= 0.0:
        raise ValueError("tyre radius must be positive and finite")
    if fore_aft_samples < 3 or fore_aft_samples % 2 == 0:
        raise ValueError("fore_aft_samples must be odd and at least three")
    x_c, y_c, z_c = (float(value) for value in wheel_xyz)
    gaps = []
    for index in range(fore_aft_samples):
        dx = radius_m * (2.0 * index / (fore_aft_samples - 1) - 1.0)
        lower_z = z_c - math.sqrt(max(0.0, radius_m * radius_m - dx * dx))
        gaps.append(lower_z - float(road_height(x_c + dx, y_c)))
    return min(gaps)


def combined_cg_projection_xy(
    body_origin_xy: tuple[float, float],
    attitude_deg: tuple[float, float, float],
    body_masses: Sequence[tuple[float, float, float, float]],
    unsprung_wheel_xy: Mapping[str, tuple[float, float]],
    unsprung_mass_per_corner_kg: float,
) -> tuple[float, float]:
    """Project explicit sprung/payload masses plus four unsprung masses.

    Body mass tuples are `(kg, local_x_m, local_y_m, local_z_m)` relative to
    the exported body reference. The reference and signs still require native
    calibration; this function is a geometric calculation, not validation.
    """
    if set(unsprung_wheel_xy) != {"FL", "FR", "RL", "RR"}:
        raise ValueError("all four unsprung wheel locations are required")
    roll, pitch, yaw = (math.radians(float(a)) for a in attitude_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    total_mass = 0.0
    sum_x = 0.0
    sum_y = 0.0
    for mass, x, y, z in body_masses:
        if mass <= 0.0:
            raise ValueError("body mass must be positive")
        y_roll = y * cr - z * sr
        z_roll = y * sr + z * cr
        x_pitch = x * cp + z_roll * sp
        x_world = body_origin_xy[0] + x_pitch * cy - y_roll * sy
        y_world = body_origin_xy[1] + x_pitch * sy + y_roll * cy
        sum_x += mass * x_world
        sum_y += mass * y_world
        total_mass += mass
    if unsprung_mass_per_corner_kg <= 0.0:
        raise ValueError("unsprung corner mass must be positive")
    for x_world, y_world in unsprung_wheel_xy.values():
        sum_x += unsprung_mass_per_corner_kg * x_world
        sum_y += unsprung_mass_per_corner_kg * y_world
        total_mass += unsprung_mass_per_corner_kg
    return sum_x / total_mass, sum_y / total_mass


def support_risk(
    state: Mapping[str, float],
    contact_xy: Mapping[str, tuple[float, float]],
    cg_xy: tuple[float, float],
    cg_error_m: float,
) -> dict:
    """Conservative geometric/dynamic support check, never a certified ZMP."""
    if not math.isfinite(cg_error_m) or cg_error_m < 0.0:
        raise ValueError("cg_error_m must be nonnegative and finite")
    if len(contact_xy) != 3:
        raise ValueError("exactly three support contacts are required")
    loads = {corner: float(state["Fz_" + corner]) for corner in contact_xy}
    total_load = sum(loads.values())
    if total_load <= 0.0:
        raise ValueError("support load sum must be positive")
    cop_xy = (
        sum(loads[c] * contact_xy[c][0] for c in contact_xy) / total_load,
        sum(loads[c] * contact_xy[c][1] for c in contact_xy) / total_load,
    )
    margin = support_triangle_margin_m(contact_xy, cg_xy)
    cop_margin = support_triangle_margin_m(contact_xy, cop_xy)
    projected_roll = abs(float(state["roll_deg"])) + (
        float(state["roll_projection_tau_s"]) * abs(float(state["roll_rate_deg_s"]))
    )
    floor = max(300.0, float(state["support_floor_n"]))
    target_clear = not bool(state.get("target_free", 0.0)) or (
        float(state["target_fz_n"]) <= 150.0
        and float(state["target_gap_m"]) > 0.0
    )
    verified = (
        cg_error_m <= 0.005
        and margin - cg_error_m >= 0.01
        and cop_margin >= float(state["cop_margin_required_m"])
        and min(loads.values()) >= floor
        and projected_roll <= float(state["roll_limit_deg"])
        and target_clear
    )
    return {
        "verified": verified,
        "margin_m": margin,
        "conservative_margin_m": margin - cg_error_m,
        "cop_xy": cop_xy,
        "cop_margin_m": cop_margin,
        "projected_roll_deg": projected_roll,
        "min_support_load_n": min(loads.values()),
        "target_clear": target_clear,
    }
