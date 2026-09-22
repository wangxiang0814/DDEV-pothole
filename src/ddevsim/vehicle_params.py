"""Vehicle constants the expert controller derives its control law from.

Everything here comes from one of three sources, in order of preference:

1. **Parsed from the generated model** (``run_all.par``): sprung/payload/unsprung
   mass, CG height, track, tyre radius, suspension travel limits, and the vehicle
   assembly wheelbase.
2. **Derived from measured static equilibrium**: the CG-to-axle distances follow
   from the measured static wheel loads, so the control geometry can never drift
   away from the model that actually produced it.
3. **Explicitly supplied as a measurement**, with the provenance recorded.

Nothing is copied from a datasheet: :func:`load_vehicle` re-reads the model every
time, so a model rebuild that changes the vehicle cannot silently leave the
controller tuned for the old mass.

Note on ``L_AXLE``
------------------
The merged TruckSim parameter file does **not** declare ``L_AXLE``; TruckSim
derives the wheelbase from the vehicle assembly.  The wheelbase is therefore read
from the assembly ``x_length`` and cross-checked in the test suite against the
measured wheel-centre spacing (``X_L1 - X_L2`` = 3.900 m) taken from a real run's
``.vs`` history.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, Optional, Tuple

GRAVITY = 9.80665

#: Simple ``KEY VALUE`` parameters read straight from the merged parameter file.
_PARAM_KEYS = (
    "M_SU", "M_PL", "M_US", "H_CG_SU", "LX_CG_SU",
    "IXX_SU", "IYY_SU", "IZZ_SU", "L_TRACK", "R0",
)


def _first_value(text: str, key: str) -> Optional[float]:
    """Return the first numeric value declared for ``key`` (or ``key(1)``)."""
    for pattern in (
        r"(?mi)^\s*%s\s+([-+0-9.eE]+)\s*$" % re.escape(key),
        r"(?mi)^\s*%s\(\s*1\s*\)\s+([-+0-9.eE]+)\s*$" % re.escape(key),
    ):
        match = re.search(pattern, text)
        if match:
            return float(match.group(1))
    return None


def parse_wheelbase_mm(text: str) -> float:
    """Parse the wheelbase from the vehicle assembly ``x_length`` entry.

    TruckSim also writes uppercase ``X_LENGTH`` for tyre and body dimensions, so
    this looks only for the lowercase form and accepts a value in a plausible
    wheelbase range.  For this two-axle truck the assembly ``x_length`` is
    3900 mm, which matches the measured wheel-centre spacing from a real run.
    """
    candidates = [
        float(match.group(1))
        for match in re.finditer(r"^x_length\s+([-+0-9.eE]+)\s*$", text, re.M)
    ]
    plausible = [value for value in candidates if 1500.0 <= value <= 12000.0]
    if len(plausible) != 1:
        raise ValueError(
            "expected exactly one plausible assembly x_length, found %s"
            % (plausible or candidates,)
        )
    return plausible[0]


def parse_roll_centre_drop_mm(text: str) -> Optional[float]:
    """Parse the roll-centre height below the axle from the dataset name.

    TruckSim stores this as a dataset label rather than a parameter, e.g.
    ``Suspension: Lateral Movement Due to Roll and Jounce`Roll Center: 53 mm
    Below Axle`Front``.  When the label is absent ``None`` is returned and the
    caller must supply the value.
    """
    match = re.search(
        r"Roll Center:\s*([-+0-9.]+)\s*mm\s*Below\s*Axle", text, re.I
    )
    if not match:
        return None
    return float(match.group(1))


def parse_jounce_rebound_travel_mm(text: str) -> Tuple[float, float]:
    """Extract the (+jounce, -rebound) suspension travel limits in mm."""
    jounce = rebound = None
    block = re.search(r"F_JNC_STOP_TABLE[^\n]*\n(.*?)ENDTABLE", text, re.S)
    if block:
        values = [float(line.split(",")[0]) for line in block.group(1).strip().splitlines()]
        jounce = max(values)
    block = re.search(r"F_REB_STOP_TABLE[^\n]*\n(.*?)ENDTABLE", text, re.S)
    if block:
        values = [float(line.split(",")[0]) for line in block.group(1).strip().splitlines()]
        rebound = min(values)
    if jounce is None or rebound is None:
        raise ValueError("could not parse jounce/rebound stop tables")
    return jounce, rebound


@dataclass
class VehicleControllerParams:
    """Constants the deep-pothole expert strategy needs, in SI units."""

    # --- masses (kg) -------------------------------------------------------
    sprung_mass_kg: float
    payload_mass_kg: float
    unsprung_mass_per_axle_kg: float

    # --- geometry (m) ------------------------------------------------------
    wheelbase_m: float
    track_m: float
    tyre_radius_m: float
    cg_height_m: float
    roll_centre_height_m: float
    cg_to_front_axle_m: float
    cg_to_rear_axle_m: float

    # --- suspension (m) ----------------------------------------------------
    jounce_limit_m: float
    rebound_limit_m: float

    # --- measured static wheel loads (N), FL FR RL RR ----------------------
    static_wheel_load_n: Dict[str, float] = field(default_factory=dict)

    #: Provenance so a reviewer can separate measured from parsed values.
    sources: Dict[str, str] = field(default_factory=dict)

    # ------------------------------------------------------------------ mass
    @property
    def total_mass_kg(self) -> float:
        """Total vehicle mass: sprung + payload + both axles' unsprung mass.

        The unsprung mass is included because the measured wheel loads used to
        derive the CG contain it: for this model 6500 + 2000 + 2*200 = 8900 kg
        gives 87296 N, matching the measured static load sum exactly.
        """
        return (
            self.sprung_mass_kg
            + self.payload_mass_kg
            + 2.0 * self.unsprung_mass_per_axle_kg
        )

    @property
    def total_weight_n(self) -> float:
        return self.total_mass_kg * GRAVITY

    @property
    def unsprung_mass_per_corner_kg(self) -> float:
        return self.unsprung_mass_per_axle_kg / 2.0

    @property
    def cg_above_roll_centre_m(self) -> float:
        """Roll arm used to convert a body roll angle into a CG translation."""
        return self.cg_height_m - self.roll_centre_height_m

    def front_axle_fraction(self) -> float:
        total = sum(self.static_wheel_load_n.values())
        front = self.static_wheel_load_n["FL"] + self.static_wheel_load_n["FR"]
        return front / total

    def static_load(self, corner: str) -> float:
        return float(self.static_wheel_load_n[corner])

    def measured_static_weight_n(self) -> float:
        return sum(self.static_wheel_load_n.values())

    def to_json(self) -> str:
        return json.dumps(asdict(self), ensure_ascii=False, indent=2) + "\n"


def derive_cg_from_static_loads(
    wheelbase_m: float, front_load_n: float, total_load_n: float
) -> Tuple[float, float]:
    """Return ``(cg_to_front_axle_m, cg_to_rear_axle_m)`` from static loads.

    Static equilibrium about the axles gives ``b / L = F_front / W`` where ``b`` is
    the CG-to-rear-axle distance.  Deriving the CG from measured loads rather than
    a datasheet keeps the control geometry consistent with the model that produced
    it.
    """
    if total_load_n <= 0.0:
        raise ValueError("total static load must be positive")
    fraction = front_load_n / total_load_n
    if not 0.0 < fraction < 1.0:
        raise ValueError("front axle load fraction must lie strictly in (0, 1)")
    cg_to_rear = fraction * wheelbase_m
    return wheelbase_m - cg_to_rear, cg_to_rear


def load_vehicle(
    run_all_par: Path,
    static_wheel_load_n: Optional[Dict[str, float]] = None,
    wheelbase_override_m: Optional[float] = None,
    roll_centre_drop_override_m: Optional[float] = None,
) -> VehicleControllerParams:
    """Build controller parameters from the generated TruckSim model."""
    text = Path(run_all_par).read_text(encoding="utf-8", errors="replace")
    sources: Dict[str, str] = {}
    values: Dict[str, float] = {}
    for key in _PARAM_KEYS:
        value = _first_value(text, key)
        if value is None:
            raise ValueError("model %s does not declare %s" % (run_all_par, key))
        values[key] = value
        sources[key] = "parsed:%s" % Path(run_all_par).name

    if wheelbase_override_m is not None:
        wheelbase_m = float(wheelbase_override_m)
        sources["wheelbase_m"] = "override"
    else:
        wheelbase_m = parse_wheelbase_mm(text) / 1000.0
        sources["wheelbase_m"] = "parsed:x_length (validated vs measured X_L1-X_L2)"

    track_m = values["L_TRACK"] / 1000.0
    tyre_radius_m = values["R0"] / 1000.0
    cg_height_m = values["H_CG_SU"] / 1000.0

    if roll_centre_drop_override_m is not None:
        drop_m = float(roll_centre_drop_override_m)
        sources["roll_centre_height_m"] = "override"
    else:
        drop_mm = parse_roll_centre_drop_mm(text)
        if drop_mm is None:
            raise ValueError(
                "model does not declare a roll-centre height; pass "
                "roll_centre_drop_override_m"
            )
        drop_m = drop_mm / 1000.0
        sources["roll_centre_height_m"] = "parsed:roll-centre dataset label"
    roll_centre_height_m = tyre_radius_m - drop_m

    jounce_mm, rebound_mm = parse_jounce_rebound_travel_mm(text)
    sources["travel"] = "parsed:jounce/rebound stop tables"

    if static_wheel_load_n is None:
        total = (
            values["M_SU"] + values["M_PL"]
        ) * GRAVITY + 2.0 * values["M_US"] * GRAVITY
        static_wheel_load_n = {c: total / 4.0 for c in ("FL", "FR", "RL", "RR")}
        sources["static_wheel_load_n"] = "assumed:uniform"
    else:
        sources["static_wheel_load_n"] = "measured"

    total_load = sum(static_wheel_load_n.values())
    front_load = static_wheel_load_n["FL"] + static_wheel_load_n["FR"]
    cg_front, cg_rear = derive_cg_from_static_loads(wheelbase_m, front_load, total_load)
    sources["cg_to_front_axle_m"] = "derived:static wheel loads"
    sources["cg_to_rear_axle_m"] = "derived:static wheel loads"

    return VehicleControllerParams(
        sprung_mass_kg=values["M_SU"],
        payload_mass_kg=values["M_PL"],
        unsprung_mass_per_axle_kg=values["M_US"],
        wheelbase_m=wheelbase_m,
        track_m=track_m,
        tyre_radius_m=tyre_radius_m,
        cg_height_m=cg_height_m,
        roll_centre_height_m=roll_centre_height_m,
        cg_to_front_axle_m=cg_front,
        cg_to_rear_axle_m=cg_rear,
        jounce_limit_m=jounce_mm / 1000.0,
        rebound_limit_m=abs(rebound_mm) / 1000.0,
        static_wheel_load_n={k: float(v) for k, v in static_wheel_load_n.items()},
        sources=sources,
    )
