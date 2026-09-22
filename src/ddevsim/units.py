"""Authoritative unit contract for every TruckSim 2019 channel the platform uses.

Why this module exists
----------------------
TruckSim 2019 does not write the unit of an ``EXPORT`` variable into the
generated ``run_all.par`` or ``simfile.sim``.  The platform previously named its
logical channels after SI quantities (``wheel_speed_radps``, ``speed_mps``)
while the solver actually returns **rpm** and **km/h**.  Any controller or
training set that trusted those names was wrong by 9.55x and 3.6x respectively.

The units below are therefore *measured*, not assumed.  Each entry records how
it was established so a later change can be re-verified.  The measurement tool is
``scripts/verify_channel_units.py``; it re-runs a real TruckSim diagnostic case
that exports deliberately redundant channels and checks each unknown against an
independently defined quantity.

Verification summary (real TruckSim 2019 run, 9 s, dt = 0.5 ms, 18000 samples)
-----------------------------------------------------------------------------
=========================  ==========  ==============================================
channel                    unit        evidence
=========================  ==========  ==============================================
``AVy_*``                  rpm         ratio to ``60*d(Rot_*)/dt`` = 0.997-1.000
``Vx``                     km/h        ratio to ``3.6*d(Xo)/dt`` = 0.9999
``Xo``                     m           pothole lip at station 101.1 m matches the
                                       measured front-right wheel-load loss
``Z_*`` (wheel centre)     m           ``.vs`` header declares ``m``; used to pin
                                       ``Vz_Wc``
``Fz_*``                   N           four static loads sum to 87296 N vs
                                       ``m*g`` = 8900*9.81 = 87309 N (0.015%)
``Roll_E``, ``Pitch``      deg         ``.vs`` header declares ``deg``
``CmpS_*``                 mm          static corner rate 22420 N / 53.6 mm =
                                       418 N/mm, plausible for a 7 t leaf spring;
                                       range vs the declared +-150 mm travel;
                                       the solver log annotates stop tables in mm
``Vz_Wc_*``                UNVERIFIED  see below
=========================  ==========  ==============================================

``Vz_Wc_*`` — unresolved on purpose
-----------------------------------
``Vz_Wc_*`` correlates strongly with the wheel-centre vertical velocity
(Pearson r up to 0.996 against ``d(Z_*)/dt`` at full resolution), so it clearly
*is* that quantity.  Its scale, however, is not reproducible: depending on the
window and estimator it ranges over 3.1-3.9 relative to m/s, near the m/s->km/h
factor 3.6 but never exact.  A factor of ~3.6 is the most likely reading, which
would be consistent with ``Vx`` also being km/h in this TruckSim variable set.

Because the scale cannot be pinned to better than roughly +-15%, this channel is
deliberately marked :data:`UNVERIFIED` and :func:`require_verified` refuses it.
Controllers must not consume it.  Use a filtered numerical derivative of
``CmpS_*`` (whose unit *is* established) whenever a suspension-deflection rate is
needed.
"""

from __future__ import annotations

import math
from typing import Dict, Mapping

#: Sentinel for a channel whose unit could not be established empirically.
UNVERIFIED = "UNVERIFIED"

#: Authoritative unit for each TruckSim channel the platform consumes.
CHANNEL_UNITS: Dict[str, str] = {
    # --- commanded imports (from the TruckSim import contract) --------------
    "IMP_MYUSM_L1": "N-m", "IMP_MYUSM_R1": "N-m",
    "IMP_MYUSM_L2": "N-m", "IMP_MYUSM_R2": "N-m",
    "IMP_FS_L1": "N", "IMP_FS_R1": "N",
    "IMP_FS_L2": "N", "IMP_FS_R2": "N",
    # --- exported feedback channels ----------------------------------------
    "AVy_L1": "rpm", "AVy_R1": "rpm", "AVy_L2": "rpm", "AVy_R2": "rpm",
    "Fz_L1": "N", "Fz_R1": "N", "Fz_L2": "N", "Fz_R2": "N",
    "CmpS_L1": "mm", "CmpS_R1": "mm", "CmpS_L2": "mm", "CmpS_R2": "mm",
    "Vz_Wc_L1": UNVERIFIED, "Vz_Wc_R1": UNVERIFIED,
    "Vz_Wc_L2": UNVERIFIED, "Vz_Wc_R2": UNVERIFIED,
    "Xo": "m", "Vx": "km/h", "Roll_E": "deg", "Pitch": "deg",
    # Pose channels added for lateral-sway diagnosis.  ``.vs`` header declares
    # Yo/Zo in m and Yaw in deg ("Yaw, vehicle").
    "Yo": "m", "Zo": "m", "Yaw": "deg",
    # --- wheel-centre stations (m) -----------------------------------------
    # Used by the expert controller's support-phase state machine so that no
    # axle-offset constant is hard-coded.  Declared ``m`` in the ``.vs`` header
    # ("X coordinate, wheel center L1"), and validated by the pothole lip: the
    # front-right wheel load collapses when X_R1 reaches the lip station 101.1 m.
    "X_L1": "m", "X_R1": "m", "X_L2": "m", "X_R2": "m",
    # --- probe-only channels (unit probes and diagnostics) -----------------
    "Rot_L1": "rev", "Rot_R1": "rev", "Rot_L2": "rev", "Rot_R2": "rev",
    "Z_L1": "m", "Z_R1": "m", "Z_L2": "m", "Z_R2": "m",
}

#: Factor that converts a value in ``unit`` into SI.
_TO_SI: Dict[str, float] = {
    "N-m": 1.0,
    "N": 1.0,
    "rpm": 2.0 * math.pi / 60.0,   # rpm -> rad/s
    "km/h": 1.0 / 3.6,             # km/h -> m/s
    "mm": 1.0e-3,                  # mm -> m
    "m": 1.0,
    "deg": math.pi / 180.0,        # deg -> rad
    "rev": 2.0 * math.pi,          # revolutions -> rad
}

#: Human-readable SI target for each source unit.
_SI_NAME: Dict[str, str] = {
    "N-m": "N*m",
    "N": "N",
    "rpm": "rad/s",
    "km/h": "m/s",
    "mm": "m",
    "m": "m",
    "deg": "rad",
    "rev": "rad",
}

WHEEL_ORDER = ("FL", "FR", "RL", "RR")

#: Order of the eight control inputs, matching the fixed Trucksim import order.
CONTROL_ORDER = (
    "T_FL", "T_FR", "T_RL", "T_RR",
    "F_FL", "F_FR", "F_RL", "F_RR",
)

#: Maps a logical control name to its TruckSim import channel.
CONTROL_TO_TRUCKSIM = {
    "T_FL": "IMP_MYUSM_L1", "T_FR": "IMP_MYUSM_R1",
    "T_RL": "IMP_MYUSM_L2", "T_RR": "IMP_MYUSM_R2",
    "F_FL": "IMP_FS_L1", "F_FR": "IMP_FS_R1",
    "F_RL": "IMP_FS_L2", "F_RR": "IMP_FS_R2",
}

#: TruckSim wheel suffix for each corner, in FL, FR, RL, RR order.
WHEEL_CHANNEL_SUFFIX = {"FL": "L1", "FR": "R1", "RL": "L2", "RR": "R2"}


class UnverifiedUnitError(ValueError):
    """Raised when a channel with an unestablished unit is used numerically."""


def unit_of(channel: str) -> str:
    """Return the measured unit of ``channel`` (``"UNVERIFIED"`` if unknown)."""
    try:
        return CHANNEL_UNITS[channel]
    except KeyError:
        raise KeyError(
            "channel %r is not in the unit contract; add it to CHANNEL_UNITS after "
            "measuring it with scripts/verify_channel_units.py" % (channel,)
        )


def require_verified(channel: str) -> str:
    """Return the unit of ``channel``, refusing channels with no measured unit."""
    unit = unit_of(channel)
    if unit == UNVERIFIED:
        raise UnverifiedUnitError(
            "unit of %r is not established; do not use it in control or training. "
            "See ddevsim.units module docstring." % (channel,)
        )
    return unit


def si_factor(channel: str) -> float:
    """Multiplier converting ``channel`` values into SI units."""
    return _TO_SI[require_verified(channel)]


def to_si(channel: str, value: float) -> float:
    """Convert a single ``channel`` value into SI units."""
    return float(value) * si_factor(channel)


def si_name(channel: str) -> str:
    """Name of the SI unit that :func:`to_si` produces for ``channel``."""
    return _SI_NAME[require_verified(channel)]


def wheel_channel(prefix: str, corner: str) -> str:
    """Build a per-wheel channel name, e.g. ``("Fz", "FR") -> "Fz_R1"``."""
    if corner not in WHEEL_CHANNEL_SUFFIX:
        raise KeyError("corner must be one of %s" % (WHEEL_ORDER,))
    return "%s_%s" % (prefix, WHEEL_CHANNEL_SUFFIX[corner])


def si_vector(channels: Mapping[str, float]) -> Dict[str, float]:
    """Convert a mapping of channel -> value into SI.

    Raises :class:`UnverifiedUnitError` if any channel lacks a measured unit, so
    that an unverified channel cannot silently reach a controller or dataset.
    """
    return {name: to_si(name, value) for name, value in channels.items()}


def contract_table() -> Dict[str, Dict[str, str]]:
    """Return the full contract as ``{channel: {unit, si_unit, status}}``."""
    table: Dict[str, Dict[str, str]] = {}
    for channel, unit in CHANNEL_UNITS.items():
        if unit == UNVERIFIED:
            table[channel] = {"unit": UNVERIFIED, "si_unit": "", "status": "unverified"}
        else:
            table[channel] = {
                "unit": unit,
                "si_unit": _SI_NAME[unit],
                "status": "measured",
            }
    return table
