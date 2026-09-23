"""Unit-contract regression tests.

The platform previously labelled its channels with SI names (``wheel_speed_radps``,
``speed_mps``) while TruckSim actually returns rpm and km/h.  These tests lock in
the measured units and, more importantly, assert that a channel whose unit has not
been established cannot be used numerically.
"""

import math
import unittest
from unittest import mock

from ddevsim import units
from ddevsim.units import (
    CHANNEL_UNITS,
    CONTROL_ORDER,
    CONTROL_TO_TRUCKSIM,
    UNVERIFIED,
    UnverifiedUnitError,
    contract_table,
    require_verified,
    si_factor,
    si_name,
    si_vector,
    to_si,
    unit_of,
    wheel_channel,
)


class UnitContractTests(unittest.TestCase):
    def test_measured_units_match_the_empirical_probe(self):
        # Values established by scripts/verify_channel_units.py against redundant
        # TruckSim channels (Rot_* in revolutions, Xo in metres, Fz summing to m*g).
        self.assertEqual(unit_of("AVy_L1"), "rpm")
        self.assertEqual(unit_of("Vx"), "km/h")
        self.assertEqual(unit_of("Fz_R2"), "N")
        self.assertEqual(unit_of("CmpS_L2"), "mm")
        self.assertEqual(unit_of("Roll_E"), "deg")
        self.assertEqual(unit_of("Pitch"), "deg")
        self.assertEqual(unit_of("Xo"), "m")
        self.assertEqual(unit_of("X_R1"), "m")
        self.assertEqual(unit_of("Rot_L1"), "rev")

    def test_wheel_speed_is_rpm_not_rad_per_second(self):
        # The distinction that silently corrupted the old contract: 1 rpm is
        # 0.1047 rad/s, a factor of 9.55.
        self.assertAlmostEqual(si_factor("AVy_L1"), 2.0 * math.pi / 60.0, places=12)
        self.assertAlmostEqual(to_si("AVy_L1", 60.0), 2.0 * math.pi, places=12)
        self.assertNotAlmostEqual(to_si("AVy_L1", 60.0), 60.0, places=3)

    def test_vehicle_speed_is_kmh_not_mps(self):
        self.assertAlmostEqual(to_si("Vx", 3.6), 1.0, places=12)
        self.assertEqual(si_name("Vx"), "m/s")

    def test_lengths_and_angles_convert_to_si(self):
        self.assertAlmostEqual(to_si("CmpS_R1", 53.6), 0.0536, places=12)
        self.assertAlmostEqual(to_si("Roll_E", 180.0), math.pi, places=12)
        self.assertAlmostEqual(to_si("Xo", 101.1), 101.1, places=12)

    def test_unverified_channel_is_refused_numerically(self):
        # Vz_Wc is now a *verified* channel: TruckSim's own output catalogue declares
        # Vz_WC_* in km/h ("Vz at wheel center L1"), which is exactly the ~3.6 factor the
        # platform had measured but declined to pin.
        self.assertEqual(unit_of("Vz_Wc_R1"), "km/h")
        self.assertAlmostEqual(si_factor("Vz_Wc_R1"), 1.0 / 3.6, places=12)
        self.assertAlmostEqual(to_si("Vz_Wc_R1", 36.0), 10.0, places=9)

    def test_an_unregistered_unit_is_refused_numerically(self):
        # The sentinel path must still refuse a channel whose unit is not established.
        # No live channel is UNVERIFIED any more, so exercise the guard directly rather
        # than relying on one happening to be unverified.
        with mock.patch.dict(
            units.CHANNEL_UNITS, {"Probe_Only": units.UNVERIFIED}, clear=False
        ):
            for call in (
                lambda: require_verified("Probe_Only"),
                lambda: si_factor("Probe_Only"),
                lambda: to_si("Probe_Only", 1.0),
                lambda: si_vector({"Probe_Only": 1.0}),
            ):
                with self.assertRaises(UnverifiedUnitError):
                    call()

    def test_unknown_channel_reports_how_to_register_it(self):
        with self.assertRaises(KeyError) as context:
            unit_of("NotAChannel")
        self.assertIn("verify_channel_units", str(context.exception))

    def test_si_vector_converts_every_requested_channel(self):
        converted = si_vector({"Vx": 3.6, "AVy_L1": 60.0, "CmpS_L1": 100.0})
        self.assertAlmostEqual(converted["Vx"], 1.0, places=12)
        self.assertAlmostEqual(converted["AVy_L1"], 2.0 * math.pi, places=12)
        self.assertAlmostEqual(converted["CmpS_L1"], 0.1, places=12)

    def test_contract_table_marks_verified_and_unverified_channels(self):
        table = contract_table()
        self.assertEqual(table["Vx"]["status"], "measured")
        self.assertEqual(table["Vx"]["si_unit"], "m/s")
        self.assertEqual(table["Vz_Wc_L1"]["status"], "measured")

    def test_control_order_matches_the_trucksim_import_order(self):
        self.assertEqual(
            CONTROL_ORDER,
            ("T_FL", "T_FR", "T_RL", "T_RR", "F_FL", "F_FR", "F_RL", "F_RR"),
        )
        self.assertEqual(
            [CONTROL_TO_TRUCKSIM[name] for name in CONTROL_ORDER],
            ["IMP_MYUSM_L1", "IMP_MYUSM_R1", "IMP_MYUSM_L2", "IMP_MYUSM_R2",
             "IMP_FS_L1", "IMP_FS_R1", "IMP_FS_L2", "IMP_FS_R2"],
        )
        self.assertEqual(wheel_channel("Fz", "FR"), "Fz_R1")
        self.assertEqual(wheel_channel("CmpS", "RL"), "CmpS_L2")

    def test_every_contract_channel_has_a_known_unit_or_is_flagged(self):
        for channel, unit in CHANNEL_UNITS.items():
            if unit == UNVERIFIED:
                continue
            self.assertTrue(si_name(channel), "no SI name for %s" % channel)


if __name__ == "__main__":
    unittest.main()
