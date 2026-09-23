import tempfile
import unittest
from pathlib import Path

from ddevsim.hd_ddev_case import (
    ACTIVE_FORCE_IMPORTS,
    DDEV_EXPORTS,
    TORQUE_IMPORTS,
    build_hd_ddev_case,
    describe_suspension_architecture,
    detect_vehicle_code,
    extend_steer_jounce_stop,
    scale_payload,
    parse_protected_parameters,
    transform_hd_ddev_run,
)


MINIMAL_SOURCE = """PARSFILE
VEHICLE_CODE s_s
TSTOP 15
TSTEP 0.0005
OPT_PT 3
M_SU 6500
IXX_SU 4200
M_PL 2000
L_AXLE 0
L_TRACK 1975
M_US 200
L_AXLE 3900
L_TRACK 1975
M_US 200
OPT_PT 3
END
"""


class HDDdevCaseTests(unittest.TestCase):
    def test_transform_disables_powertrain_and_appends_ordered_eight_inputs(self):
        transformed = transform_hd_ddev_run(MINIMAL_SOURCE, stop_s=1.0)

        self.assertNotIn("OPT_PT 3", transformed)
        self.assertEqual(transformed.count("OPT_PT 0"), 2)
        expected_imports = TORQUE_IMPORTS + ACTIVE_FORCE_IMPORTS
        import_lines = [line for line in transformed.splitlines() if line.startswith("IMPORT ")]
        self.assertEqual(import_lines, list(expected_imports))
        export_lines = [line for line in transformed.splitlines() if line.startswith("EXPORT ")]
        self.assertEqual(export_lines, list(DDEV_EXPORTS))
        self.assertLess(transformed.rfind("EXPORT "), transformed.rfind("\nEND"))

    def test_transform_preserves_vehicle_parameters_byte_for_byte(self):
        before = parse_protected_parameters(MINIMAL_SOURCE)
        after = parse_protected_parameters(transform_hd_ddev_run(MINIMAL_SOURCE, stop_s=1.0))
        self.assertEqual(after, before)

    def test_transform_is_deterministic_and_enforces_a_disabled_powertrain(self):
        first = transform_hd_ddev_run(MINIMAL_SOURCE, stop_s=1.0)
        second = transform_hd_ddev_run(MINIMAL_SOURCE, stop_s=1.0)
        self.assertEqual(second, first)

        # The invariant is that no *active* powertrain option survives, not that the
        # source had a particular number of them: a solid-axle case ships OPT_PT 3
        # once per unit, while a corner-module case is already OPT_PT 0.
        self.assertNotIn("OPT_PT 3", first)
        single = transform_hd_ddev_run(MINIMAL_SOURCE.replace("OPT_PT 3\n", "", 1), stop_s=1.0)
        self.assertNotIn("OPT_PT 3", single)
        already_disabled = transform_hd_ddev_run(
            MINIMAL_SOURCE.replace("OPT_PT 3", "OPT_PT 0"), stop_s=1.0
        )
        self.assertNotIn("OPT_PT 3", already_disabled)

        # A source declaring neither option cannot be shown to be a DDEV at all.
        with self.assertRaisesRegex(ValueError, "OPT_PT"):
            transform_hd_ddev_run(MINIMAL_SOURCE.replace("OPT_PT 3\n", ""), stop_s=1.0)

    def test_every_tstop_is_set_because_the_last_one_is_the_run_control(self):
        # A merged file can carry one TSTOP per unit block and the solver honours the
        # last; replacing only the first silently left a 20 s integration.
        two_units = MINIMAL_SOURCE.replace("TSTOP 15", "TSTOP 20\nTSTOP 20")
        transformed = transform_hd_ddev_run(two_units, stop_s=1.0)
        self.assertNotIn("TSTOP 20", transformed)
        self.assertEqual(transformed.count("TSTOP 1"), 2)
        with self.assertRaisesRegex(ValueError, "TSTOP"):
            transform_hd_ddev_run(MINIMAL_SOURCE.replace("TSTOP 15\n", ""), stop_s=1.0)

    def test_detects_vehicle_code_and_suspension_architecture(self):
        self.assertEqual(detect_vehicle_code(MINIMAL_SOURCE), "S_S")
        # a lead unit towing a solid-axle trailer still reports the lead unit's code
        self.assertEqual(detect_vehicle_code("VEHICLE_CODE i_i__s\nEND\n"), "I_I")
        self.assertEqual(detect_vehicle_code("VEHICLE_CODE i_i\nEND\n"), "I_I")
        with self.assertRaises(ValueError):
            detect_vehicle_code("PARSFILE\nEND\n")

        corner_module = (
            "VEHICLE_CODE i_i\n"
            "#FullDataName Suspension: Independent System Kinematics`Steer Axle`x\n"
            "#FullDataName Suspension: Independent System Kinematics`Drive Axle`x\n"
            "#FullDataName Suspension: Independent Compliance, Springs, and Dampers`A`x\n"
            "#FullDataName Suspension: Independent Compliance, Springs, and Dampers`B`x\n"
        )
        arch = describe_suspension_architecture(corner_module)
        self.assertTrue(arch["corners_mechanically_independent"])
        self.assertEqual(arch["front_axle"], "independent")
        self.assertEqual(arch["rear_axle"], "independent")
        self.assertEqual(len(arch["independent_kinematics_datasets"]), 2)

        arch_solid = describe_suspension_architecture(MINIMAL_SOURCE)
        self.assertFalse(arch_solid["corners_mechanically_independent"])
        self.assertEqual(arch_solid["front_axle"], "solid")

    def test_builder_writes_eight_by_sixteen_solver_contract(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.par"
            source.write_text(MINIMAL_SOURCE, encoding="utf-8")
            artifacts = build_hd_ddev_case(
                source,
                root / "model",
                program_dir=Path(r"F:\TruckSim2019\TruckSim2019.0_Prog"),
                data_dir=Path(r"F:\TruckSim2019\TruckSim2019.0_Data"),
                stop_s=1.0,
            )
            simfile = artifacts["simfile"].read_text(encoding="ascii")
            self.assertIn("VEHICLE_CODE S_S", simfile)
            self.assertIn("PORTS_IMP 8", simfile)
            self.assertIn("PORTS_EXP 96", simfile)
            self.assertTrue(artifacts["manifest"].is_file())

    def test_extend_jounce_stop_moves_every_low_onset_table(self):
        # The corner-module dataset ships a front jounce stop ending at 61 mm while the
        # vehicle's static ride position is 80.03 mm, so the stop force is extrapolated
        # at 7000 N/mm to ~140 kN per corner -- about ten times the vehicle weight --
        # at t=0. Extending the travel is what stops the model ringing at rest.
        source = (
            "F_JNC_STOP_TABLE LINEAR\n50, 0\n60, 0\n61, 7000\nENDTABLE\n"
            "F_JNC_STOP_TABLE LINEAR\n50, 0\n60, 0\n61, 7000\nENDTABLE\n"
            "F_JNC_STOP_TABLE LINEAR\n60, 0\n100, 0\n101, 7000\nENDTABLE\n"
            # already beyond the target: must be left untouched
            "F_JNC_STOP_TABLE LINEAR\n190, 0\n200, 0\n201, 7000\nENDTABLE\n"
        )
        extended = extend_steer_jounce_stop(source, 121.0)
        # every stop whose onset is below the target is moved exactly once
        self.assertEqual(extended.count("111, 0"), 3)
        self.assertEqual(extended.count("121, 7000"), 3)
        self.assertNotIn("61, 7000", extended)
        self.assertIn("201, 7000", extended)
        with self.assertRaises(ValueError):
            extend_steer_jounce_stop("PARSFILE\nEND\n", 121.0)

    def test_scale_payload_multiplies_every_instance(self):
        scaled = scale_payload("M_PL 200\nM_PL(2) 200\nM_PL(3) 200\n", 0.5)
        self.assertIn("M_PL 100", scaled)
        self.assertIn("M_PL(2) 100", scaled)
        self.assertIn("M_PL(3) 100", scaled)
        with self.assertRaises(ValueError):
            scale_payload("PARSFILE\nEND\n", 0.5)

    def test_builder_takes_the_vehicle_code_from_the_source(self):
        # The corner-module control object is I_I; the simfile must say so or the
        # solver loads the wrong structure.
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.par"
            source.write_text(
                MINIMAL_SOURCE.replace("VEHICLE_CODE s_s", "VEHICLE_CODE i_i"), encoding="utf-8"
            )
            artifacts = build_hd_ddev_case(
                source,
                root / "model",
                program_dir=Path(r"F:\TruckSim2019\TruckSim2019.0_Prog"),
                data_dir=Path(r"F:\TruckSim2019\TruckSim2019.0_Data"),
                stop_s=1.0,
            )
            simfile = artifacts["simfile"].read_text(encoding="ascii")
            self.assertIn("VEHICLE_CODE I_I", simfile)
            self.assertNotIn("VEHICLE_CODE S_S", simfile)


if __name__ == "__main__":
    unittest.main()
