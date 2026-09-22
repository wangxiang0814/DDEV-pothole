import tempfile
import unittest
from pathlib import Path

from ddevsim.hd_ddev_case import (
    ACTIVE_FORCE_IMPORTS,
    DDEV_EXPORTS,
    TORQUE_IMPORTS,
    build_hd_ddev_case,
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

    def test_transform_is_deterministic_and_rejects_unexpected_powertrain_count(self):
        first = transform_hd_ddev_run(MINIMAL_SOURCE, stop_s=1.0)
        second = transform_hd_ddev_run(MINIMAL_SOURCE, stop_s=1.0)
        self.assertEqual(second, first)
        with self.assertRaisesRegex(ValueError, "OPT_PT 3"):
            transform_hd_ddev_run(MINIMAL_SOURCE.replace("OPT_PT 3\n", "", 1), stop_s=1.0)

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
            self.assertIn("PORTS_EXP 16", simfile)
            self.assertTrue(artifacts["manifest"].is_file())


if __name__ == "__main__":
    unittest.main()
