import tempfile
import unittest
from pathlib import Path

from ddevsim.hd_ddev_case import parse_protected_parameters
from ddevsim.pothole_case import (
    PotholeScenario,
    SCENARIO_EXPORTS,
    build_single_wheel_pothole_case,
    transform_single_wheel_pothole,
)


SOURCE = """PARSFILE
SET_AZIMUTH 150
SET_ELEVATION 4.5
SET_DISTANCE 100
SET_LOOKPOINT_X 0
SET_LOOKPOINT_Z 0.25
SET_FIELD_OF_VIEW 10
M_SU 6500
M_PL 2000
ENTER_PARSFILE Procedures\\old.par
#FullDataName Procedures`River Crossing`Driving
TSTOP 15
SPEED_TARGET_CONSTANT 10
LOG_ENTRY Used Dataset: Procedures; { Driving } River Crossing
EXIT_PARSFILE Procedures\\old.par
IMPORT IMP_MYUSM_L1 Add 0.0! 0
EXPORT AVy_L1
END
"""


class PotholeCaseTests(unittest.TestCase):
    def test_transform_builds_right_track_pit_with_matching_physics_and_visuals(self):
        scenario = PotholeScenario()
        transformed = transform_single_wheel_pothole(SOURCE, scenario)

        self.assertIn("ROAD_DZ_CARPET 2D_LINEAR", transformed)
        self.assertIn("101.15, 0, 0, -0.45, -0.45, 0, 0", transformed)
        self.assertIn("102.25, 0, 0, -0.45, -0.45, 0, 0", transformed)
        self.assertIn("LIN(5) -1.3875", transformed)
        self.assertIn("LOUT(5) -0.5875", transformed)
        # the hole must be near-black and clearly darker than the road
        self.assertIn("COLOR(5) 0.050 0.050 0.050", transformed)
        self.assertIn("COLOR(1) 0.300 0.300 0.330", transformed)
        # a flat built-in material, so the rendered colour is exactly this value
        self.assertIn("MATERIAL(1) No Texture", transformed)
        self.assertIn("MATERIAL(5) No Texture", transformed)
        # the road must be wide enough to be visible around the vehicle
        self.assertIn("LIN(1) -5", transformed)
        self.assertIn("LOUT(1) 5", transformed)
        self.assertIn("MU_ROAD_CONSTANT 0.7", transformed)

    def test_transform_sets_clear_camera_and_keeps_ddev_contract_and_parameters(self):
        transformed = transform_single_wheel_pothole(SOURCE, PotholeScenario())
        self.assertIn("SET_AZIMUTH -45", transformed)
        self.assertIn("SET_ELEVATION 16", transformed)
        self.assertIn("SET_DISTANCE 20", transformed)
        # look point at mid-body height, not wheel height, and a lens wide enough
        # to contain the whole vehicle
        self.assertIn("SET_LOOKPOINT_Z 1", transformed)
        self.assertIn("SET_FIELD_OF_VIEW 26", transformed)
        self.assertIn("Partly Cloudy Sky", transformed)
        self.assertIn("IMPORT IMP_MYUSM_L1 Add 0.0! 0", transformed)
        self.assertIn("EXPORT AVy_L1", transformed)
        self.assertEqual(parse_protected_parameters(transformed), parse_protected_parameters(SOURCE))

    def test_camera_is_a_scenario_parameter_not_a_constant(self):
        scenario = PotholeScenario(
            **{**PotholeScenario().__dict__, "camera_distance_m": 30.0,
               "camera_field_of_view_deg": 40.0, "camera_look_z_m": 1.6}
        )
        transformed = transform_single_wheel_pothole(SOURCE, scenario)
        self.assertIn("SET_DISTANCE 30", transformed)
        self.assertIn("SET_FIELD_OF_VIEW 40", transformed)
        self.assertIn("SET_LOOKPOINT_Z 1.6", transformed)

    def test_optional_camera_directives_may_be_absent(self):
        # A minimal source without look-point / FOV entries must still transform.
        stripped = "\n".join(
            line for line in SOURCE.splitlines()
            if not line.startswith(("SET_LOOKPOINT_", "SET_FIELD_OF_VIEW"))
        )
        transformed = transform_single_wheel_pothole(stripped, PotholeScenario())
        self.assertIn("SET_AZIMUTH -45", transformed)

    def test_builder_gives_pothole_run_its_own_history_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source_run = root / "source.par"
            source_run.write_text(SOURCE, encoding="utf-8")
            source_sim = root / "source.sim"
            source_sim.write_text(
                "SIMFILE\nFILEBASE output\\hd_utility_ddev\nINPUT run_all.par\n"
                "INPUTARCHIVE output\\hd_utility_ddev_all.par\nECHO output\\hd_utility_ddev_echo.par\n"
                "FINAL output\\hd_utility_ddev_end.par\nLOGFILE output\\hd_utility_ddev_log.txt\n"
                "ERDFILE output\\hd_utility_ddev.erd\nPORTS_IMP 8\nPORTS_EXP 16\nEND\n",
                encoding="ascii",
            )
            artifacts = build_single_wheel_pothole_case(source_run, source_sim, root / "case")
            simfile = artifacts["simfile"].read_text(encoding="ascii")
            self.assertIn("FILEBASE output\\single_wheel_deep_pothole", simfile)
            self.assertNotIn("hd_utility_ddev", simfile)
            self.assertIn("PORTS_EXP %d" % (16 + len(SCENARIO_EXPORTS)), simfile)
            run_all = artifacts["run_all"].read_text(encoding="utf-8")
            for name in SCENARIO_EXPORTS:
                self.assertIn("EXPORT " + name, run_all)
            self.assertTrue(artifacts["run_all"].is_file())

    def test_history_name_gives_each_batch_case_its_own_output_paths(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source_run = root / "source.par"
            source_run.write_text(SOURCE, encoding="utf-8")
            source_sim = root / "source.sim"
            source_sim.write_text(
                "SIMFILE\nFILEBASE output\\hd_utility_ddev\nINPUT run_all.par\n"
                "INPUTARCHIVE output\\hd_utility_ddev_all.par\nECHO output\\hd_utility_ddev_echo.par\n"
                "FINAL output\\hd_utility_ddev_end.par\nLOGFILE output\\hd_utility_ddev_log.txt\n"
                "ERDFILE output\\hd_utility_ddev.erd\nPORTS_IMP 8\nPORTS_EXP 16\nEND\n",
                encoding="ascii",
            )
            artifacts = build_single_wheel_pothole_case(
                source_run, source_sim, root / "case", history_name="depth_010"
            )
            simfile = artifacts["simfile"].read_text(encoding="ascii")
            self.assertIn("FILEBASE output\\depth_010", simfile)
            self.assertIn("depth_010_all.par", simfile)
            self.assertNotIn("hd_utility_ddev", simfile)

    def test_scenario_exposes_pothole_geometry_for_the_controller(self):
        scenario = PotholeScenario()
        self.assertAlmostEqual(scenario.leading_edge_m, 101.1)
        self.assertAlmostEqual(scenario.trailing_edge_m, 102.3)
        self.assertTrue(scenario.covers_station(101.5))
        self.assertFalse(scenario.covers_station(100.9))
        self.assertEqual(scenario.station_span(), (101.1, 102.3))
        # the right-track hole must really sit under the right wheel only
        self.assertTrue(scenario.covers_point(101.5, -0.9875))
        self.assertFalse(scenario.covers_point(101.5, 0.9875))


if __name__ == "__main__":
    unittest.main()
