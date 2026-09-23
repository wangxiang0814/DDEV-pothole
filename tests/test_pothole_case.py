import re
import tempfile
import unittest
from pathlib import Path

from ddevsim.hd_ddev_case import parse_protected_parameters
from ddevsim.pothole_case import (
    PotholeScenario,
    SCENARIO_EXPORTS,
    build_single_wheel_pothole_case,
    corner_module_scenario,
    transform_single_wheel_pothole,
)
from ddevsim.visual_mesh import build_visual_mesh_text


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
    def test_explicit_visual_mesh_contains_ground_road_and_fine_pit_geometry(self):
        obj, mtl = build_visual_mesh_text(PotholeScenario())
        self.assertIn("usemtl DDEV_Grass", obj)
        self.assertIn("usemtl DDEV_Road", obj)
        self.assertIn("usemtl DDEV_Pit", obj)
        self.assertIn("newmtl DDEV_Grass", mtl)
        self.assertIn("newmtl DDEV_Road", mtl)
        self.assertIn("newmtl DDEV_Pit", mtl)
        # The explicit mesh must carry UVs and use TruckSim's shipped natural
        # textures; a Kd-only material renders as the flat colour we are replacing.
        self.assertIn("\nvt ", obj)
        self.assertRegex(obj, r"(?m)^f \d+/\d+ \d+/\d+ \d+/\d+ \d+/\d+$")
        self.assertIn("Road_Surfaces/Asphalt_Fine_di.dds", mtl)
        self.assertIn("Grass/grass_dark_di.dds", mtl)
        self.assertIn("Dirt/Dirt_Ground_B_di.dds", mtl)
        self.assertIn("Sky_Boxes/Partly_Cloudy_Sky/Sky_Partly_Cloudy_di.dds", mtl)
        self.assertNotIn("roadsurface_rough_di.dds", mtl)
        # Sky faces are close enough for the TruckSim 2019 far clip.  The first two
        # vertices are angle-1 then angle-0, i.e. inward rather than outward winding.
        first_vertices = [
            tuple(float(v) for v in line.split()[1:])
            for line in obj.splitlines() if line.startswith("v ")
        ][:2]
        self.assertLess(first_vertices[0][0], first_vertices[1][0])
        # The pit is actual depressed geometry with a 50 mm grid, not a flat decal.
        self.assertGreater(obj.count("usemtl DDEV_Pit"), 80)
        expected_floor = PotholeScenario().road_visual_offset_m - PotholeScenario().depth_m
        self.assertIn(" %.6f" % expected_floor, obj)

    def test_transform_builds_right_track_pit_with_matching_physics_and_visuals(self):
        scenario = PotholeScenario()
        transformed = transform_single_wheel_pothole(SOURCE, scenario)

        self.assertIn("ROAD_DZ_CARPET 2D_LINEAR", transformed)
        self.assertIn("101.15, 0, 0, -0.45, -0.45, 0, 0", transformed)
        self.assertIn("102.25, 0, 0, -0.45, -0.45, 0, 0", transformed)
        # lane 6 is the hole itself (lane 1 and 7 are off-road ground)
        self.assertIn("LIN(6) -1.3875", transformed)
        self.assertIn("LOUT(6) -0.5875", transformed)
        # A medium earth tint preserves shadows but does not collapse the whole
        # depression to black, so the entry/exit slopes remain readable.
        self.assertIn("COLOR(6) 0.460 0.380 0.270", transformed)
        self.assertIn("COLOR(2) 0.180 0.200 0.220", transformed)
        # material names must exist in the library the block loads via MTL_FILE,
        # otherwise the surface has no material and does not render
        self.assertIn("MATERIAL(2) Asphalt (Fine)", transformed)
        self.assertIn("MATERIAL(6) Dirt (Light)", transformed)
        self.assertIn("AUTODETAIL(2) Road", transformed)
        self.assertIn("AUTODETAIL(6) Road", transformed)
        self.assertIn("AUTODETAIL(1) Foliage", transformed)
        self.assertIn("LDIV(2) 5", transformed)
        self.assertIn("MTL_FILE Animator/Road_Materials/road.mtl", transformed)
        self.assertIn(
            "add_obj Animator\\3D_Shape_Files\\DDEV\\DDEV_pothole_scene.obj",
            transformed,
        )
        self.assertIn("LINUNITS(2) m", transformed)
        self.assertIn("SINT(2) 0.5", transformed)
        self.assertIn("SINT(6) 0.05", transformed)
        # visual road sits slightly above the horizon bowl, while the hole bottom
        # remains below it; this prevents the bowl from hiding the carriageway.
        self.assertIn("DZ(2) 0.02", transformed)
        self.assertIn("DZ(6) -0.43", transformed)
        # the carriageway must be wide enough to be visible around the vehicle
        self.assertIn("LIN(2) -5", transformed)
        self.assertIn("LOUT(2) 5", transformed)
        self.assertIn("MU_ROAD_CONSTANT 0.7", transformed)

    def test_road_path_covers_every_station_the_scene_uses(self):
        # THE floating-vehicle bug: a surface shape is resolved against the road PATH,
        # and the template declared `SPATH_START 0 / SEGMENT_LENGTH 40` while the
        # vehicle drove at station 100-107 and the shapes were declared at 95-140. The
        # whole scene therefore sat outside the path, no surface geometry was generated
        # there, and the vehicle had no ground under it.
        scenario = PotholeScenario()
        transformed = transform_single_wheel_pothole(SOURCE, scenario)
        path_start = float(
            re.search(r"(?m)^SPATH_START\s+([-+0-9.eE]+)\s*$", transformed).group(1)
        )
        path_length = float(
            re.search(r"(?m)^SEGMENT_LENGTH\s+([-+0-9.eE]+)\s*$", transformed).group(1)
        )
        shape_starts = [float(v) for v in re.findall(r"(?m)^SSTART\(\d+\)\s+([-+0-9.eE]+)\s*$", transformed)]
        shape_stops = [float(v) for v in re.findall(r"(?m)^SSTOP\(\d+\)\s+([-+0-9.eE]+)\s*$", transformed)]
        self.assertTrue(shape_starts and shape_stops)
        self.assertEqual(path_start, 0.0)
        self.assertGreaterEqual(path_length, max(shape_stops))
        # the vehicle starts at road_lead_in before the lip, inside the path
        vehicle_start = scenario.leading_edge_m - scenario.approach_distance_m
        self.assertLessEqual(min(shape_starts), vehicle_start)
        self.assertLessEqual(vehicle_start, path_length)

    def test_off_road_ground_exists_beside_the_carriageway(self):
        # Without it the vehicle drives on a 10 m ribbon with nothing beside it, which
        # also reads as floating.
        transformed = transform_single_wheel_pothole(SOURCE, PotholeScenario())
        self.assertIn("NLANES 7", transformed)
        self.assertIn("LIN(1) -200", transformed)
        self.assertIn("LOUT(1) -5", transformed)
        self.assertIn("LIN(7) 5", transformed)
        self.assertIn("LOUT(7) 200", transformed)
        self.assertIn("MATERIAL(1) Grass (Dark)", transformed)
        self.assertIn("MATERIAL(7) Grass (Dark)", transformed)
        self.assertIn("COLOR(1) 0.250 0.550 0.200", transformed)

    def test_scene_places_visible_cones_before_and_after_the_pothole(self):
        transformed = transform_single_wheel_pothole(SOURCE, PotholeScenario())
        self.assertGreaterEqual(transformed.count("Traffic_Cone_Small.obj"), 4)
        for token in ("DDEV_Cone_Entry_Left", "DDEV_Cone_Entry_Right",
                      "DDEV_Cone_Exit_Left", "DDEV_Cone_Exit_Right"):
            self.assertIn(token, transformed)

    def test_land_bowl_does_not_occlude_the_generated_road_surfaces(self):
        transformed = transform_single_wheel_pothole(SOURCE, PotholeScenario())
        # The old Light Grass land bowl rendered above the Road Surface Shapes, so
        # Visualizer showed one continuous green texture and hid both asphalt and pit.
        self.assertNotIn("Land_Bowl_Light_Grass.obj", transformed)
        self.assertNotIn("DDEV_light_grass_background.par", transformed)
        # Sky is part of the explicit scene mesh; ground and road must not come from
        # a land bowl that can cover the pothole.
        self.assertIn("DDEV_pothole_scene.obj", transformed)

    def test_scene_uses_stock_textured_skybox_without_old_background_layers(self):
        transformed = transform_single_wheel_pothole(SOURCE, PotholeScenario())
        self.assertIn("Sky_Partly_Cloudy.obj", transformed)
        self.assertIn("Partly_Cloudy_Sky\\cubeXPos.tga", transformed)
        self.assertNotIn("add_reference_frame DDEV_Sky", transformed)
        # The stock sky link is retained, and a near explicit textured cylinder is
        # also emitted because API-only histories can omit the database sky object.
        self.assertIn("usemtl DDEV_Sky", build_visual_mesh_text(PotholeScenario())[0])

    def test_transform_sets_clear_camera_and_keeps_ddev_contract_and_parameters(self):
        transformed = transform_single_wheel_pothole(SOURCE, PotholeScenario())
        self.assertIn("SET_AZIMUTH -45", transformed)
        self.assertIn("SET_ELEVATION 16", transformed)
        self.assertIn("SET_DISTANCE 32", transformed)
        # look point at mid-body height, not wheel height, and a lens wide enough
        # to contain the whole vehicle
        self.assertIn("SET_LOOKPOINT_Z 1", transformed)
        self.assertIn("SET_FIELD_OF_VIEW 26", transformed)
        self.assertIn("DDEV_pothole_scene.obj", transformed)
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
            run_all = artifacts["run_all"].read_text(encoding="utf-8")
            simfile = artifacts["simfile"].read_text(encoding="ascii")
            self.assertIn("FILEBASE output\\single_wheel_deep_pothole", simfile)
            self.assertNotIn("hd_utility_ddev", simfile)
            # PORTS_EXP is derived from the EXPORT lines actually present, so that a
            # source which already carries scenario channels cannot desynchronise.
            exports = len([l for l in run_all.splitlines() if l.startswith("EXPORT ")])
            self.assertIn("PORTS_EXP %d" % exports, simfile)
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

    def test_corner_module_pit_is_wider_but_remains_single_right_track(self):
        scenario = corner_module_scenario()
        tire_width_m = 0.175
        left_wheel_center_m = 0.63
        left_tire_inner_edge_m = left_wheel_center_m - tire_width_m / 2.0

        self.assertAlmostEqual(scenario.width_m, 0.90)
        self.assertTrue(scenario.covers_point(101.5, -0.63))
        self.assertFalse(scenario.covers_point(101.5, left_tire_inner_edge_m))
        self.assertGreaterEqual(left_tire_inner_edge_m - scenario.lateral_max_m, 0.70)


if __name__ == "__main__":
    unittest.main()
