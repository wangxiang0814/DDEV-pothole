"""Batch pipeline and dataset-export regression tests."""

import json
import struct
import tempfile
import unittest
from pathlib import Path

from ddevsim.batch import (
    DEFAULT_EXCLUDED_CHANNELS,
    _safe_name,
    observations_in_si,
    parse_solver_log,
    quality_gates,
)
from ddevsim.native_video import (
    describe_history,
    find_history,
    is_ascii_path,
    manual_export_steps,
    resolve_stage_dir,
    write_animator_par,
)

CORNERS = ("FL", "FR", "RL", "RR")
SUFFIX = {"FL": "L1", "FR": "R1", "RL": "L2", "RR": "R2"}


class _FakeVehicle:
    jounce_limit_m = 0.151
    rebound_limit_m = 0.151
    total_weight_n = 87279.0


class _FakeController:
    safe_stop = False
    safe_stop_reason = ""

    def step_summary(self):
        return {"final_step_name": "4 recover to normal driving"}


def _row(**overrides):
    row = {
        "time_s": 0.0,
        "exp_Vx": 2.8, "exp_AVy_L1": 13.4, "exp_CmpS_L1": 53.6,
        "exp_Xo": 100.0, "exp_Roll_E": 0.0, "exp_Pitch": 0.0,
        "exp_Vz_Wc_L1": -0.01,
    }
    for corner in CORNERS:
        suffix = SUFFIX[corner]
        row["exp_Fz_" + suffix] = 21800.0
        row["exp_CmpS_" + suffix] = 53.0
        row["exp_AVy_" + suffix] = 13.4
        row["exp_Vz_Wc_" + suffix] = -0.01
    for name in ("L1", "R1", "L2", "R2"):
        row.setdefault("exp_X_" + name, 100.0)
    row.update(overrides)
    return row


class SafeNameTests(unittest.TestCase):
    def test_case_names_are_made_safe_for_simfile_paths(self):
        self.assertEqual(_safe_name("depth_010"), "depth_010")
        self.assertEqual(_safe_name("baseline"), "baseline")
        self.assertEqual(_safe_name("case-1"), "case_1")
        # a leading digit would be a poor basename; prefix it
        self.assertTrue(_safe_name("1case").startswith("case_"))


class SolverLogTests(unittest.TestCase):
    def test_extrapolation_warnings_are_detected(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "case_log.txt"
            log.write_text(
                "Run started\n"
                "Function FY_TIRE_CARPET(1,1,1) extrapolated (out of range HIGH) for "
                "XCOL = 39255.6 N at T = 1.625\n"
                "Function F_JNC_STOP_TABLE(1,1) extrapolated (out of range LOW) for "
                "X = 2.5 mm at T = 0\n"
                "Run stopped at t = 9. Stop time reached.\n",
                encoding="utf-8",
            )
            report = parse_solver_log(log)
            self.assertTrue(report["log_found"])
            self.assertEqual(report["extrapolation_count"], 2)
            self.assertIn("FY_TIRE_CARPET(1,1,1)", report["extrapolation_functions"])
            self.assertIn("Run stopped at t = 9", report["stopped_reason"])

    def test_missing_log_is_reported_rather_than_crashing(self):
        report = parse_solver_log(Path("does_not_exist_log.txt"))
        self.assertFalse(report["log_found"])
        self.assertEqual(report["extrapolation_count"], 0)


class QualityGateTests(unittest.TestCase):
    def _gates(self, rows, solver=None, log=None, controller=None, tyre=20000.0):
        return quality_gates(
            rows,
            _FakeVehicle(),
            solver or {"status": "COMPLETED"},
            log or {"extrapolation_count": 0, "extrapolation_functions": []},
            controller or _FakeController(),
            tyre,
        )

    def test_a_clean_run_is_usable(self):
        rows = [_row() for _ in range(50)]
        gates = self._gates(rows)
        self.assertTrue(gates["usable"], gates["failures"])
        self.assertEqual(gates["failures"], [])

    def test_tyre_table_extrapolation_makes_the_run_unusable(self):
        rows = [_row() for _ in range(50)]
        gates = self._gates(rows, log={"extrapolation_count": 3, "extrapolation_functions": ["FY_TIRE_CARPET"]})
        self.assertFalse(gates["usable"])
        self.assertIn("tyre_or_suspension_table_extrapolated", gates["failures"])

    def test_excessive_wheel_load_is_rejected_against_the_tyre_reference(self):
        rows = [_row() for _ in range(10)]
        rows[5]["exp_Fz_R1"] = 4.0 * 20000.0
        gates = self._gates(rows)
        self.assertIn("wheel_load_above_3x_tyre_reference", gates["failures"])
        self.assertAlmostEqual(gates["peak_load_over_tyre_reference"], 4.0, places=3)

    def test_suspension_travel_violation_is_detected(self):
        rows = [_row() for _ in range(10)]
        rows[3]["exp_CmpS_R1"] = 249.0  # beyond the +151 mm jounce stop
        gates = self._gates(rows)
        self.assertIn("suspension_travel_exceeded", gates["failures"])

    def test_whole_vehicle_airborne_is_detected(self):
        rows = [_row() for _ in range(10)]
        for corner in CORNERS:
            rows[0]["exp_Fz_" + SUFFIX[corner]] = 0.0
        gates = self._gates(rows)
        self.assertIn("whole_vehicle_airborne", gates["failures"])
        self.assertAlmostEqual(gates["all_wheels_airborne_fraction"], 0.1, places=6)

    def test_controller_safe_stop_and_incomplete_runs_are_flagged(self):
        class _Stopped(_FakeController):
            safe_stop = True
            safe_stop_reason = "roll_exceeds_12.0_deg"

        rows = [_row() for _ in range(10)]
        gates = self._gates(rows, solver={"status": "TERMINATED"}, controller=_Stopped())
        self.assertIn("run_incomplete", gates["failures"])
        self.assertIn("controller_safe_stop:roll_exceeds_12.0_deg", gates["failures"])


class ObservationTests(unittest.TestCase):
    def test_unverified_channels_are_excluded_from_the_observation_vector(self):
        rows = [_row()]
        names, matrix = observations_in_si(rows)
        for excluded in DEFAULT_EXCLUDED_CHANNELS:
            self.assertNotIn(excluded, names)
        self.assertNotIn("Vz_Wc_R1", names)
        self.assertEqual(len(names), len(matrix[0]))

    def test_observations_are_converted_into_si(self):
        rows = [_row()]
        names, matrix = observations_in_si(rows)
        index = names.index("Vx")
        self.assertAlmostEqual(matrix[0][index], 2.8 / 3.6, places=9)
        index = names.index("AVy_L1")
        import math
        self.assertAlmostEqual(matrix[0][index], 13.4 * 2 * math.pi / 60.0, places=9)
        index = names.index("CmpS_L1")
        self.assertAlmostEqual(matrix[0][index], 0.053, places=9)


# ------------------------------------------------------------------ native video
def _write_history(directory: Path, basename: str, channels, samples: int, step: float):
    group = {
        "VsChannelGroup": {
            "XStart": 0.0,
            "XStep": step,
            "XUnits": "s",
            "Channels": [
                {"Name Aliases": [name], "Units": unit, "Generic Name": "g"}
                for name, unit in channels
            ],
        }
    }
    (directory / (basename + ".vs")).write_text(json.dumps(group), encoding="utf-8")
    count = len(channels)
    payload = struct.pack("<6i", 1, 0, 0, 0, 4, count)
    payload += struct.pack("<%df" % (count * samples), *([0.0] * count * samples))
    (directory / (basename + ".vsb")).write_bytes(payload)
    (directory / (basename + "_all.par")).write_text("PARSFILE\nEND\n", encoding="utf-8")


class NativeVideoTests(unittest.TestCase):
    CHANNELS = [("Rot_L1", "rev"), ("Xo", "m"), ("Roll_E", "deg")]

    def test_history_is_discovered_from_a_directory_and_validated(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _write_history(root, "case_a", self.CHANNELS, samples=40, step=0.025)
            history = find_history(root)
            self.assertEqual(history.basename, "case_a")
            self.assertEqual(history.samples, 40)
            self.assertEqual(len(history.channels), 3)
            self.assertEqual(history.units["Roll_E"], "deg")
            self.assertAlmostEqual(history.duration_s, 0.975, places=9)

    def test_missing_vsb_is_reported_as_an_incomplete_history(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _write_history(root, "case_b", self.CHANNELS, samples=10, step=0.1)
            (root / "case_b.vsb").unlink()
            with self.assertRaises(FileNotFoundError):
                find_history(root)

    def test_ambiguous_directory_lists_the_candidate_histories(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _write_history(root, "one", self.CHANNELS, samples=5, step=0.1)
            _write_history(root, "two", self.CHANNELS, samples=5, step=0.1)
            with self.assertRaises(ValueError) as context:
                find_history(root)
            self.assertIn("one", str(context.exception))
            self.assertIn("two", str(context.exception))

    def test_mismatched_channel_count_between_vs_and_vsb_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _write_history(root, "case_c", self.CHANNELS, samples=8, step=0.1)
            # corrupt the declared channel count in the binary header
            raw = bytearray((root / "case_c.vsb").read_bytes())
            struct.pack_into("<i", raw, 20, 7)
            (root / "case_c.vsb").write_bytes(bytes(raw))
            with self.assertRaises(ValueError):
                find_history(root)

    def test_animator_par_references_the_history_and_parsfle(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _write_history(root, "case_d", self.CHANNELS, samples=8, step=0.1)
            history = find_history(root)
            par = write_animator_par(root / "stage", history)
            text = par.read_text(encoding="ascii")
            self.assertIn("SET_RUN_SLOT 0", text)
            self.assertIn(str(history.vs), text)
            self.assertIn(str(history.par), text)

    def test_manual_steps_state_that_the_avi_writer_is_gui_only(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _write_history(root, "case_e", self.CHANNELS, samples=400, step=0.025)
            history = find_history(root)
            steps = "\n".join(manual_export_steps(history, root / "stage"))
            self.assertIn("Export Video", steps)
            self.assertIn("compressor", steps)
            self.assertIn("9.975", steps)  # the actual duration must be quoted

    def test_non_ascii_stage_paths_are_relocated(self):
        self.assertTrue(is_ascii_path(Path(r"C:\temp\stage")))
        self.assertFalse(is_ascii_path(Path(r"F:\1tongji\分布式电驱\stage")))
        resolved, relocated = resolve_stage_dir(Path(r"F:\1tongji\分布式电驱\stage"))
        self.assertTrue(relocated)
        self.assertTrue(is_ascii_path(resolved))
        keep, moved = resolve_stage_dir(Path(r"C:\temp\stage"))
        self.assertFalse(moved)
        self.assertEqual(keep, Path(r"C:\temp\stage"))

    def test_ascii_check_uses_the_absolute_path(self):
        # A relative path can look ASCII while its absolute form is not, and it is
        # the absolute form that reaches VS Visualizer on the command line. Missing
        # this let the export hand it a mangled path it then failed to open.
        relative = Path("runs") / "batch_x" / "native_video"
        self.assertEqual(str(relative).encode("ascii").decode("ascii"), str(relative))
        self.assertEqual(is_ascii_path(relative), is_ascii_path(relative.resolve()))

    def test_animator_par_uses_absolute_paths(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _write_history(root, "case_f", self.CHANNELS, samples=8, step=0.1)
            history = find_history(root)
            self.assertTrue(history.vs.is_absolute())
            self.assertTrue(history.par.is_absolute())
            par = write_animator_par(root / "stage", history)
            for line in par.read_text(encoding="mbcs").splitlines():
                if line.startswith(("DATASET", "PARSFILE ")) and "animator" not in line:
                    self.assertTrue(Path(line.split(None, 1)[1]).is_absolute(), line)

    def test_launch_flags_are_not_mutually_exclusive(self):
        # CREATE_NEW_CONSOLE (0x10) and DETACHED_PROCESS (0x08) cannot be combined;
        # doing so made CreateProcess fail with WinError 87 and no window appeared.
        source = (
            Path(__file__).resolve().parents[1] / "src" / "ddevsim" / "native_video.py"
        ).read_text(encoding="utf-8")
        self.assertIn("creation = 0x00000008", source)
        self.assertNotIn("0x00000010 | 0x00000008", source)

    def test_workdir_is_the_tree_that_holds_the_3d_shape_assets(self):
        # The parsfle uses relative asset paths, so the working directory must
        # contain Animator\3D_Shape_Files. Verified: launching from _Data gives an
        # "Unable to load asset file" warning per truck part; Prog\Resources gives 0.
        from ddevsim.native_video import DEFAULT_RESOURCE_ROOT, default_workdir

        self.assertEqual(default_workdir(), DEFAULT_RESOURCE_ROOT)
        self.assertIn("Resources", str(DEFAULT_RESOURCE_ROOT))


if __name__ == "__main__":
    unittest.main()
