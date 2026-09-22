import tempfile
import unittest
from pathlib import Path

from ddevsim.solver_api import parse_simfile, probe_solver, run_native, solver_input_name


class SolverApiTests(unittest.TestCase):
    def test_solver_receives_ascii_basename_after_working_directory_change(self):
        path = Path(r"F:\研究\TruckSim仿真\simfile.sim")
        self.assertEqual(solver_input_name(path), b"simfile.sim")

    def test_parse_simfile_resolves_declared_dll(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "case.sim"
            path.write_text(
                "SIMFILE\nDLLFILE C:\\Vendor\\trucksim_64.dll\nPRODUCT_ID TruckSim\nEND\n",
                encoding="utf-8",
            )
            parsed = parse_simfile(path)
            self.assertEqual(parsed["PRODUCT_ID"], "TruckSim")
            self.assertEqual(parsed["DLLFILE"], "C:\\Vendor\\trucksim_64.dll")

    def test_missing_simfile_returns_machine_readable_block(self):
        result = probe_solver(Path("missing.sim"))
        self.assertEqual(result["status"], "BLOCKED")
        self.assertEqual(result["reason_code"], "SIMFILE_NOT_FOUND")

        run_result = run_native(Path("missing.sim"))
        self.assertEqual(run_result["status"], "BLOCKED")
        self.assertEqual(run_result["reason_code"], "SIMFILE_NOT_FOUND")


if __name__ == "__main__":
    unittest.main()
