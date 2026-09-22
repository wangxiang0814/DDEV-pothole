import unittest

from ddevsim.interface_validation import (
    ACTIVE_AMPLITUDE_N,
    IMPORT_NAMES,
    TORQUE_AMPLITUDE_NM,
    build_validation_cases,
    command_for_case,
    signed_peak_delta,
)


class InterfaceValidationTests(unittest.TestCase):
    def test_matrix_has_baseline_and_positive_negative_case_for_every_port(self):
        cases = build_validation_cases()
        self.assertEqual(len(cases), 17)
        self.assertEqual(cases[0]["name"], "baseline")
        self.assertEqual([case["port_index"] for case in cases[1:]], [index for index in range(8) for _ in range(2)])
        self.assertEqual([case["polarity"] for case in cases[1:]], [value for _ in range(8) for value in (1, -1)])

    def test_each_pulse_writes_only_its_target_port_in_half_open_window(self):
        for case in build_validation_cases()[1:]:
            command = command_for_case(case, start_s=0.25, stop_s=0.35)
            before = command(0.249, ())
            during = command(0.25, ())
            after = command(0.35, ())
            self.assertEqual(before, (0.0,) * 8)
            self.assertEqual(after, (0.0,) * 8)
            nonzero = [index for index, value in enumerate(during) if value != 0.0]
            self.assertEqual(nonzero, [case["port_index"]])
            magnitude = TORQUE_AMPLITUDE_NM if case["port_index"] < 4 else ACTIVE_AMPLITUDE_N
            self.assertEqual(during[case["port_index"]], case["polarity"] * magnitude)

    def test_import_order_is_four_torques_then_four_active_forces(self):
        self.assertEqual(
            IMPORT_NAMES,
            (
                "IMP_MYUSM_L1", "IMP_MYUSM_R1", "IMP_MYUSM_L2", "IMP_MYUSM_R2",
                "IMP_FS_L1", "IMP_FS_R1", "IMP_FS_L2", "IMP_FS_R2",
            ),
        )

    def test_signed_peak_delta_preserves_the_direction_of_largest_response(self):
        self.assertEqual(signed_peak_delta([10.0, 10.0, 10.0], [10.2, 7.0, 12.0]), -3.0)
        self.assertEqual(signed_peak_delta([0.0, 0.0], [0.25, -0.2]), 0.25)


if __name__ == "__main__":
    unittest.main()
