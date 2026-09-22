import unittest

from ddevsim.pothole_controller import WheelLiftController


class PotholeControllerTests(unittest.TestCase):
    def setUp(self):
        self.controller = WheelLiftController()
        self.base_exports = (0.0, 0.0, 0.0, 0.0, 22000.0, 22000.0, 22000.0, 22000.0) + (0.0,) * 8

    def exports(self, xo, vx):
        return self.base_exports + (xo, vx, 0.0, 0.0)

    def test_front_right_window_unloads_front_right_and_uses_other_three_as_supports(self):
        command = self.controller(2.0, self.exports(101.5, 2.7))
        self.assertEqual(len(command), 8)
        self.assertEqual(command[1], 0.0)
        self.assertTrue(all(command[index] > 0.0 for index in (0, 2, 3)))
        self.assertLess(command[5], 0.0)
        self.assertGreater(command[4], 0.0)
        self.assertGreater(command[6], 0.0)
        self.assertGreater(command[7], 0.0)
        self.assertAlmostEqual(command[5], -17425.0)
        self.assertAlmostEqual(command[4], 17425.0 / 3.0)
        self.assertAlmostEqual(command[6], 17425.0 / 3.0)
        self.assertAlmostEqual(command[7], 17425.0 / 3.0)

    def test_rear_right_window_switches_target_without_cross_wiring(self):
        command = self.controller(6.0, self.exports(105.5, 2.7))
        self.assertEqual(command[3], 0.0)
        self.assertTrue(all(command[index] > 0.0 for index in (0, 1, 2)))
        self.assertLess(command[7], 0.0)
        self.assertGreater(command[4], 0.0)
        self.assertGreater(command[5], 0.0)
        self.assertGreater(command[6], 0.0)

    def test_commands_respect_initial_safe_limits(self):
        for time_s, xo, vx in ((0.0, 100.0, 0.0), (2.0, 101.5, 2.7), (4.0, 103.0, 2.8), (6.0, 105.5, 2.7), (8.0, 107.0, 5.0)):
            command = self.controller(time_s, self.exports(xo, vx))
            self.assertTrue(all(-400.0 <= value <= 700.0 for value in command[:4]))
            self.assertTrue(all(-20000.0 <= value <= 20000.0 for value in command[4:]))

    def test_speed_feedback_commands_regenerative_braking_above_crawl_speed(self):
        command = self.controller(8.0, self.exports(107.0, 5.0))
        self.assertTrue(all(value < 0.0 for value in command[:4]))


if __name__ == "__main__":
    unittest.main()
