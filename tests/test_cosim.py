import unittest

from ddevsim.cosim import signed_pulse


class CosimTests(unittest.TestCase):
    def test_signed_pulse_is_one_corner_only_and_has_half_open_window(self):
        self.assertEqual(signed_pulse(0.1, 2, -1500.0, 0.2, 0.4), (0.0, 0.0, 0.0, 0.0))
        self.assertEqual(signed_pulse(0.2, 2, -1500.0, 0.2, 0.4), (0.0, 0.0, -1500.0, 0.0))
        self.assertEqual(signed_pulse(0.399, 2, -1500.0, 0.2, 0.4), (0.0, 0.0, -1500.0, 0.0))
        self.assertEqual(signed_pulse(0.4, 2, -1500.0, 0.2, 0.4), (0.0, 0.0, 0.0, 0.0))

    def test_bad_corner_is_rejected(self):
        with self.assertRaises(ValueError):
            signed_pulse(0.3, 4, 1000.0, 0.2, 0.4)


if __name__ == "__main__":
    unittest.main()
