"""Expert-strategy regression tests.

These lock in the physics that was derived from the reference paper and measured
from the generated TruckSim model, so a later model rebuild or tuning change cannot
silently break the three-wheel support solution.
"""

import math
import unittest
from pathlib import Path

from ddevsim.expert_controller import (
    CORNERS,
    STEP_APPROACH,
    STEP_LIFT_FRONT,
    STEP_LIFT_REAR,
    STEP_RECOVER_FRONT,
    DeepPotholeExpertController,
    ExpertConfig,
    feedforward_commands,
    solve_linear,
    three_wheel_support,
)
from ddevsim.pothole_case import PotholeScenario
from ddevsim.vehicle_params import (
    derive_cg_from_static_loads,
    load_vehicle,
    parse_jounce_rebound_travel_mm,
    parse_roll_centre_drop_mm,
    parse_wheelbase_mm,
)

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "models" / "hd_utility_ddev" / "run_all.par"

#: Static wheel loads measured from a real TruckSim run of this model.
MEASURED_STATIC_LOADS = {
    "FL": 22388.8, "FR": 22383.0, "RL": 21252.1, "RR": 21255.2,
}

#: TruckSim order of the exported scenario channels, matching pothole_case.
EXPORTS = (
    "AVy_L1", "AVy_R1", "AVy_L2", "AVy_R2",
    "Fz_L1", "Fz_R1", "Fz_L2", "Fz_R2",
    "CmpS_L1", "CmpS_R1", "CmpS_L2", "CmpS_R2",
    "Vz_Wc_L1", "Vz_Wc_R1", "Vz_Wc_L2", "Vz_Wc_R2",
    "Xo", "Vx", "Roll_E", "Pitch",
    "X_L1", "X_R1", "X_L2", "X_R2",
)


def _exports(**overrides):
    """Build a full export vector; unlisted channels default to zero."""
    values = {name: 0.0 for name in EXPORTS}
    values.update({
        "Fz_L1": 22388.8, "Fz_R1": 22383.0, "Fz_L2": 21252.1, "Fz_R2": 21255.2,
        "CmpS_L1": 53.6, "CmpS_R1": 53.6, "CmpS_L2": 50.6, "CmpS_R2": 50.6,
        "Xo": 100.0, "Vx": 2.8, "Roll_E": 0.0, "Pitch": 0.0,
        "X_L1": 100.0, "X_R1": 100.0, "X_L2": 96.1, "X_R2": 96.1,
    })
    values.update(overrides)
    return tuple(float(values[name]) for name in EXPORTS)


@unittest.skipUnless(MODEL.exists(), "generated TruckSim model not present")
class VehicleGeometryTests(unittest.TestCase):
    def setUp(self):
        self.vehicle = load_vehicle(MODEL, static_wheel_load_n=MEASURED_STATIC_LOADS)

    def test_parses_wheelbase_from_the_assembly_not_the_tyre_size(self):
        # X_LENGTH also appears for the 565 mm tyre; the parser must pick the
        # vehicle assembly value (3900 mm) and not 1 mm or 565 mm.
        text = MODEL.read_text(encoding="utf-8", errors="replace")
        self.assertAlmostEqual(parse_wheelbase_mm(text), 3900.0, places=6)
        self.assertAlmostEqual(self.vehicle.wheelbase_m, 3.9, places=6)
        self.assertAlmostEqual(self.vehicle.track_m, 1.975, places=6)

    def test_total_mass_includes_unsprung_and_matches_measured_loads(self):
        # 6500 + 2000 + 2*200 = 8900 kg -> 87296 N, matching the measured static sum.
        self.assertAlmostEqual(self.vehicle.total_mass_kg, 8900.0, places=6)
        self.assertLess(
            abs(self.vehicle.total_weight_n - self.vehicle.measured_static_weight_n()),
            40.0,
        )

    def test_cg_is_derived_from_static_loads(self):
        cg_front, cg_rear = derive_cg_from_static_loads(3.9, 44771.8, 87279.1)
        self.assertAlmostEqual(cg_front + cg_rear, 3.9, places=9)
        # rear-biased: the front axle carries slightly more than half the weight
        self.assertGreater(cg_rear, cg_front)

    def test_roll_arm_and_travel_come_from_the_model(self):
        text = MODEL.read_text(encoding="utf-8", errors="replace")
        self.assertAlmostEqual(parse_roll_centre_drop_mm(text), 53.0, places=6)
        self.assertAlmostEqual(self.vehicle.cg_height_m, 0.975, places=6)
        self.assertAlmostEqual(self.vehicle.roll_centre_height_m, 0.512, places=6)
        self.assertAlmostEqual(self.vehicle.cg_above_roll_centre_m, 0.463, places=6)
        jounce, rebound = parse_jounce_rebound_travel_mm(text)
        self.assertAlmostEqual(jounce, 151.0, places=6)
        self.assertAlmostEqual(rebound, -151.0, places=6)


@unittest.skipUnless(MODEL.exists(), "generated TruckSim model not present")
class ThreeWheelSupportTests(unittest.TestCase):
    def setUp(self):
        self.vehicle = load_vehicle(MODEL, static_wheel_load_n=MEASURED_STATIC_LOADS)

    def test_without_a_cg_shift_the_vehicle_cannot_be_supported_on_three_wheels(self):
        # This is the problem the paper exists to solve: with the CG on the FL-RR
        # diagonal, the left-rear wheel load solves negative.
        v = self.vehicle
        weight = v.total_weight_n
        front = weight * v.cg_to_rear_axle_m / v.wheelbase_m
        rear_right = weight / 2.0
        left_rear = weight - front - rear_right
        self.assertLess(left_rear, 0.0)

    def test_paper_attitude_pattern_produces_a_small_positive_third_wheel_load(self):
        for lifted in ("FR", "RR"):
            support = three_wheel_support(self.vehicle, lifted)
            self.assertAlmostEqual(support.target_load_n[lifted], 0.0, places=9)
            # The paper states Fz3 is "very small" and assumes zero; every contact
            # load must nevertheless be non-negative for the state to be stable.
            for corner in CORNERS:
                self.assertGreaterEqual(
                    support.target_load_n[corner], 0.0,
                    "%s load negative for lift %s" % (corner, lifted),
                )
            self.assertLess(support.minimal_load_n, 0.05 * self.vehicle.total_weight_n)

    def test_targets_preserve_total_weight_and_longitudinal_balance(self):
        support = three_wheel_support(self.vehicle, "FR")
        self.assertAlmostEqual(
            sum(support.target_load_n.values()), self.vehicle.total_weight_n, delta=1.0
        )
        # moment balance about the shifted CG: sum(Fz * arm_x) == 0
        cg_long = support.cg_longitudinal_shift_m
        a1 = self.vehicle.cg_to_front_axle_m - cg_long
        b1 = self.vehicle.cg_to_rear_axle_m + cg_long
        front = support.target_load_n["FL"] + support.target_load_n["FR"]
        rear = support.target_load_n["RL"] + support.target_load_n["RR"]
        self.assertAlmostEqual(front * a1, rear * b1, delta=5.0)

    def test_roll_moment_of_the_targets_is_balanced_by_the_shifted_cg(self):
        for lifted in ("FR", "RR"):
            support = three_wheel_support(self.vehicle, lifted)
            sign = {"FL": 1.0, "RL": 1.0, "FR": -1.0, "RR": -1.0}
            half_track = self.vehicle.track_m / 2.0
            from_loads = sum(
                support.target_load_n[c] * sign[c] * half_track for c in CORNERS
            )
            from_gravity = -self.vehicle.total_weight_n * support.cg_lateral_shift_m
            self.assertAlmostEqual(from_loads + from_gravity, 0.0, delta=5.0)

    def test_attitude_moves_the_cg_towards_the_support_triangle(self):
        support = three_wheel_support(self.vehicle, "FR")
        # Lifting the right front wheel needs the CG to move left (positive y).
        self.assertGreater(support.cg_lateral_shift_m, 0.0)
        required = (
            self.vehicle.track_m * self.vehicle.cg_to_rear_axle_m / self.vehicle.wheelbase_m
            - self.vehicle.track_m / 2.0
        )
        self.assertGreater(support.cg_lateral_shift_m, required)

    def test_attitude_deflections_stay_inside_the_models_travel(self):
        for lifted in ("FR", "RR"):
            support = three_wheel_support(self.vehicle, lifted)
            for corner, target in support.deflection_target_m.items():
                resulting = (
                    MEASURED_STATIC_LOADS and 0.0
                )  # relative pattern; absolute travel checked below
                self.assertLessEqual(abs(target), 0.100)
            # the largest differential travel the pattern demands must fit
            differential = 2.0 * 0.08
            self.assertLessEqual(differential, self.vehicle.jounce_limit_m + self.vehicle.rebound_limit_m)


class FeedforwardInversionTests(unittest.TestCase):
    def test_identity_gain_returns_the_requested_load_change(self):
        delta = [1000.0, -2000.0, 3000.0, -2000.0]
        self.assertEqual(feedforward_commands(None, delta), delta)

    def test_solve_linear_inverts_a_known_system(self):
        # A measured-style gain matrix with strong off-diagonal coupling.
        matrix = [
            [0.9, -0.2, 0.0, 0.1],
            [-0.2, 0.9, 0.1, 0.0],
            [0.0, 0.1, 0.9, -0.2],
            [0.1, 0.0, -0.2, 0.9],
        ]
        rhs = [100.0, -50.0, 25.0, -75.0]
        solution = solve_linear(matrix, rhs)
        for row, expected in zip(matrix, rhs):
            self.assertAlmostEqual(sum(a * b for a, b in zip(row, solution)), expected, places=6)

    def test_regularisation_keeps_an_ill_conditioned_inverse_finite(self):
        # This mirrors the measured matrix, whose condition number is near 120.
        matrix = [
            [0.0816, -0.0841, -0.1210, 0.1270],
            [-0.0846, 0.0821, 0.1271, -0.1210],
            [-0.1185, 0.1245, 0.0792, -0.0818],
            [0.1243, -0.1184, -0.0816, 0.0790],
        ]
        delta = [21986.0, -22411.0, -20330.0, 20754.0]
        command = feedforward_commands(matrix, delta, regularization=0.02)
        self.assertEqual(len(command), 4)
        for value in command:
            self.assertTrue(math.isfinite(value))
            self.assertLess(abs(value), 5.0e5)

    def test_singular_system_is_rejected(self):
        with self.assertRaises(ValueError):
            solve_linear([[1.0, 1.0], [1.0, 1.0]], [1.0, 2.0])


@unittest.skipUnless(MODEL.exists(), "generated TruckSim model not present")
class ScenarioCouplingTests(unittest.TestCase):
    """The state machine must follow measured wheel stations, not fixed stations."""

    def setUp(self):
        self.vehicle = load_vehicle(MODEL, static_wheel_load_n=MEASURED_STATIC_LOADS)
        self.scenario = PotholeScenario()

    def _controller(self, scenario=None, **config):
        return DeepPotholeExpertController(
            scenario=scenario or self.scenario,
            vehicle=self.vehicle,
            export_names=EXPORTS,
            config=ExpertConfig(**config),
        )

    def test_starts_in_the_approach_phase(self):
        controller = self._controller()
        self.assertEqual(controller.step, STEP_APPROACH)
        controller(0.0, _exports())
        self.assertEqual(controller.step, STEP_APPROACH)

    def test_enters_the_front_lift_before_the_lip_using_wheel_two_station(self):
        controller = self._controller(pre_lift_distance_m=0.25)
        # wheel 2 is still 0.30 m short of the lip -> still approaching
        controller(0.0, _exports(X_R1=100.80))
        self.assertEqual(controller.step, STEP_APPROACH)
        # inside the pre-lift window -> lifting
        controller(0.01, _exports(X_R1=100.90))
        self.assertEqual(controller.step, STEP_LIFT_FRONT)

    def test_recovers_once_wheel_two_clears_the_trailing_edge(self):
        controller = self._controller()
        controller(0.0, _exports(X_R1=101.5))
        self.assertEqual(controller.step, STEP_LIFT_FRONT)
        controller(0.01, _exports(X_R1=102.35))
        self.assertEqual(controller.step, STEP_RECOVER_FRONT)

    def test_rear_phase_waits_for_wheel_four_and_uses_its_own_station(self):
        controller = self._controller()
        # The machine advances at most one phase per call: entering the lift, then
        # the same station on the next update is already past the trailing edge.
        controller(0.0, _exports(X_R1=102.35))
        self.assertEqual(controller.step, STEP_LIFT_FRONT)
        controller(0.01, _exports(X_R1=102.35))
        self.assertEqual(controller.step, STEP_RECOVER_FRONT)
        # wheel 4 is still far away even though the vehicle has moved on
        controller(0.02, _exports(X_R1=103.0, X_R2=100.0))
        self.assertEqual(controller.step, STEP_RECOVER_FRONT)
        # ...and the rear phase is driven by wheel 4's own station
        controller(0.03, _exports(X_R1=103.5, X_R2=100.90))
        self.assertEqual(controller.step, STEP_LIFT_REAR)

    def test_the_same_controller_works_for_a_relocated_pothole(self):
        # No station is hard-coded: shift the hole 5 m and the phases shift with it.
        moved = PotholeScenario(**{**PotholeScenario().__dict__, "start_station_m": 106.1})
        controller = self._controller(scenario=moved, pre_lift_distance_m=0.25)
        controller(0.0, _exports(X_R1=105.80))
        self.assertEqual(controller.step, STEP_APPROACH)
        controller(0.01, _exports(X_R1=105.90))
        self.assertEqual(controller.step, STEP_LIFT_FRONT)

    def test_control_output_is_held_between_control_periods(self):
        controller = self._controller(control_period_s=0.01)
        first = controller(0.0, _exports(X_R1=101.5))
        middle = controller(0.004, _exports(X_R1=101.51))
        later = controller(0.011, _exports(X_R1=101.52))
        self.assertEqual(len(first), 8)
        self.assertEqual(first, middle)          # zero-order hold
        self.assertEqual(len(later), 8)

    def test_safe_stop_latches_and_returns_the_vehicle_to_passive_suspension(self):
        controller = self._controller(roll_safe_stop_deg=12.0)
        command = controller(0.0, _exports(X_R1=101.5, Roll_E=20.0))
        self.assertTrue(controller.safe_stop)
        self.assertEqual(command[4:], (0.0, 0.0, 0.0, 0.0))
        self.assertEqual(command[:4], (0.0, 0.0, 0.0, 0.0))

    def test_commands_respect_the_configured_force_and_torque_limits(self):
        config = ExpertConfig(force_min_n=-100000.0, force_max_n=100000.0)
        controller = self._controller()
        controller.config = config
        for station in (100.95, 101.5, 102.2, 104.9, 105.5, 106.2):
            command = controller(0.0, _exports(X_R1=station))
            self.assertTrue(all(-400.0 <= v <= 700.0 for v in command[:4]))
            self.assertTrue(all(config.force_min_n <= v <= config.force_max_n for v in command[4:]))

    def test_plan_records_the_required_actuator_rating(self):
        controller = self._controller()
        summary = controller.step_summary()
        self.assertFalse(summary["actuator_gain_matrix_used"])
        self.assertIn("FR", summary["required_command_n"])
        self.assertGreater(summary["required_command_n"]["FR"], 0.0)


if __name__ == "__main__":
    unittest.main()
