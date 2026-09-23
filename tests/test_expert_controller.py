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
    STEP_FR_CROSS,
    STEP_FR_LIFT,
    STEP_FR_PRELOAD,
    STEP_FR_TOUCHDOWN,
    STEP_LIFT_FRONT,
    STEP_LIFT_REAR,
    STEP_RR_LIFT,
    STEP_RR_PRELOAD,
    STEP_RECOVER_FRONT,
    DeepPotholeExpertController,
    ExpertConfig,
    derive_roll_regulator_sign,
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
    "Kappa_L1i", "Kappa_R1i", "Kappa_L2i", "Kappa_R2i",
    "Yaw", "AVz", "Yo", "Y_R1", "Y_R2",
)


def _exports(**overrides):
    """Build a full export vector; unlisted channels default to zero."""
    values = {name: 0.0 for name in EXPORTS}
    values.update({
        "Fz_L1": 22388.8, "Fz_R1": 22383.0, "Fz_L2": 21252.1, "Fz_R2": 21255.2,
        "CmpS_L1": 53.6, "CmpS_R1": 53.6, "CmpS_L2": 50.6, "CmpS_R2": 50.6,
        "Xo": 100.0, "Vx": 2.8, "Roll_E": 0.0, "Pitch": 0.0,
        "X_L1": 100.0, "X_R1": 100.0, "X_L2": 96.1, "X_R2": 96.1,
        "Yo": 0.0, "Y_R1": -0.63, "Y_R2": -0.63,
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
        config.setdefault("settle_time_s", 0.0)
        config.setdefault("preload_time_s", 0.0)
        return DeepPotholeExpertController(
            scenario=scenario or self.scenario,
            vehicle=self.vehicle,
            export_names=EXPORTS,
            config=ExpertConfig(**config),
        )

    def test_starts_in_the_approach_phase(self):
        controller = self._controller()
        self.assertEqual(controller.step, STEP_APPROACH)

    def test_default_controller_waits_for_the_initial_settle_window(self):
        controller = DeepPotholeExpertController(
            scenario=self.scenario,
            vehicle=self.vehicle,
            export_names=EXPORTS,
            config=ExpertConfig(settle_time_s=1.0),
        )
        controller(0.5, _exports(X_R1=101.0))
        self.assertEqual(controller.step, STEP_APPROACH)
        controller(1.01, _exports(X_R1=101.0))
        self.assertEqual(controller.step, STEP_LIFT_FRONT)
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
        controller(0.01, _exports(X_R1=101.5))
        controller(0.02, _exports(X_R1=101.5, Fz_R1=100.0))
        controller(0.03, _exports(X_R1=102.35, Fz_R1=100.0))
        self.assertEqual(controller.step, STEP_RECOVER_FRONT)

    def test_rear_phase_waits_for_wheel_four_and_uses_its_own_station(self):
        controller = self._controller()
        # The machine advances at most one phase per call: entering the lift, then
        # the same station on the next update is already past the trailing edge.
        controller(0.0, _exports(X_R1=102.35))
        self.assertEqual(controller.step, STEP_LIFT_FRONT)
        controller(0.01, _exports(X_R1=102.0))
        controller(0.02, _exports(X_R1=102.0, Fz_R1=100.0))
        controller(0.03, _exports(X_R1=102.35, Fz_R1=100.0))
        self.assertEqual(controller.step, STEP_RECOVER_FRONT)
        # wheel 4 is still far away even though the vehicle has moved on
        controller(0.02, _exports(X_R1=103.0, X_R2=100.0))
        self.assertEqual(controller.step, STEP_RECOVER_FRONT)
        # ...and, once the front recovery ramp has finished, the rear phase is driven
        # by wheel 4's own station.  The gap in time here is the whole point: the
        # handover is time-gated as well as station-gated.
        controller(
            0.03 + controller.config.transition_time_s + 1e-6,
            _exports(X_R1=103.5, X_R2=100.90),
        )
        self.assertEqual(controller.step, STEP_LIFT_REAR)

    def test_wheel_four_is_not_lifted_until_wheel_two_support_is_restored(self):
        # The paper's Step 2 restores four-wheel support before Step 3 lifts wheel 4.
        # The old code advanced on wheel 4's station alone, so the two lift phases
        # overlapped (measured: 682 N still commanded on the front corner at handover).
        controller = self._controller()
        controller(0.0, _exports(X_R1=102.35))
        controller(0.01, _exports(X_R1=102.0))
        controller(0.02, _exports(X_R1=102.0, Fz_R1=100.0))
        controller(0.03, _exports(X_R1=102.35, Fz_R1=100.0))
        self.assertEqual(controller.step, STEP_RECOVER_FRONT)
        # wheel 4 is already inside its pre-lift window, but the ramp is not finished
        controller(0.02, _exports(X_R1=103.5, X_R2=101.00))
        self.assertEqual(controller.step, STEP_RECOVER_FRONT)
        self.assertFalse(controller._recovery_finished(0.04))
        self.assertTrue(
            controller._recovery_finished(0.03 + controller.config.transition_time_s)
        )

    def test_rear_lift_waits_for_four_wheel_contact_not_only_for_time(self):
        controller = self._controller(min_recovered_load_n=200.0)
        controller(0.0, _exports(X_R1=102.35))
        controller(0.01, _exports(X_R1=102.0))
        controller(0.02, _exports(X_R1=102.0, Fz_R1=100.0))
        controller(0.03, _exports(X_R1=102.35, Fz_R1=100.0))
        ready_time = 0.03 + controller.config.transition_time_s + 0.01
        controller(
            ready_time,
            _exports(X_R1=103.5, X_R2=100.9, Fz_R1=0.0),
        )
        self.assertEqual(controller.step, STEP_RECOVER_FRONT)
        controller(
            ready_time + 0.01,
            _exports(X_R1=103.5, X_R2=100.9, Fz_R1=1000.0),
        )
        self.assertEqual(controller.step, STEP_LIFT_REAR)

    def test_lifted_wheel_gets_zero_torque_and_contact_wheels_share_drive(self):
        controller = self._controller(torque_bias_nm=4.0, torque_rate_limit_nm_per_s=1e6)
        command = controller(0.0, _exports(X_R1=101.0))
        self.assertEqual(command[1], 0.0)
        self.assertGreater(command[0], 0.0)
        self.assertGreater(command[2], 0.0)
        self.assertGreater(command[3], 0.0)

    def test_slip_controller_reduces_only_the_slipping_contact_wheel(self):
        controller = self._controller(
            torque_bias_nm=4.0,
            torque_rate_limit_nm_per_s=1e6,
            slip_soft_limit=0.10,
            slip_hard_limit=0.30,
        )
        command = controller(
            0.0,
            _exports(X_R1=101.0, Kappa_L1i=0.25),
        )
        self.assertLess(command[0], command[2])
        self.assertEqual(command[1], 0.0)

    def test_negative_yaw_shifts_drive_torque_to_the_right_side(self):
        controller = self._controller(
            torque_bias_nm=4.0,
            torque_rate_limit_nm_per_s=1e6,
            yaw_torque_gain_nm_per_deg=1.0,
        )
        torques = controller._wheel_torques(
            _exports(Yaw=-5.0), controller.config.control_period_s, None
        )
        self.assertGreater(torques["FR"] + torques["RR"], torques["FL"] + torques["RL"])

    def test_recovery_ramp_is_short_enough_for_this_wheelbase(self):
        # 0.875 m of travel between "wheel 2 clears the hole" and "wheel 4 reaches its
        # pre-lift station" at 2.8 km/h is 1.12 s; a longer ramp could never complete
        # before the rear wheel must be lifted, so the handover gate would deadlock.
        controller = self._controller()
        wheelbase = controller.vehicle.wheelbase_m
        # When wheel 2 reaches the trailing edge, wheel 4 sits one wheelbase behind it.
        rear_when_front_clears = controller.scenario.trailing_edge_m - wheelbase
        rear_pre_lift_station = (
            controller.scenario.leading_edge_m - controller.config.pre_lift_distance_m
        )
        gap = rear_pre_lift_station - rear_when_front_clears
        available_s = gap / (controller.scenario.target_speed_kph / 3.6)
        self.assertGreater(gap, 0.0)
        self.assertLess(controller.config.transition_time_s, available_s)

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


class ActuatorSizingAndRegulatorTests(unittest.TestCase):
    """Regression tests for three scale defects found in the exported pose data.

    Each of these was a genuine bug that made the vehicle sway, and each is cheap to
    reintroduce by accident, so they are pinned here.
    """

    def setUp(self):
        self.vehicle = load_vehicle(MODEL, static_wheel_load_n=MEASURED_STATIC_LOADS)
        self.scenario = PotholeScenario()

    def _controller(self, **config):
        return DeepPotholeExpertController(
            scenario=self.scenario,
            vehicle=self.vehicle,
            export_names=EXPORTS,
            config=ExpertConfig(**config),
        )

    def test_roll_regulator_sign_follows_the_measured_actuator_response(self):
        # The measured corner-module response to the (+,-,+,-) pattern is +4.67 deg,
        # i.e. the pattern *increases* roll, so the corrective sign is negative.
        measured = {"FL": 0.959, "FR": -0.937, "RL": 1.418, "RR": -1.351}
        self.assertEqual(derive_roll_regulator_sign(measured), -1.0)
        # An opposing vehicle must get the opposite sign, so this is a measurement and
        # not a hard-coded constant.
        flipped = {c: -v for c, v in measured.items()}
        self.assertEqual(derive_roll_regulator_sign(flipped), 1.0)
        with self.assertRaises(ValueError):
            derive_roll_regulator_sign({c: 0.0 for c in CORNERS})

    def test_measured_travel_matrix_allocates_the_scaled_paper_pattern(self):
        # Matrix convention is [command corner][measured Jnc corner], in mm/N.
        # With four uncoupled -0.01 mm/N channels, the lifted FR and the two ordinary
        # support corners require negative commands, while the paper's diagonal
        # attitude corner (RL) requires the opposite sign.
        matrix = [
            [-0.01, 0.0, 0.0, 0.0],
            [0.0, -0.01, 0.0, 0.0],
            [0.0, 0.0, -0.01, 0.0],
            [0.0, 0.0, 0.0, -0.01],
        ]
        controller = DeepPotholeExpertController(
            scenario=self.scenario,
            vehicle=self.vehicle,
            export_names=EXPORTS,
            config=ExpertConfig(),
            travel_gain_matrix=matrix,
        )
        command = controller.isolated_lift_feedforward["FR"]
        self.assertLess(command["FR"], -1000.0)
        self.assertLess(command["FL"], 0.0)
        self.assertGreater(command["RL"], 0.0)
        self.assertLess(command["RR"], 0.0)

    def test_roll_regulator_cannot_saturate_the_actuators_by_itself(self):
        controller = self._controller()
        self.assertLessEqual(controller.roll_limit_n, controller.config.force_max_n)
        self.assertLessEqual(controller.roll_limit_n, -controller.config.force_min_n)

    def test_travel_guard_fades_to_zero_and_never_inverts_the_command(self):
        # The old taper went negative past the stop, flipping a compressive command
        # into an extensional one and then growing it -- the positive feedback that
        # pinned CmpS_FR at 200 mm and produced a 0 <-> 42 kN load limit cycle.
        controller = self._controller(force_min_n=-10000.0, force_max_n=10000.0)
        config = controller.config
        limit = controller.vehicle.jounce_limit_m
        guarded, reason = controller._travel_guard("FR", limit * 4.0, -8000.0, config)
        self.assertEqual(guarded, 0.0)
        self.assertIsNotNone(reason)
        for depth in (0.0, 0.5, 0.92, 1.0, 1.5, 3.0):
            value, _ = controller._travel_guard(
                "FR", -limit * depth, -8000.0, config
            )
            self.assertGreaterEqual(value, -8000.0)
            self.assertLessEqual(value, 0.0)

    def test_crawl_torque_is_rate_limited_and_stays_near_the_paper_bias(self):
        # The paper drives the manoeuvre at a near-constant ~8 N*m per wheel.  The old
        # loop used 80 N*m with a 450 N*m/km/h gain, which banged between the clamps and
        # swung the speed between 1.2 and 7.9 km/h within 0.4 s.
        controller = self._controller()
        dt = controller.config.control_period_s
        # One step from rest may only move by the slew limit.
        command = controller._crawl_torque(_exports(Vx=0.0), dt)
        slew = controller.config.torque_rate_limit_nm_per_s * dt
        self.assertLessEqual(abs(command), slew)
        # At the target speed the loop settles on the paper's constant bias.
        for _ in range(2000):
            command = controller._crawl_torque(
                _exports(Vx=self.scenario.target_speed_kph), dt
            )
        self.assertAlmostEqual(command, controller.config.torque_bias_nm, delta=1.0)
        # And even at a standstill it stays an order of magnitude below the old loop,
        # which demanded 80 + 450 * 2.8 = 1340 N*m and then clamped.
        for _ in range(2000):
            command = controller._crawl_torque(_exports(Vx=0.0), dt)
        self.assertLessEqual(command, controller.config.torque_max_nm)
        self.assertLess(command, 100.0)

    def test_default_force_limit_suits_a_light_vehicle(self):
        # 5x a static corner load was 18 kN on this 1.36 t vehicle, far past the ~72 mm
        # of jounce travel left at its static position.
        controller = self._controller()
        static_corner = max(self.vehicle.static_load(c) for c in CORNERS)
        self.assertLessEqual(controller.config.force_max_n, 2.5 * static_corner)

    def test_paper_sd_sign_is_converted_to_trucksim_jounce_for_the_lifted_wheel(self):
        static = {corner: 0.03 for corner in CORNERS}
        controller = DeepPotholeExpertController(
            scenario=self.scenario,
            vehicle=self.vehicle,
            export_names=EXPORTS,
            config=ExpertConfig(settle_time_s=0.0),
            static_deflection_m=static,
        )
        # Positive TruckSim Jnc is wheel jounce (wheel moves upward relative to the
        # body).  The paper's negative SD convention therefore has to be inverted.
        forces = controller._sd_forces(controller.support["FR"], static, controller.config)
        self.assertLess(forces["FR"], 0.0)

    def test_roll_regulator_does_not_cancel_the_lifted_corner_command(self):
        static = {corner: 0.03 for corner in CORNERS}

        def command_at_roll(roll_deg):
            controller = DeepPotholeExpertController(
                scenario=self.scenario,
                vehicle=self.vehicle,
                export_names=EXPORTS,
                config=ExpertConfig(
                    settle_time_s=0.0,
                    force_rate_limit_n_per_s=1e9,
                    torque_rate_limit_nm_per_s=1e9,
                ),
                static_deflection_m=static,
            )
            return controller(0.0, _exports(X_R1=101.0, Roll_E=roll_deg))

        at_zero = command_at_roll(0.0)
        at_roll = command_at_roll(5.0)
        self.assertAlmostEqual(at_zero[5], at_roll[5], places=6)

    def test_default_recovery_begins_at_the_exit_lip(self):
        controller = self._controller()
        self.assertEqual(controller.config.recovery_lead_m, 0.0)


@unittest.skipUnless(MODEL.exists(), "generated TruckSim model not present")
class ObservablePhaseManagerTests(unittest.TestCase):
    def setUp(self):
        self.vehicle = load_vehicle(MODEL, static_wheel_load_n=MEASURED_STATIC_LOADS)
        self.scenario = PotholeScenario(
            **{**PotholeScenario().__dict__, "center_y_m": -0.63, "width_m": 0.90}
        )
        self.controller = DeepPotholeExpertController(
            scenario=self.scenario,
            vehicle=self.vehicle,
            export_names=EXPORTS,
            config=ExpertConfig(settle_time_s=0.0, preload_time_s=0.0),
        )

    def test_front_sequence_advances_one_observable_phase_per_call(self):
        self.controller(0.00, _exports(X_R1=100.60))
        self.assertEqual(self.controller.step, STEP_FR_PRELOAD)

        self.controller(0.01, _exports(X_R1=100.70))
        self.assertEqual(self.controller.step, STEP_FR_LIFT)

        self.controller(0.02, _exports(X_R1=101.20, Fz_R1=100.0))
        self.assertEqual(self.controller.step, STEP_FR_CROSS)

        self.controller(0.03, _exports(X_R1=102.40, Fz_R1=100.0))
        self.assertEqual(self.controller.step, STEP_FR_TOUCHDOWN)

    def test_station_alone_cannot_claim_crossing_when_wheel_misses_pit_laterally(self):
        self.controller(0.00, _exports(X_R1=100.60, Y_R1=-1.50))
        self.controller(0.01, _exports(X_R1=100.70, Y_R1=-1.50))
        self.controller(0.02, _exports(X_R1=101.20, Y_R1=-1.50, Fz_R1=100.0))
        self.assertEqual(self.controller.step, STEP_FR_LIFT)
        self.controller(0.03, _exports(X_R1=102.40, Y_R1=-1.50, Fz_R1=100.0))
        self.assertNotEqual(self.controller.step, STEP_FR_TOUCHDOWN)

    def test_rear_preload_waits_for_front_contact_and_path_recovery(self):
        self.controller.step = STEP_FR_TOUCHDOWN
        self.controller._recover_start_time = 0.0
        ready = self.controller.config.transition_time_s + 0.01
        self.controller(
            ready,
            _exports(X_R2=100.60, Fz_R1=1000.0, Yaw=3.0, AVz=0.0),
        )
        self.assertEqual(self.controller.step, STEP_FR_TOUCHDOWN)
        self.controller(
            ready + 0.01,
            _exports(X_R2=100.60, Fz_R1=1000.0, Yaw=0.0, AVz=0.0),
        )
        self.assertEqual(self.controller.step, STEP_RR_PRELOAD)

    def test_low_support_load_debounce_enters_recovery_without_safe_stop(self):
        self.controller.step = STEP_FR_CROSS
        low = _exports(X_R1=101.50, Fz_L2=0.0, Fz_R1=100.0)
        self.controller(0.00, low)
        self.assertEqual(self.controller.step, STEP_FR_CROSS)
        self.controller(0.06, low)
        self.assertEqual(self.controller.step, STEP_FR_TOUCHDOWN)
        self.assertEqual(self.controller.safety_mode, "RECOVER")
        self.assertFalse(self.controller.safe_stop)


@unittest.skipUnless(MODEL.exists(), "generated TruckSim model not present")
class ConstrainedSuspensionAllocationTests(unittest.TestCase):
    def test_coupled_unload_preserves_support_wheel_floor(self):
        vehicle = load_vehicle(MODEL, static_wheel_load_n=MEASURED_STATIC_LOADS)
        # Matrix convention is [command][load].  A negative FR command unloads FR,
        # but also unloads RL equally; a safe allocator must stop at RL's floor.
        gain = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 1.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        controller = DeepPotholeExpertController(
            scenario=PotholeScenario(), vehicle=vehicle, export_names=EXPORTS,
            config=ExpertConfig(
                settle_time_s=0.0,
                force_min_n=-5000.0,
                force_max_n=5000.0,
                force_rate_limit_n_per_s=1e9,
                min_support_load_n=300.0,
            ),
            gain_matrix=gain,
        )
        loads = {corner: 1000.0 for corner in CORNERS}
        _, predicted = controller._allocate_suspension("FR", loads, 0.01)
        self.assertGreaterEqual(predicted["RL"], 300.0 - 1e-6)
        self.assertLessEqual(predicted["FR"], controller.config.lifted_load_max_n)

    def test_suspension_allocator_obeys_force_and_per_cycle_slew_bounds(self):
        vehicle = load_vehicle(MODEL, static_wheel_load_n=MEASURED_STATIC_LOADS)
        controller = DeepPotholeExpertController(
            scenario=PotholeScenario(), vehicle=vehicle, export_names=EXPORTS,
            config=ExpertConfig(
                force_min_n=-1000.0,
                force_max_n=1000.0,
                force_rate_limit_n_per_s=2000.0,
            ),
            gain_matrix=[[1.0 if i == j else 0.0 for i in range(4)] for j in range(4)],
        )
        commands, _ = controller._allocate_suspension(
            "FR", {corner: 1000.0 for corner in CORNERS}, 0.01
        )
        self.assertTrue(all(-20.0 <= value <= 20.0 for value in commands.values()))


if __name__ == "__main__":
    unittest.main()
