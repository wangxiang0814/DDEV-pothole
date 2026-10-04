"""Simulation-only limits for the I_I static FR preload gate."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

from .identification_settle import IdentificationSettleLimits


@dataclass(frozen=True)
class PreloadGateConfig:
    lambda_safe: float = 0.05
    support_floor_n: float = 300.0
    attitude_limit_deg: float = 5.0
    force_limit_static_multiple: float = 5.0
    fr_half_fraction: float = 0.5


PRELOAD_GATE = PreloadGateConfig()


@dataclass(frozen=True)
class CoupledProbeConfig:
    max_amplitude_n: float = 3000.0
    max_slew_n_s: float = 500.0
    start_s: float = 7.0
    ramp_s: float = 8.0
    hold_after_ramp_s: float = 5.0
    settled_delay_s: float = 3.0
    settled_window_s: float = 1.0
    recovery_s: float = 2.0
    min_wheel_load_n: float = 500.0
    min_travel_mm: float = -95.0
    max_travel_mm: float = 140.0
    max_attitude_deg: float = 5.0
    max_rate_deg_s: float = 1.0
    max_vx_kph: float = 0.036
    max_wheel_rpm: float = 1.0


COUPLED_PROBE = CoupledProbeConfig()


@dataclass(frozen=True)
class LocalIdentificationConfig:
    base_start_s: float = 7.0
    base_ramp_s: float = 24.0
    perturb_start_s: float = 34.0
    perturb_ramp_s: float = 8.0
    settled_start_s: float = 44.0
    settled_end_s: float = 45.0
    duration_s: float = 48.0
    perturb_amplitude_n: float = 500.0
    max_corner_force_n: float = 4000.0


LOCAL_ID = LocalIdentificationConfig()


@dataclass(frozen=True)
class LocalStepConfig:
    incremental_force_bound_n: float = 1500.0
    fr_load_drop_target_n: float = 300.0
    max_total_diagnostic_force_n: float = 5000.0
    base_start_s: float = 7.0
    base_ramp_s: float = 24.0
    step_start_s: float = 34.0
    step_ramp_s: float = 32.0
    duration_s: float = 72.0
    settled_start_s: float = 70.0
    settled_end_s: float = 71.0


LOCAL_STEP = LocalStepConfig()


@dataclass(frozen=True)
class SmallUnloadConfig:
    force_step_bound_n: float = 2000.0
    fr_drop_target_n: float = 600.0
    validated_online_gain_multiplier: float = 4.0
    margin_reserve: float = 0.003
    margin_gain_target: float = 0.0005
    planning_min_travel_mm: float = -80.0
    load_filter_cutoff_hz: float = 10.0
    start_s: float = 7.0
    settle_s: float = 0.5
    ramp_s: float = 8.0
    hold_s: float = 1.0
    fr_tolerance_n: float = 30.0
    duration_s: float = 20.0
    settled_start_s: float = 17.0
    settled_end_s: float = 18.0


SMALL_UNLOAD = SmallUnloadConfig()


@dataclass(frozen=True)
class FollowupUnloadConfig:
    base_start_s: float = 7.0
    base_ramp_s: float = 24.0
    step_earliest_s: float = 34.0
    duration_s: float = 48.0
    settled_start_s: float = 44.0
    settled_end_s: float = 45.0


FOLLOWUP_UNLOAD = FollowupUnloadConfig()


ID_SETTLE = IdentificationSettleLimits()


@dataclass(frozen=True)
class OnlineGainConfig:
    max_fz_prediction_error_n: float = 20.0
    max_attitude_prediction_error_rad: float = 0.002
    max_travel_prediction_error_m: float = 0.003
    max_com_prediction_error_m: float = 0.002
    max_zmp_prediction_error_m: float = 0.002
    max_updates_since_full_id: int = 10


ONLINE_GAIN = OnlineGainConfig()


@dataclass(frozen=True)
class DirectionalRecalibrationConfig:
    max_fz_prediction_error_n: float = 500.0
    max_attitude_prediction_error_rad: float = 0.01
    max_travel_prediction_error_m: float = 0.02
    max_com_prediction_error_m: float = 0.01
    max_zmp_prediction_error_m: float = 0.01


DIRECTIONAL_RECALIBRATION = DirectionalRecalibrationConfig()


@dataclass(frozen=True)
class HoldValidationConfig:
    near_zero_fr_n: float = 100.0
    required_hold_s: float = 5.0


HOLD_VALIDATION = HoldValidationConfig()


@dataclass(frozen=True)
class LiftProbeConfig:
    extra_fr_force_n: float = -600.0
    start_s: float = 48.0
    ramp_s: float = 4.0
    hold_s: float = 7.0
    lower_s: float = 4.0
    return_to_four_s: float = 12.0
    recovery_s: float = 3.0
    clearance_target_m: float = 0.01
    clearance_positive_m: float = 0.002
    near_zero_fr_n: float = 100.0
    support_adjust_ramp_s: float = 2.0
    max_support_adjust_n: float = 5000.0
    max_transition_rate_deg_s: float = 3.0
    max_transition_vx_kph: float = 0.5


LIFT_PROBE = LiftProbeConfig()


@dataclass(frozen=True)
class ClosedLoopRunConfig:
    control_period_s: float = 0.02
    init_settle_s: float = 7.0
    wheel_load_filter_cutoff_hz: float = 5.0
    preload_feedback_start_s: float = 7.0
    preload_max_reference_rate: float = 1.5
    preload_slow_tracking_error_n: float = 200.0
    preload_pause_tracking_error_n: float = 500.0
    preload_end_wait_limit_s: float = 3.0
    preload_total_limit_s: float = 60.0
    gain_blend_swing_fr_load_n: float = 50.0
    gain_blend_stance_fr_load_n: float = 300.0
    ready_dwell_s: float = 0.5
    three_wheel_hold_s: float = 5.0
    lift_ramp_s: float = 4.0
    support_ramp_s: float = 2.0
    lower_ramp_s: float = 4.0
    return_ramp_s: float = 12.0
    fr_lift_force_n: float = -100.0
    max_lift_trial_force_n: float = 1000.0
    rl_support_force_n: float = -4500.0
    fr_feedback_limit_n: float = 500.0
    fr_feedback_slew_n_s: float = 400.0
    fr_feedback_tracking_gain: float = 0.1
    fr_feedback_deadband_n: float = 10.0
    support_feedback_limit_n: float = 500.0
    max_support_trial_feedback_n: float = 2000.0
    max_continuous_support_saturation_s: float = 0.5
    support_floor_n: float = 500.0
    lambda_target: float = 0.05
    lambda_abort: float = 0.015
    fr_ready_load_n: float = 100.0
    fr_swing_load_n: float = 100.0
    lift_clearance_m: float = 0.01
    lip_clearance_m: float = 0.005
    post_pit_positive_clearance_m: float = 0.002
    attitude_limit_deg: float = 8.0
    rebound_guard_mm: float = -95.0
    rebound_abort_mm: float = -99.0
    moving_rebound_guard_mm: float = -140.0
    moving_rebound_abort_mm: float = -149.0
    jounce_abort_mm: float = 155.0
    crawl_speed_kph: float = 2.2
    crawl_accel_ramp_s: float = 0.0
    crawl_min_speed_kph: float = 2.0
    crawl_max_speed_kph: float = 8.0
    crawl_max_lateral_error_m: float = 0.05
    crawl_max_yaw_error_deg: float = 2.0
    crossing_clearance_m: float = 0.30
    brake_start_before_far_edge_m: float = 0.15
    rear_stop_clearance_m: float = 0.05
    stop_speed_kph: float = 0.1
    stop_dwell_s: float = 1.0
    stop_ramp_s: float = 2.2
    max_stop_trial_ramp_s: float = 4.0
    stop_support_slew_n_s: float = 400.0
    max_stop_support_trial_slew_n_s: float = 1200.0
    stop_max_brake_nm: float = 80.0
    torque_release_s: float = 1.0
    max_crawl_s: float = 40.0
    tyre_low_speed_slip_m_s: float = 0.2
    tyre_radius_m: float = 0.263
    tyre_half_track_m: float = 0.625
    reference_hold_s: float = 56.0
    init_max_vx_kph: float = 0.1
    ready_attitude_limit_deg: float = 5.0


CLOSED_LOOP_RUN = ClosedLoopRunConfig()
BALANCED_CYCLE_FRONT = replace(
    CLOSED_LOOP_RUN,
    init_settle_s=3.0,
    preload_max_reference_rate=1.7,
)
FAST_CYCLE_FRONT = replace(
    CLOSED_LOOP_RUN,
    init_settle_s=2.0,
    preload_max_reference_rate=2.0,
)
COMPACT_CYCLE_FRONT = replace(
    BALANCED_CYCLE_FRONT,
    preload_max_reference_rate=3.0,
)
EFFICIENT_CYCLE_FRONT = replace(
    COMPACT_CYCLE_FRONT,
    preload_max_reference_rate=5.0,
    return_ramp_s=9.0,
)


@dataclass(frozen=True)
class CurrentModelProbeConfig:
    amplitude_n: float = 250.0
    duration_s: float = 18.0
    ramp_start_s: float = 8.0
    ramp_duration_s: float = 2.0
    window_start_s: float = 16.0
    window_end_s: float = 18.0
    log_decimation: int = 10
    hold_age_s: float = 3.0
    swing_unloaded_n: float = 100.0
    swing_support_floor_n: float = 500.0
    swing_clearance_m: float = 0.01
    lambda_safe: float = 0.05
    tyre_radius_m: float = 0.263
    attitude_limit_deg: float = 10.0
    travel_min_mm: float = -149.0
    travel_max_mm: float = 155.0


CURRENT_MODEL_PROBE = CurrentModelProbeConfig()
RECOVERY_PRELOAD_ABORT_AFTER_S = 1.0


@dataclass(frozen=True)
class SupportQPConfig:
    lambda_target: float = 0.075
    lambda_floor: float = 0.05
    clearance_floor_m: float = 0.01
    swing_load_ceiling_n: float = 100.0
    tyre_radius_m: float = 0.263
    support_floor_n: float = 500.0
    max_load_n: float = 12000.0
    force_limit_n: float = 18200.0
    correction_limit_n: float = 500.0
    force_slew_n_s: float = 400.0
    abort_force_slew_n_s: float = 1200.0
    abort_correction_limit_n: float = 750.0
    travel_lower_m: float = -0.149
    travel_upper_m: float = 0.155
    attitude_limit_deg: float = 10.0
    load_scale_n: float = 1000.0
    attitude_scale_deg: float = 3.0
    load_weight: float = 1.0
    attitude_weight: float = 2.0
    tracking_gain: float = 0.1
    attitude_damping_s: float = 0.8
    increment_weight: float = 0.002
    correction_weight: float = 0.002
    filter_cutoff_hz: float = 5.0
    release_s: float = 4.0
    max_iterations: int = 80
    direct_feasible_solve: bool = True
    solver_tolerance: float = 1e-10
    constraint_tolerance: float = 1e-7
    max_continuous_failure_s: float = 0.5
    transition_clearance_slack_m: float = 0.002
    transition_contact_blend_low_n: float = 50.0
    transition_contact_blend_high_n: float = 300.0


SUPPORT_QP = SupportQPConfig()


@dataclass(frozen=True)
class FeedbackHealthConfig:
    max_packet_age_s: float = .12
    torque_release_slew_nm_s: float = 500.
    log_period_s: float = .02


FEEDBACK_HEALTH = FeedbackHealthConfig()


@dataclass(frozen=True)
class FeedbackFaultTrialConfig:
    after_hold_s: float = .5
    packet_outage_s: float = 1.
    moving_pit_fraction: float = .2
    moving_observe_s: float = 25.
    moving_stopped_hold_s: float = 5.


FEEDBACK_FAULT_TRIAL = FeedbackFaultTrialConfig()


@dataclass(frozen=True)
class PathReferenceConfig:
    window_s: float = 1.0
    period_s: float = 0.02
    min_samples: int = 30


PATH_REFERENCE = PathReferenceConfig()


@dataclass(frozen=True)
class RearCycleConfig:
    control_period_s: float = 0.02
    post_complete_observe_s: float = 2.0
    settle_dwell_s: float = 1.0
    preload_ramp_s: float = 8.0
    preload_ready_timeout_s: float = 12.0
    adaptive_preload: bool = False
    sequential_preload: bool = False
    preload_diagonal_fraction: float = 0.5
    preload_fl_unload_share: float = 0.51
    adaptive_preload_timeout_s: float = 20.0
    preload_yaw_slow_deg_s: float = 0.6
    preload_yaw_pause_deg_s: float = 2.0
    preload_fl_force_n: float = -3200.0
    preload_fr_force_n: float = 6400.0
    preload_rl_force_n: float = 0.0
    preload_feedback_fl_per_fr: float = -0.5
    preload_feedback_limit_n: float = 1000.0
    preload_feedback_slew_n_s: float = 1000.0
    preload_feedback_gain: float = 0.1
    preload_feedback_deadband_n: float = 10.0
    max_continuous_feedback_saturation_s: float = 0.5
    sim_force_limit_n: float = 18200.0
    sim_force_slew_n_s: float = 45400.0
    lift_force_n: float = -200.0
    max_lift_trial_force_n: float = 1000.0
    lift_ramp_s: float = 4.0
    lift_timeout_s: float = 8.0
    lift_entry_clearance_m: float = 0.01
    posture_fl_force_n: float = -250.0
    posture_hold_fl_force_n: float = -100.0
    max_posture_trial_force_n: float = 1000.0
    posture_ramp_s: float = 1.0
    posture_target_roll_deg: float = -7.2
    posture_target_clearance_m: float = 0.045
    posture_release_s: float = 0.5
    posture_settle_s: float = 1.0
    posture_timeout_s: float = 4.0
    posture_stop_release_s: float = 0.8
    stop_roll_counter_fl_n: float = 250.0
    stop_roll_counter_ramp_s: float = 0.5
    stop_travel_relief_start_mm: float = -143.0
    stop_travel_relief_gain_n_per_mm: float = 80.0
    stop_travel_relief_limit_n: float = 500.0
    stop_travel_relief_slew_n_s: float = 1000.0
    parking_damping_nm_per_rpm: float = 0.0
    parking_limit_nm: float = 150.0
    parking_slew_nm_s: float = 500.0
    hold_s: float = 5.0
    lower_ramp_s: float = 4.0
    return_ramp_s: float = 12.0
    ready_dwell_s: float = 0.5
    support_floor_n: float = 500.0
    wheel_unloaded_n: float = 100.0
    lambda_safe: float = 0.05
    lambda_abort: float = 0.015
    clearance_m: float = 0.01
    lip_clearance_m: float = 0.005
    attitude_limit_deg: float = 9.0
    stationary_kph: float = 0.1
    travel_rebound_abort_mm: float = -149.0
    travel_jounce_abort_mm: float = 155.0
    tyre_radius_m: float = 0.263
    tyre_half_track_m: float = 0.625
    crawl_speed_kph: float = 2.2
    crawl_accel_ramp_s: float = 2.0
    min_pit_speed_kph: float = 2.0
    max_crawl_speed_kph: float = 8.0
    max_lateral_error_m: float = 0.05
    pit_evaluation_edge_trim_m: float = 0.1
    crawl_yaw_kp_nm_per_deg: float = 80.0
    crawl_yaw_kd_nm_per_deg_s: float = 25.0
    crawl_lateral_kp_n_per_m: float = 800.0
    steer_lateral_deg_per_m: float = 1000.0
    steer_yaw_deg_per_deg: float = 15.0
    steer_limit_deg: float = 220.0
    steer_slew_deg_s: float = 400.0
    static_steering_feedback: bool = False
    brake_start_before_far_edge_m: float = 0.15
    crossing_clearance_m: float = 0.30
    stop_ramp_s: float = 2.2
    stop_dwell_s: float = 1.0
    stop_max_brake_nm: float = 80.0
    max_crawl_s: float = 30.0
    allow_abort_exit: bool = False
    abort_stop_ramp_s: float = 1.0
    abort_exit_dwell_s: float = 2.0
    abort_exit_speed_kph: float = 0.7
    abort_exit_accel_s: float = 1.0
    abort_exit_stop_buffer_m: float = 0.15
    abort_exit_timeout_s: float = 10.0


REAR_CYCLE_RUN = RearCycleConfig()
TUNED_REAR_RUN = replace(
    REAR_CYCLE_RUN,
    lift_entry_clearance_m=0.003,
    posture_fl_force_n=-500.0,
    posture_hold_fl_force_n=-300.0,
    posture_target_clearance_m=0.035,
    attitude_limit_deg=10.0,
)
FAST_CYCLE_REAR = TUNED_REAR_RUN
ROBUST_REAR_RUN = replace(
    TUNED_REAR_RUN,
    posture_fl_force_n=-650.0,
    posture_hold_fl_force_n=-450.0,
)
COMPACT_REAR_RETURN_RAMP_S = 6.0


@dataclass(frozen=True)
class RightSideModelConfig:
    clamp_ride_tables: bool = True
    lateral_ride_scale: float = 0.5
    max_boundary_pit_width_m: float = 1.4
    max_start_offset_m: float = 0.4


RIGHT_SIDE_MODEL = RightSideModelConfig()


@dataclass(frozen=True)
class EnvironmentGainTransferConfig:
    min_trial_friction: float = 0.5
    max_trial_friction: float = 1.0
    max_start_station_delta_m: float = 0.8


ENVIRONMENT_GAIN_TRANSFER = EnvironmentGainTransferConfig()


def rear_speed_trial_config(target_kph: float, min_pit_kph: float,
                            base: RearCycleConfig = TUNED_REAR_RUN,
                            accel_ramp_s: float | None = None) -> RearCycleConfig:
    if (not math.isfinite(target_kph) or not math.isfinite(min_pit_kph) or
            not 0. < min_pit_kph <= target_kph <= base.max_crawl_speed_kph):
        raise ValueError("invalid rear crawl speed trial")
    if accel_ramp_s is not None and (
            not math.isfinite(accel_ramp_s) or
            accel_ramp_s < base.control_period_s):
        raise ValueError("invalid rear acceleration ramp")
    return replace(base, crawl_speed_kph=target_kph,
                   min_pit_speed_kph=min_pit_kph,
                   crawl_accel_ramp_s=(base.crawl_accel_ramp_s if
                                       accel_ramp_s is None else accel_ramp_s))


def speed_trial_config(target_kph: float, min_pit_kph: float,
                       accel_ramp_s: float | None = None,
                       *, base: ClosedLoopRunConfig = CLOSED_LOOP_RUN) -> ClosedLoopRunConfig:
    """Keep requested speed and native acceptance bound in one trial config."""
    if (not math.isfinite(target_kph) or not math.isfinite(min_pit_kph) or
            not 0. < min_pit_kph <= target_kph <= base.crawl_max_speed_kph):
        raise ValueError("invalid crawl speed trial")
    if accel_ramp_s is not None and (
            not math.isfinite(accel_ramp_s) or accel_ramp_s < 0.):
        raise ValueError("invalid front acceleration ramp")
    return replace(base, crawl_speed_kph=target_kph,
                   crawl_min_speed_kph=min_pit_kph,
                   crawl_accel_ramp_s=(base.crawl_accel_ramp_s if
                                       accel_ramp_s is None else accel_ramp_s))


@dataclass(frozen=True)
class ObservationNoiseConfig:
    # Simulation stress settings, not measured sensor specifications.
    seed: int = 20261001
    period_s: float = .02
    load_std_n: float = 20.
    lateral_std_m: float = .001
    attitude_std_deg: float = .02
    speed_std_kph: float = .01
    clip_sigma: float = 3.
    truth_load_cutoff_hz: float = 5.
    feedback_delay_s: float = 0.

    def __post_init__(self):
        if (not isinstance(self.seed, int) or self.seed < 0 or
                not all(math.isfinite(v) for v in (self.period_s, self.load_std_n,
                    self.lateral_std_m, self.attitude_std_deg, self.speed_std_kph,
                    self.clip_sigma, self.truth_load_cutoff_hz, self.feedback_delay_s)) or
                min(self.period_s, self.clip_sigma, self.truth_load_cutoff_hz) <= 0 or
                min(self.load_std_n, self.lateral_std_m,
                    self.attitude_std_deg, self.speed_std_kph, self.feedback_delay_s) < 0):
            raise ValueError('invalid noise configuration')


@dataclass(frozen=True)
class SupportForcePulseConfig:
    # Explicit simulation actuator-bias stress trial, not a hardware specification.
    amplitude_n: float = 200.
    phase: str = 'RR_HOLD'
    corner: str = 'FL'
    start_delay_s: float = .5
    ramp_s: float = .5
    hold_s: float = 1.
    recovery_release_s: float = SUPPORT_QP.release_s
    force_limit_n: float = SUPPORT_QP.force_limit_n

    def __post_init__(self):
        support = {'THREE_WHEEL_HOLD': ('FL', 'RL', 'RR'),
                   'RR_HOLD': ('FL', 'FR', 'RL')}
        if (self.phase not in support or self.corner not in support[self.phase] or
                not all(math.isfinite(x) for x in (self.amplitude_n, self.start_delay_s,
                    self.ramp_s, self.hold_s, self.recovery_release_s, self.force_limit_n)) or
                self.amplitude_n == 0. or abs(self.amplitude_n) > self.force_limit_n or
                min(self.start_delay_s, self.hold_s) < 0. or
                min(self.ramp_s, self.recovery_release_s, self.force_limit_n) <= 0.):
            raise ValueError('invalid support force pulse configuration')


@dataclass(frozen=True)
class PitAbortTrialConfig:
    trigger_fraction: float = .5
    max_observe_after_abort_s: float = 25.
    minimum_stopped_hold_s: float = 5.
    max_recovery_observe_s: float = 45.

    def __post_init__(self):
        if (not all(math.isfinite(x) for x in (self.trigger_fraction,
                self.max_observe_after_abort_s, self.minimum_stopped_hold_s,
                self.max_recovery_observe_s)) or
                not 0. < self.trigger_fraction < 1. or
                not 0. < self.minimum_stopped_hold_s < self.max_observe_after_abort_s or
                self.max_recovery_observe_s < self.max_observe_after_abort_s):
            raise ValueError('invalid pit abort trial configuration')
