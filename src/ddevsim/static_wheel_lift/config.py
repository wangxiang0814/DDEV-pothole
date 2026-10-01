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


@dataclass(frozen=True)
class RightSideModelConfig:
    clamp_ride_tables: bool = True
    lateral_ride_scale: float = 0.5
    max_boundary_pit_width_m: float = 1.4
    max_start_offset_m: float = 0.4


RIGHT_SIDE_MODEL = RightSideModelConfig()


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
