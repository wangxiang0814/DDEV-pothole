"""Simulation-only limits for the I_I static FR preload gate."""

from dataclasses import dataclass

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
