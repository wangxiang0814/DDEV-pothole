"""Native TruckSim FR crossing followed by RR crossing and four-wheel return."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import re
from dataclasses import asdict, replace
from pathlib import Path

import numpy as np

from . import front as front_run

from ddevsim.cosim import run_stepwise
from ddevsim.pothole_case import corner_module_scenario
from ddevsim.scenarios.approach import approach_distance
from ddevsim.visual_mesh import write_visual_mesh
from ddevsim.static_wheel_lift.config import (REAR_CYCLE_RUN,
                                             RIGHT_SIDE_MODEL, TUNED_REAR_RUN,
                                             FAST_CYCLE_FRONT, FAST_CYCLE_REAR,
                                             BALANCED_CYCLE_FRONT, CLOSED_LOOP_RUN,
                                             COMPACT_CYCLE_FRONT, EFFICIENT_CYCLE_FRONT, COMPACT_REAR_RETURN_RAMP_S,
                                             RESPONSIVE_CYCLE_FRONT, RESPONSIVE_REAR_TIMING, RESPONSIVE_STOP_GAP_M,
                                             ROBUST_REAR_RUN,
                                             rear_speed_trial_config)
from ddevsim.static_wheel_lift.rear_cycle import RearCycleController
from ddevsim.static_wheel_lift.observation_noise import (
    ObservationNoiseConfig, NoisyFeedbackTrial)
from ddevsim.static_wheel_lift.system_identification import validated_contact_gains, validated_support_models
from ddevsim.static_wheel_lift.support_allocation_feedback import SupportAllocationFeedback
from ddevsim.static_wheel_lift.path_reference import StaticPathReference
from ddevsim.static_wheel_lift.config import PATH_REFERENCE
from ddevsim.static_wheel_lift.config import FEEDBACK_HEALTH
from ddevsim.static_wheel_lift.force_disturbance import SupportForcePulse
from ddevsim.static_wheel_lift.config import SupportForcePulseConfig
from ddevsim.static_wheel_lift.pit_scene_transfer import (
    replace_pit_geometry_parameters, validated_pit_scene_models)
from ddevsim.static_wheel_lift.environment_gain_transfer import validated_environment_models

STEERING_IMPORT = "IMP_STEER_SW"



from .cycle import FullRightSideController
from .lift_scene import LiftScene, SideMirror, length_adjusted_crawl_timing
from ddevsim.vehicle_profiles.tuned_ii import CENTERED_PAYLOAD_PROFILE_ID, center_rear_payload, vehicle_identity
from .acceptance import evaluate_full_cycle
from .model_builder import (_enable_steering_import, _clamp_kinematics_at_original_bounds,
                            _scale_lateral_ride_movement, _set_pit_geometry, _set_vehicle_start_offset,
                            set_approach_start)

def _write_rows(path: Path, rows: list[dict]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demonstration-config', type=Path)
    parser.add_argument('--claimed-output', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--efficiency-profile", choices=("baseline", "balanced", "fast", "compact", "efficient", "responsive"),
                        default="baseline")
    parser.add_argument("--identify-rr-swing", action="store_true")
    parser.add_argument('--contact-gain-bundle', type=Path,
                        help='Optional native mode gains; exact selected model hash must match.')
    parser.add_argument('--support-allocation', choices=('off', 'monitor', 'active'), default='off')
    parser.add_argument('--support-allocation-gains', type=Path)
    parser.add_argument('--support-allocation-reference-model', type=Path,
                        help='Explicit source run_all.par for strictly verified pit width/depth gain reuse.')
    parser.add_argument('--support-allocation-environment-reference',type=Path,
                        help='Explicit source model for environment-only robustness trials; no vehicle change allowed.')
    parser.add_argument('--support-allocation-transitions', action='store_true')
    parser.add_argument('--rear-abort-exit', action='store_true',
                        help='Optional gated RR over-pit exit; requires active support allocation.')
    parser.add_argument('--average-path-reference', action='store_true')
    parser.add_argument("--steer-probe-deg", type=float)
    parser.add_argument("--stop-at", type=float)
    parser.add_argument("--original-kinematics", action="store_true",
                        help="diagnostic: preserve the source I_I ride tables")
    parser.add_argument("--lateral-ride-scale", type=float,
                        default=RIGHT_SIDE_MODEL.lateral_ride_scale,
                        help="copied suspension lateral movement scale, 0..1; default 0.5")
    parser.add_argument("--rear-target-speed-kph", type=float)
    parser.add_argument("--rear-min-pit-speed-kph", type=float)
    parser.add_argument("--rear-accel-ramp-s", type=float)
    parser.add_argument("--rear-preload-fl-force-n", type=float)
    parser.add_argument("--rear-preload-fr-force-n", type=float)
    parser.add_argument("--rear-preload-rl-force-n", type=float)
    parser.add_argument("--rear-adaptive-preload", action="store_true")
    parser.add_argument("--rear-sequential-preload", action="store_true")
    parser.add_argument("--rear-static-steering", action="store_true")
    parser.add_argument("--front-steering-feedback", action="store_true")
    parser.add_argument("--measurement-noise", action="store_true",
                        help="deterministic sensor noise; acceptance uses plant truth")
    parser.add_argument("--support-force-pulse-n", type=float,
                        help="finite unknown support-actuator force bias, signed N")
    parser.add_argument("--support-force-pulse-stage", choices=('FR', 'RR'), default='RR')
    parser.add_argument("--feedback-delay-s", type=float, default=0.,
                        help="sampled observation packet delay; requires --measurement-noise")
    parser.add_argument("--noise-seed", type=int, default=ObservationNoiseConfig().seed)
    parser.add_argument("--rear-parking-damping-nm-per-rpm", type=float)
    parser.add_argument("--rear-lift-force-n", type=float)
    parser.add_argument("--rear-posture-fl-force-n", type=float)
    parser.add_argument("--rear-control-profile", choices=("baseline", "robust"),
                        default="baseline")
    parser.add_argument("--rear-posture-hold-fl-force-n", type=float)
    parser.add_argument("--front-target-speed-kph", type=float)
    parser.add_argument("--front-min-pit-speed-kph", type=float)
    parser.add_argument("--front-accel-ramp-s", type=float)
    parser.add_argument("--front-preload-reference-rate", type=float,
                        help="maximum feedback-governed preload reference speed; baseline unchanged")
    parser.add_argument("--front-lift-force-n", type=float)
    parser.add_argument("--front-support-feedback-limit-n", type=float)
    parser.add_argument("--front-brake-lead-m", type=float)
    parser.add_argument("--front-stop-ramp-s", type=float,
                        help="explicit bounded FR stopping-reference trial; acceptance unchanged")
    parser.add_argument("--front-stop-support-slew-n-s", type=float)
    parser.add_argument("--pit-width-m", type=float)
    parser.add_argument("--pit-depth-m", type=float)
    parser.add_argument("--road-friction", type=float)
    parser.add_argument("--vehicle-start-offset-m", type=float, default=0.0)
    parser.add_argument('--approach-distance-m', type=float,
                        help='drive from this distance before the pit to the original lift station')
    parser.add_argument('--lift-scene', type=Path, help='single-track scene JSON in integer centimetres')
    parser.add_argument('--centered-payload-trial', action='store_true',
                        help='run a centered I_I development trial without legacy support QP gains')
    args = parser.parse_args(argv)
    lift_scene = None
    if args.lift_scene is not None:
        if args.approach_distance_m is None or args.pit_width_m is not None or args.pit_depth_m is not None:
            parser.error('--lift-scene requires fixed-origin approach and owns pit geometry')
        try:
            lift_scene = LiftScene.read(args.lift_scene)
        except (OSError, TypeError, ValueError) as exc:
            parser.error(str(exc))
        if lift_scene.side == 'left' and args.demonstration_config:
            parser.error('left-side native logs are supported; legacy capture requires phase-label migration')
    responsive = args.efficiency_profile == 'responsive'
    if responsive and args.approach_distance_m is None:
        parser.error('responsive cycle requires a fixed-origin approach')
    if args.approach_distance_m is not None:
        try:
            approach_distance(args.approach_distance_m)
        except ValueError as exc:
            parser.error(str(exc))
        if args.identify_rr_swing or args.contact_gain_bundle:
            parser.error('explicit identification or exact contact-gain runs use the original start')
    if args.centered_payload_trial:
        if lift_scene is None or args.support_allocation != 'off' or args.support_allocation_gains is not None:
            parser.error('centered payload trial requires an explicit lift scene and no legacy support QP gains')
        if args.demonstration_config or getattr(args, 'dynamics_episode_id', None):
            parser.error('centered payload trial is not released for demonstration or dynamics collection')
    capture_enabled = False
    if args.demonstration_config:
        from ddevsim.demonstration.cameras import load_config
        capture_enabled = load_config(args.demonstration_config)['enabled']
    if args.support_allocation_environment_reference is not None:
        if args.support_allocation_reference_model is not None or args.support_allocation_gains is None:
            parser.error('environment reference requires gains and cannot combine with pit-scene transfer')
    disturbance = None
    if args.support_force_pulse_n is not None:
        try:
            disturbance = SupportForcePulse(SupportForcePulseConfig(
                amplitude_n=args.support_force_pulse_n,
                phase='THREE_WHEEL_HOLD' if args.support_force_pulse_stage == 'FR' else 'RR_HOLD',
                corner='RL' if args.support_force_pulse_stage == 'FR' else 'FL'))
        except ValueError as exc:
            parser.error(str(exc))
    if (args.support_allocation != 'off') != (args.support_allocation_gains is not None):
        parser.error('support allocation requires explicit gain bundle; off mode takes no bundle')
    if args.support_allocation_transitions and args.support_allocation == 'off':
        parser.error('transition allocation requires active or monitor support allocation')
    if not np.isfinite(args.lateral_ride_scale) or not 0. <= args.lateral_ride_scale <= 1.:
        parser.error("lateral ride scale must be finite and in 0..1")
    if (not np.isfinite(args.feedback_delay_s) or args.feedback_delay_s < 0. or
            (args.feedback_delay_s > 0. and not args.measurement_noise)):
        parser.error("feedback delay must be finite, nonnegative and requires --measurement-noise")
    if args.noise_seed < 0:
        parser.error("noise seed must be nonnegative")
    if ((args.rear_target_speed_kph is None) !=
            (args.rear_min_pit_speed_kph is None)):
        parser.error("rear target and minimum pit speed must be supplied together")
    if ((args.front_target_speed_kph is None) !=
            (args.front_min_pit_speed_kph is None)):
        parser.error("front target and minimum pit speed must be supplied together")
    front_config = {
        "baseline": CLOSED_LOOP_RUN,
        "balanced": BALANCED_CYCLE_FRONT,
        "fast": FAST_CYCLE_FRONT,
        "compact": COMPACT_CYCLE_FRONT,
        "efficient": EFFICIENT_CYCLE_FRONT,
        "responsive": RESPONSIVE_CYCLE_FRONT,
    }[args.efficiency_profile]
    if args.front_preload_reference_rate is not None:
        if (not np.isfinite(args.front_preload_reference_rate) or
                args.front_preload_reference_rate < 1.):
            parser.error("preload rate must be finite and at least 1")
        front_config = replace(front_config,
            preload_max_reference_rate=args.front_preload_reference_rate)
    if args.front_target_speed_kph is not None:
        front_config = front_run.speed_trial_config(
            args.front_target_speed_kph, args.front_min_pit_speed_kph,
            args.front_accel_ramp_s, base=front_config)
    elif args.front_accel_ramp_s is not None:
        parser.error("front acceleration ramp requires a front speed trial")
    if args.front_lift_force_n is not None:
        if (not np.isfinite(args.front_lift_force_n) or
                not -front_config.max_lift_trial_force_n <=
                args.front_lift_force_n < 0.0):
            parser.error("front lift force must be negative and within the configured simulation trial range")
        front_config = replace(front_config,
                                fr_lift_force_n=args.front_lift_force_n)
    if args.front_support_feedback_limit_n is not None:
        if (not np.isfinite(args.front_support_feedback_limit_n) or
                not 0.0 < args.front_support_feedback_limit_n <=
                front_config.max_support_trial_feedback_n):
            parser.error("front support feedback limit exceeds the simulation trial range")
        front_config = replace(front_config,
                                support_feedback_limit_n=args.front_support_feedback_limit_n)
    if args.front_stop_support_slew_n_s is not None:
        if (not np.isfinite(args.front_stop_support_slew_n_s) or
                not 0.0 < args.front_stop_support_slew_n_s <= front_config.max_stop_support_trial_slew_n_s):
            parser.error("front stop support slew exceeds configured trial range")
        front_config = replace(front_config, stop_support_slew_n_s=args.front_stop_support_slew_n_s)
    if args.front_stop_ramp_s is not None:
        if (not np.isfinite(args.front_stop_ramp_s) or
                not 0.0 < args.front_stop_ramp_s <= front_config.max_stop_trial_ramp_s):
            parser.error("front stop ramp must be positive, finite and within configured trial range")
        front_config = replace(front_config, stop_ramp_s=args.front_stop_ramp_s)
    if args.front_brake_lead_m is not None:
        if not np.isfinite(args.front_brake_lead_m) or args.front_brake_lead_m < 0.0:
            parser.error("front brake lead must be nonnegative and finite")
        front_config = replace(
            front_config,
            brake_start_before_far_edge_m=args.front_brake_lead_m)
    output = args.output.resolve()
    model, scenario = front_run._prepare_model(
        output, crawl=True, road_friction=args.road_friction, config=front_config,
        claimed_output=args.claimed_output)
    if (not np.isfinite(args.vehicle_start_offset_m) or
            abs(args.vehicle_start_offset_m) > RIGHT_SIDE_MODEL.max_start_offset_m):
        parser.error("vehicle start offset exceeds the configured trial range")
    if args.vehicle_start_offset_m:
        _set_vehicle_start_offset(model, scenario, args.vehicle_start_offset_m)
    if (args.front_brake_lead_m is not None and
            args.front_brake_lead_m >= float(scenario["length_m"])):
        parser.error("front brake lead must be shorter than pit length")
    if args.pit_width_m is not None or args.pit_depth_m is not None:
        _set_pit_geometry(model, scenario,
                          width_m=(args.pit_width_m if args.pit_width_m is not None
                                   else float(scenario["width_m"])),
                          depth_m=(args.pit_depth_m if args.pit_depth_m is not None
                                   else float(scenario["depth_m"])))
    if not args.original_kinematics and RIGHT_SIDE_MODEL.clamp_ride_tables:
        _clamp_kinematics_at_original_bounds(model)
    if not args.original_kinematics:
        _scale_lateral_ride_movement(model, args.lateral_ride_scale)
    _enable_steering_import(model)
    path = model / "run_all.par"
    content, count = re.subn(r"(?m)^TSTOP\s+[-+0-9.eE]+\s*$",
                             "TSTOP 115" if args.identify_rr_swing else "TSTOP 180",
                             path.read_text(encoding="utf-8"))
    if count != 2:
        raise ValueError("expected two TruckSim stop times")
    path.write_text(content, encoding="utf-8")
    gain = np.array(json.loads((front_run.EVIDENCE / "gain_matrix.json").read_text(
        encoding="utf-8"))["gains"]["Fz_n"], dtype=float)
    contact_gains, gain_provenance = None, None
    if args.contact_gain_bundle is not None:
        bundle_path = args.contact_gain_bundle.resolve()
        bundle = json.loads(bundle_path.read_text(encoding='utf-8'))
        model_hash = hashlib.sha256(path.read_bytes()).hexdigest()
        contact_gains = validated_contact_gains(bundle, model_sha256=model_hash)
        gain = contact_gains['FOUR_CONTACT']
        gain_provenance = {'bundle': str(bundle_path),
                           'bundle_sha256': hashlib.sha256(bundle_path.read_bytes()).hexdigest(),
                           'model_sha256': model_hash,
                           'scope': 'four-contact/FR local load feedback; RR swing gain reserved for allocation'}
    rear_config = (REAR_CYCLE_RUN if args.original_kinematics else
                   ROBUST_REAR_RUN if args.rear_control_profile == "robust" else
                   FAST_CYCLE_REAR if args.efficiency_profile == "fast" else
                   TUNED_REAR_RUN)
    if args.efficiency_profile in ("compact", "efficient"):
        rear_config = replace(rear_config, return_ramp_s=COMPACT_REAR_RETURN_RAMP_S)
    elif responsive:
        rear_config = replace(rear_config, **RESPONSIVE_REAR_TIMING)
    if args.rear_adaptive_preload:
        rear_config = replace(rear_config, adaptive_preload=True,
                              preload_ready_timeout_s=rear_config.adaptive_preload_timeout_s)
    if args.rear_sequential_preload:
        rear_config = replace(rear_config, sequential_preload=True)
    if args.rear_static_steering:
        rear_config = replace(rear_config, static_steering_feedback=True)
    if args.rear_parking_damping_nm_per_rpm is not None:
        if (not np.isfinite(args.rear_parking_damping_nm_per_rpm) or
                not 0. <= args.rear_parking_damping_nm_per_rpm <=
                rear_config.parking_limit_nm):
            parser.error("invalid rear parking damping trial")
        rear_config = replace(rear_config,
                              parking_damping_nm_per_rpm=args.rear_parking_damping_nm_per_rpm)
    if args.rear_preload_rl_force_n is not None:
        if (not np.isfinite(args.rear_preload_rl_force_n) or
                not 0. <= args.rear_preload_rl_force_n <= rear_config.sim_force_limit_n):
            parser.error("rear RL preload must be nonnegative and within simulation force limits")
        rear_config = replace(rear_config, preload_rl_force_n=args.rear_preload_rl_force_n)
    if args.rear_accel_ramp_s is not None and args.rear_target_speed_kph is None:
        parser.error("rear acceleration ramp requires a rear speed trial")
    if args.rear_target_speed_kph is not None:
        rear_config = rear_speed_trial_config(
            args.rear_target_speed_kph, args.rear_min_pit_speed_kph,
            rear_config, args.rear_accel_ramp_s)
    for value, label, sign in (
            (args.rear_preload_fl_force_n, "FL", -1),
            (args.rear_preload_fr_force_n, "FR", 1)):
        if value is not None and (
                not np.isfinite(value) or sign * value <= 0.0 or
                abs(value) > rear_config.sim_force_limit_n):
            parser.error(f"rear preload {label} force has invalid sign or exceeds the simulation limit")
    rear_config = replace(
        rear_config,
        preload_fl_force_n=(rear_config.preload_fl_force_n if
                            args.rear_preload_fl_force_n is None else
                            args.rear_preload_fl_force_n),
        preload_fr_force_n=(rear_config.preload_fr_force_n if
                            args.rear_preload_fr_force_n is None else
                            args.rear_preload_fr_force_n))
    if args.rear_lift_force_n is not None:
        if (not np.isfinite(args.rear_lift_force_n) or
                not -rear_config.max_lift_trial_force_n <=
                args.rear_lift_force_n < 0.0):
            parser.error("rear lift force must be negative and within the configured trial range")
        rear_config = replace(rear_config, lift_force_n=args.rear_lift_force_n)
    posture_force = (rear_config.posture_fl_force_n if
                     args.rear_posture_fl_force_n is None else
                     args.rear_posture_fl_force_n)
    posture_hold = (rear_config.posture_hold_fl_force_n if
                    args.rear_posture_hold_fl_force_n is None else
                    args.rear_posture_hold_fl_force_n)
    if (not np.isfinite(posture_force) or not np.isfinite(posture_hold) or
            not -rear_config.max_posture_trial_force_n <= posture_force < 0.0 or
            not posture_force <= posture_hold < 0.0):
        parser.error("rear posture force must be negative and hold force no stronger than its ramp")
    rear_config = replace(rear_config, posture_fl_force_n=posture_force,
                          posture_hold_fl_force_n=posture_hold)
    if args.rear_abort_exit:
        if args.support_allocation != 'active' or args.support_allocation_gains is None:
            parser.error('rear abort exit requires active support allocation and gains')
        rear_config = replace(rear_config, allow_abort_exit=True)
    if responsive and lift_scene is not None:
        brake_lead, rear_ramp, support_feedback = length_adjusted_crawl_timing(
            lift_scene, front_config.brake_start_before_far_edge_m,
            rear_config.crawl_accel_ramp_s, front_config.support_feedback_limit_n)
        front_config = replace(front_config, brake_start_before_far_edge_m=brake_lead,
                               support_feedback_limit_n=support_feedback)
        rear_config = replace(rear_config, crawl_accel_ramp_s=rear_ramp)
    allocation, allocation_provenance = None, None
    if args.support_allocation_reference_model is not None and args.support_allocation_gains is None:
        parser.error('scene gain reuse requires --support-allocation-gains')
    if args.support_allocation_gains is not None:
        allocation_path = args.support_allocation_gains.resolve()
        bundle = json.loads(allocation_path.read_text(encoding='utf-8'))
        model_hash = hashlib.sha256(path.read_bytes()).hexdigest()
        transfer = None
        if args.support_allocation_environment_reference is not None:
            models,transfer=validated_environment_models(bundle,
                args.support_allocation_environment_reference.resolve(),path)
        elif args.support_allocation_reference_model is None:
            models = validated_support_models(bundle, model_sha256=model_hash)
        else:
            variant = corner_module_scenario(width_m=scenario['width_m'],
                depth_m=scenario['depth_m'], target_speed_kph=float(scenario['target_speed_kph']))
            models, transfer = validated_pit_scene_models(bundle,
                args.support_allocation_reference_model.resolve(), path, variant)
        allocation = SupportAllocationFeedback(mode_models=models,
            active=args.support_allocation == 'active', period_s=front_config.control_period_s,
            front_limits=front_config, rear_limits=rear_config,
            transitions=args.support_allocation_transitions)
        allocation_provenance = {'bundle_sha256': hashlib.sha256(allocation_path.read_bytes()).hexdigest(),
                                 'model_sha256': model_hash, 'mode': args.support_allocation}
        if transfer is not None:
            allocation_provenance['scene_transfer'] = transfer
    if args.approach_distance_m is not None:
        # Validate the existing gain model first. Only the scene is transformed
        # to the shared fixed origin. Gains remain inactive on approach.
        gain_model_friction = scenario['friction']
        set_approach_start(model, scenario, args.approach_distance_m,
                           stop_gap_m=RESPONSIVE_STOP_GAP_M if responsive else None, lift_scene=lift_scene)
        if allocation_provenance is not None:
            allocation_provenance['approach_start'] = {
                'simulation_model_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                'gain_model_friction': gain_model_friction,
                'simulation_road_friction': scenario['friction'],
                'scene_transform': 'fixed origin, translated pit and road markers, common dry road',
                'gain_activation': 'after approach and stationary settle',
            }
    centered_provenance = None
    if args.centered_payload_trial:
        original_sha = hashlib.sha256(path.read_bytes()).hexdigest()
        path.write_text(center_rear_payload(path.read_text(encoding='utf-8')), encoding='utf-8')
        centered_provenance = dict(profile_id=CENTERED_PAYLOAD_PROFILE_ID,
                                   original_model_sha256=original_sha,
                                   model_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                                   vehicle_sha256=vehicle_identity(path),
                                   experimental=True,
                                   feedback_seed='legacy reference and base scalar/support feedback; not reidentified')
    controller = FullRightSideController(
        scenario=scenario,
        rr_gain_per_coupled_force=float(
            gain[3, 1] + REAR_CYCLE_RUN.preload_feedback_fl_per_fr * gain[3, 0]),
        identify_rr_swing=args.identify_rr_swing,
        steer_probe_deg=args.steer_probe_deg,
        front_steering_feedback=args.front_steering_feedback,
        contact_gains=contact_gains,
        allocation=allocation,
        disturbance=disturbance,
        path_reference=StaticPathReference() if args.average_path_reference else None,
        rear_config=rear_config, front_config=front_config)
    noise_config = ObservationNoiseConfig(seed=args.noise_seed,
                                         period_s=front_config.control_period_s,
                                         feedback_delay_s=args.feedback_delay_s)
    trial = (NoisyFeedbackTrial(controller, front_run.EXPORTS, front_config,
                               noise_config) if args.measurement_noise else None)
    command = trial or controller
    left_side = lift_scene is not None and lift_scene.side == 'left'
    if left_side:
        command = SideMirror(front_run.EXPORTS).wrap(command)
    native = run_stepwise(model / "simfile.sim", command,
                          output / "native_5ms.csv",
                          (*front_run.IMPORT_NAMES, STEERING_IMPORT),
                          front_run.EXPORTS, log_decimation=10,
                          demonstration_dir=(output / 'demonstration' if capture_enabled else None),
                          stop_at_s=args.stop_at,
                          stop_when=(lambda now_s: (
                              (controller.approach is not None and controller.approach.failed) or
                              (controller.front.mode == "COMPLETE" and
                               controller.front.abort_reason is not None and
                               now_s - controller.front.lower_start_s >=
                               front_config.lower_ramp_s + front_config.return_ramp_s +
                               rear_config.post_complete_observe_s) or (
                              controller.rear.mode == "RR_COMPLETE" and
                              controller.rear.mode_start_s is not None and
                              now_s - controller.rear.mode_start_s >=
                              rear_config.post_complete_observe_s))))
    front_rows = trial.front_truth.rows if trial else controller.front.rows
    rear_rows = trial.rear_truth.rows if trial else controller.rear.rows
    if trial:
        _write_rows(output / "front_observed_20ms.csv", controller.front.rows)
        _write_rows(output / "rear_observed_20ms.csv", controller.rear.rows)
    if controller.approach is not None:
        _write_rows(output / 'approach_control_20ms.csv', controller.approach.rows)
    _write_rows(output / "front_control_20ms.csv", front_rows)
    _write_rows(output / "rear_control_20ms.csv", rear_rows)
    path_yo = trial.path_reference_yo_m if trial else controller.front.initial_yo_m
    path_yaw = trial.path_reference_yaw_deg if trial else controller.front.initial_yaw_deg
    front_result = front_run.evaluate_control_rows(
        front_rows, crawl=True, scenario=scenario,
        native_completed=native["status"] == "COMPLETED",
        abort_reason=controller.front.abort_reason,
        path_reference_yo_m=path_yo,
        path_reference_yaw_deg=path_yaw, config=front_config)
    result = evaluate_full_cycle(front_result, controller.rear, scenario,
                                 native["status"] == "COMPLETED",
                                 truth_rows=rear_rows,
                                 path_reference_yo_m=path_yo,
                                 path_reference_yaw_deg=path_yaw)
    if controller.approach is not None:
        report = controller.approach.summary()
        result['approach'] = {**report, 'config': asdict(controller.approach.config)}
        result['criteria'].update(report['criteria'])
        if not all(report['criteria'].values()):
            result['status'] = 'FAIL'
    if disturbance is not None:
        report = disturbance.summary()
        result['force_disturbance'] = report
        passed = report['delivered']
        result['criteria']['disturbance_trial_delivered'] = passed
        if not passed:
            result['status'] = 'FAIL'
    result["fr_abort_reason"] = controller.front.abort_reason
    result["measurement_noise"] = {
        "enabled": args.measurement_noise, "config": asdict(noise_config),
        "acceptance_source": "plant_truth",
        "control_path_reference_yo_m": controller.front.initial_yo_m,
        "control_path_reference_yaw_deg": controller.front.initial_yaw_deg,
        "load_observation_floor_n": 0.0,
        "ideal_channels": "contact geometry, CoM, clearance, rates, travel, wheel speed",
        "delay_scope": "all exported observation channels; controller timestamp remains current",
        "delay_sampling": "control-period packets, causal hold, first packet during warmup",
        "maximum_logged_age_s": max((r.get("measurement_age_s", 0.)
                                       for r in front_rows + rear_rows), default=0.),
    }
    result["model_tuning"] = {
        "original_kinematics": args.original_kinematics,
        "clamp_ride_tables": (not args.original_kinematics and
                              RIGHT_SIDE_MODEL.clamp_ride_tables),
        "lateral_ride_scale": (1.0 if args.original_kinematics else
                               args.lateral_ride_scale),
        "steering_import": STEERING_IMPORT,
    }
    result["efficiency_profile"] = args.efficiency_profile
    result["rear_control_profile"] = args.rear_control_profile
    result["front_steering_feedback"] = args.front_steering_feedback
    result['path_reference_estimation'] = {
        'averaged': args.average_path_reference,
        'config': asdict(PATH_REFERENCE),
        'sample_count': controller.path_reference.sample_count if controller.path_reference else 1,
        'scope': 'INIT_SETTLE trailing window; freeze on existing settle-to-preload transition; shared FR/RR reference',
    }
    result['feedback_health_config'] = asdict(FEEDBACK_HEALTH)
    result["controller_config"] = {
        "front": asdict(front_config), "rear": asdict(rear_config),
    }
    result['identified_gain_provenance'] = gain_provenance
    if centered_provenance is not None:
        result['vehicle_profile'] = centered_provenance
    if allocation is not None:
        result['support_allocation'] = {**allocation.summary(), 'config': asdict(allocation.config),
                                         'provenance': allocation_provenance}
    result["front_timing_config"] = {
        "init_settle_s": front_config.init_settle_s,
        "preload_max_reference_rate": front_config.preload_max_reference_rate,
        "lift_ramp_s": front_config.lift_ramp_s,
        "hold_s": front_config.three_wheel_hold_s,
        "lower_ramp_s": front_config.lower_ramp_s,
        "return_ramp_s": front_config.return_ramp_s,
    }
    result["scenario"] = {key: scenario[key] for key in
                          ("start_station_m", "length_m", "width_m", "depth_m",
                           "friction")}
    result["scenario"]["vehicle_start_offset_m"] = args.vehicle_start_offset_m
    result["scenario"]["approach_distance_m"] = scenario["approach_distance_m"]
    if lift_scene is not None:
        result['scenario'].update(side=lift_scene.side, center_y_m=lift_scene.center_y_m,
                                  wheel_sequence=lift_scene.wheel_sequence)
        result['diagnostic_coordinates'] = 'right-side canonical' if left_side else 'physical'
    if controller.approach is not None:
        for key in ('vehicle_start_station_m', 'pre_lift_stop_station_m'):
            result['scenario'][key] = scenario[key]
    result["rear_speed_config"] = {
        "target_kph": rear_config.crawl_speed_kph,
        "minimum_pit_kph": rear_config.min_pit_speed_kph,
        "accel_ramp_s": rear_config.crawl_accel_ramp_s,
        "preload_fl_force_n": rear_config.preload_fl_force_n,
        "preload_fr_force_n": rear_config.preload_fr_force_n,
        "preload_rl_force_n": rear_config.preload_rl_force_n,
        "lift_force_n": rear_config.lift_force_n,
        "posture_fl_force_n": rear_config.posture_fl_force_n,
        "posture_hold_fl_force_n": rear_config.posture_hold_fl_force_n,
    }
    result["front_speed_config"] = {
        "target_kph": front_config.crawl_speed_kph,
        "minimum_pit_kph": front_config.crawl_min_speed_kph,
        "accel_ramp_s": front_config.crawl_accel_ramp_s,
        "lift_force_n": front_config.fr_lift_force_n,
        "support_feedback_limit_n": front_config.support_feedback_limit_n,
        "brake_lead_m": front_config.brake_start_before_far_edge_m,
        "stop_ramp_s": front_config.stop_ramp_s,
        "stop_support_slew_n_s": front_config.stop_support_slew_n_s,
    }
    result["front"] = front_result
    result["native"] = native
    (output / "result.json").write_text(json.dumps(result, indent=2),
                                        encoding="utf-8")
    print(json.dumps({k: v for k, v in result.items() if k != "native"},
                     indent=2))


