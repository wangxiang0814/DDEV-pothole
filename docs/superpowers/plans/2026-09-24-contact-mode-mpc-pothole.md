# Sequential Wheel-Lift Contact-Mode MPC Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** On the frozen TruckSim corner-module model, determine whether a bounded, stable FR→four-contact→RR lift over the 0.20 m right-track pit is reachable, then build and validate a classical scheduled-contact controller only if each prerequisite passes.

**Architecture:** Native TruckSim provides the eight independent actuator ports and 0.5 ms dynamics. A frozen run contract, contact/CG estimator, bounded reachability experiments, held-out gray-box predictor, discrete scheduler, sequentially linearized suspension MPC, torque/yaw controller and independent safety supervisor form a gated pipeline. A failed reachability search is reported as unresolved, not as a proof that the plant is physically incapable.

**Tech Stack:** Windows, TruckSim 2019 VS Solver API, Python 3.8.5, NumPy 1.24.4, SciPy 1.10.1 (installed), pytest, CSV/JSON. Start with installed `scipy.optimize.minimize(method="SLSQP")` for the small constrained quadratic subproblems; it is permitted only if the measured timing and feasibility gates pass. No new solver package is assumed.

**Spec:** `docs/superpowers/specs/2026-09-24-contact-mode-mpc-pothole-design.md`

## Global Constraints

- Primary model is `models/corner_module_ddev/run_all.par`, SHA-256 `a5bf133c1b7312c897861e2da45ab9d7f887a5e765b78d4b8ff201775ecb2f73`; re-baseline and stop if bytes differ.
- Scenario is `models/corner_module_ddev/single_wheel_deep_pothole/scenario.json`: station 101.1 m, 0.80 m length, 0.90 m width, 0.20 m depth, friction 0.7, 2.8 km/h.
- Current model is the approximately 1.36 t independent-suspension `I_I` utility truck, **not** a calibrated heavy-duty truck; keep its mass, passive springs, tyres and +160/−100 mm travel unchanged for primary A/B trials.
- Input order is `IMP_MYUSM_L1/R1/L2/R2`, then `IMP_FS_L1/R1/L2/R2` (FL/FR/RL/RR). `IMP_FS` is additive spring-seat force, not a ball-screw motor model. Record requested and `FsExt` applied forces.
- Use unique run directories. Preserve all existing dirty-worktree files and native histories. Never overwrite `runs/corner_module_expert_pothole` or relabel an old run.
- Terrain preview is simulator truth. Record its provenance; do not claim perception-robust operation.
- Safety constraints dominate speed tracking. An optimizer candidate using safety slack, a stale command after failed solve, or a missed contact guard is never sent to TruckSim.
- Final success requires the frozen native run to pass every pre-registered physical criterion; unit tests, `DONE`, `safe_stop=False` and video alone do not establish success.

## Review Focus

- Plant hash or controller force authority changes mid-experiment → refuse reuse of identification or comparison data (Task 1 test).
- Wheel load becomes zero while tyre is at the pit floor → clearance test rejects apparent lift (Tasks 2 and 7 tests).
- A support tyre lifts or a contact event happens outside the scheduled mode → supervisor enters RECOVER/STOP (Tasks 4 and 5 tests).
- Four-contact PRELOAD starts with CG on the future triangle edge → permit active posture trim, but never waive the measured three-contact guard once target lift begins (Task 5 test).
- QP returns success after the deadline or with nonlinear clearance/CG violation → reject its action and issue a typed safe fallback (Task 6 test).

## File map and boundaries

| Responsibility | Files |
|---|---|
| Frozen identities, units and required observations | Create `src/ddevsim/experiment_contract.py`, `tests/test_experiment_contract.py`; use `src/ddevsim/interface_validation.py`, `src/ddevsim/plant_mode.py` |
| Tyre envelope, full-CG uncertainty and dynamic risk | Create `src/ddevsim/contact_geometry.py`, `tests/test_contact_geometry.py`; calibrate against `src/ddevsim/support_geometry.py` |
| Bounded native reachability and event-resolved probes | Create `src/ddevsim/reachability.py`, `scripts/probe_lift_reachability.py`, `tests/test_reachability.py`; reuse `src/ddevsim/cosim.py` |
| Hybrid predictor and held-out qualification | Create `src/ddevsim/contact_predictor.py`, `scripts/fit_contact_predictor.py`, `tests/test_contact_predictor.py` |
| Discrete progression and independent safety | Create `src/ddevsim/lift_scheduler.py`, `src/ddevsim/lift_safety.py`, their tests; keep legacy `src/ddevsim/expert_controller.py` as untouched A/B baseline |
| Constrained force plan and eight-input integration | Create `src/ddevsim/lift_mpc.py`, `src/ddevsim/lift_control.py`, their tests |
| Native acceptance, CLI and A/B artifacts | Extend `src/ddevsim/traversal_metrics.py`, `scripts/run_expert_pothole.py`; create `scripts/report_contact_mpc.py`, matching tests |

Do not add the new controller as another long branch inside the already ~2,000-line `expert_controller.py`. The new modules expose typed boundaries; the old controller remains a reproducible comparator.

---

### Task 1: Freeze the exact case and validate observability

**Files:** Create `src/ddevsim/experiment_contract.py`, `tests/test_experiment_contract.py`; extend `scripts/probe_actuator_gain.py` only to accept `--log-decimation 10` or smaller and an isolated output directory. Do not mutate generated vehicle files.

**Interfaces:** `freeze_contract(run_all: Path, scenario: Path, manifest: Mapping[str, object], import_names: Sequence[str], export_names: Sequence[str]) -> dict` raises `ValueError` on a model hash or missing-channel mismatch and returns both hashes, ordered channels and units. `require_contract(data: Mapping[str, object], expected: Mapping[str, object]) -> None` prevents stale data reuse.

- [ ] **Step 1: Red test.** Add a test with a deliberately changed `run_all` byte and a test with missing `FsExt_R2`; each must raise `ValueError` before solver launch.
  ```python
  with pytest.raises(ValueError, match="plant hash"):
      freeze_contract(par, scene, {"generated_sha256": "0" * 64}, IMPORT_NAMES, exports)
  with pytest.raises(ValueError, match="FsExt_R2"):
      freeze_contract(par, scene, good_manifest, IMPORT_NAMES,
                      tuple(name for name in exports if name != "FsExt_R2"))
  ```
- [ ] **Step 2: Confirm red.** Run `python -m pytest tests/test_experiment_contract.py -q`; expected failure is an unresolved import/function, not a TruckSim error.
- [ ] **Step 3: Implement minimum.** Compute SHA-256 over bytes, check the manifest hash, assert all eight inputs and the required `Fz_*`, `X_*`, `Y_*`, `Z_*`, `Jnc_*`, `FsExt_*`, `Zgnd_*`, `Roll_E`, `Pitch`, `Yaw`, `AVx`, `AVy`, `AVz`, `Yo` exports, and write immutable JSON metadata. Implement `--log-decimation` as an integer from 1 to 10 for new probes; keep the existing 50 ms default only for historical comparison.
  ```python
  digest = hashlib.sha256(run_all.read_bytes()).hexdigest()
  if digest != manifest["generated_sha256"]:
      raise ValueError("plant hash mismatch")
  missing = sorted(required_exports - set(export_names))
  if missing:
      raise ValueError("missing export: " + ", ".join(missing))
  ```
- [ ] **Step 4: Green and native audit.** Run focused tests, then `python scripts/validate_hd_ddev_interfaces.py --simfile models/corner_module_ddev/single_wheel_deep_pothole/simfile.sim --target runs/contact_mpc_interface_gate1_a`; verify each torque and force channel's independent sign and the `FsExt` response. Preserve CSV/native history and create a units/coordinate-frame audit note. If any channel is unobservable or sign-inconsistent, stop here.
- [ ] **Step 5: Commit only this task's source/tests/docs.** Leave generated models, old runs and unrelated dirty files unstaged.

### Task 2: Calibrate contact geometry and the safety estimator

**Files:** Create `src/ddevsim/contact_geometry.py`, `tests/test_contact_geometry.py`. Read `src/ddevsim/support_geometry.py`, `src/ddevsim/vehicle_params.py`, the frozen parameter file and native traces; do not silently treat wheel-centre XY as contact-patch XY.

**Interfaces:** `tyre_envelope_gap_m(wheel_xyz: tuple[float,float,float], radius_m: float, road_height: Callable[[float,float],float], fore_aft_samples: int = 21) -> float` returns minimum signed gap across the forward/backward tyre envelope. `support_risk(state: Mapping[str,float], contact_xy: Mapping[str,tuple[float,float]], cg_xy: tuple[float,float], cg_error_m: float) -> dict` returns estimated triangle margin, conservative margin, loads and dynamic-risk status.

- [ ] **Step 1: Red tests.** A tyre centred above a flat road has positive gap; a tyre whose centre is above a 0.20 m pit but whose forward envelope intersects the exit lip has non-positive gap. Also test that `Fz=0` with non-positive gap is *not* accepted as a clean lift, and that 15 mm CG uncertainty cannot validate a 10 mm claimed margin.
  ```python
  gap = tyre_envelope_gap_m((0.0, 0.0, 0.263), 0.263, exit_lip_profile)
  assert gap <= 0.0
  assert not support_risk(state, contacts, cg_xy, cg_error_m=0.015)["verified"]
  ```
- [ ] **Step 2: Confirm red.** Run `python -m pytest tests/test_contact_geometry.py -q`.
- [ ] **Step 3: Implement geometry and uncertainty.** Sample the tyre lower circular envelope `z_low(x)=z_c−sqrt(R²−(x−x_c)²)` for `|x−x_c|≤R`; subtract the real pit-floor/lip height and take the minimum. Reconstruct full-vehicle CG from the documented masses/centres and attitude; compare the estimate to TruckSim outputs and contact-patch coordinates on whole held-out native runs. Define conservative dynamic risk from the minimum support `Fz`, normal-force-weighted contact centre `Σ(Fz_i·p_i)/ΣFz_i`, roll/rate and short-horizon projected roll `|roll|+τ|roll_rate|` with τ fixed from measured recovery time. Calibrate each bound on native traces; the weighted contact centre is an approximate COP, not a certified ZMP. Do not use `Zgnd_*` alone as a lip-clearance measure because it is a local road sample, not the full envelope.
  ```python
  dx = radius_m * (2.0 * i / (fore_aft_samples - 1) - 1.0)
  lower_z = wheel_xyz[2] - math.sqrt(max(0.0, radius_m**2 - dx**2))
  gap_i = lower_z - road_height(wheel_xyz[0] + dx, wheel_xyz[1])
  ```
- [ ] **Step 4: Green plus estimator gate.** Run focused tests. Publish worst-case CG/contact-coordinate error and compare it to the requested 0.01 m margin. For the initial gate require error ≤0.005 m, leaving at least half the 0.01 m margin for the geometric bound; if that cannot be demonstrated, report `geometry_unverified` and halt the lift-acceptance claim rather than reducing uncertainty by assumption.
- [ ] **Step 5: Commit task files and the calibration report.**

### Task 3: Establish bounded FR and RR reachability before MPC

**Files:** Create `src/ddevsim/reachability.py`, `scripts/probe_lift_reachability.py`, `tests/test_reachability.py`; reuse `run_stepwise()` and the native-history copy procedure in `scripts/run_expert_pothole.py`.

**Interfaces:** `build_profile(corner: str, force_kn: tuple[float,float,float,float], t_preload_s: float, t_lift_s: float, force_limit_n: float, slew_n_per_s: float) -> Callable[[float,Sequence[float]],tuple[float,...]]`; `classify_reachability(rows: Sequence[Mapping[str,str]], corner: str, limits: Mapping[str,float]) -> dict` returns `admissible_candidate` or `feasibility_unresolved`, first binding constraint and evidence. No classification returns `physically_infeasible` from a failed search.

- [ ] **Step 1: Red tests.** Test that a profile exceeding force/slew is clipped/rejected, a positive-load three-support FR sample with 4 mm *negative* calibrated margin is rejected, and an exhausted candidate list returns `feasibility_unresolved` rather than a no-go.
  ```python
  result = classify_reachability(rows_with_negative_margin, "FR", limits)
  assert result["status"] == "feasibility_unresolved"
  assert result["binding_constraint"] == "cg_margin"
  ```
- [ ] **Step 2: Confirm red.** Run `python -m pytest tests/test_reachability.py -q`.
- [ ] **Step 3: Implement profiles and scan.** Use four-contact bounded force ramps to trim posture, then unload FR or RR individually, with sampled force/slew limits fixed in the contract. Scan a registered finite grid of force amplitudes, phase durations and crawl speeds, saving *every* candidate's full 5 ms CSV (`log_decimation=10`) and full-rate 0.5 ms CSV (`log_decimation=1`) for contact-event candidates. Use fresh run directories. Separate stationary flat-road trim, isolated lip/clearance and low-speed pit trials; never command both wheels free.
  ```python
  requested = tuple(1000.0 * f for f in force_kn)
  applied = tuple(max(-force_limit_n, min(force_limit_n, f)) for f in requested)
  assert max(abs(f) for f in applied) <= force_limit_n
  ```
- [ ] **Step 4: Green and native Gate 2.** Run focused tests; then run `python scripts/probe_lift_reachability.py --corner FR --run-dir runs/contact_mpc_reachability_fr_a` and `python scripts/probe_lift_reachability.py --corner RR --run-dir runs/contact_mpc_reachability_rr_a`. Both paths must be unused; the script refuses existing targets. Record force/travel saturation, support loads, calibrated CG margin, road/tyre clearance and yaw. A failed search stops this plan at `feasibility_unresolved`; change model authority or pit geometry only in separately named sensitivity runs after user decision.
- [ ] **Step 5: Commit task source/tests/report; do not commit large native histories.**

### Task 4: Identify and qualify the scheduled contact predictor

**Files:** Create `src/ddevsim/contact_predictor.py`, `scripts/fit_contact_predictor.py`, `tests/test_contact_predictor.py`. Reuse Task 3 native data and Task 1 frozen contract. No training/validation row split within the same native run.

**Interfaces:** `ContactMode = Literal["FOUR_CONTACT","FR_FREE","RR_FREE","FR_TOUCHDOWN","RR_TOUCHDOWN"]`; `classify_mode(gaps_m: Mapping[str,float], loads_n: Mapping[str,float], previous: str) -> str`; `predict_trajectory(x0: np.ndarray, u: np.ndarray, schedule: Sequence[str], model: ContactModel) -> np.ndarray`; `validate_predictor(model: ContactModel, held_out_runs: Sequence[Path], margins: Mapping[str,float]) -> dict`.

- [ ] **Step 1: Red tests.** Assert that a target wheel with positive gap and negligible Fz is FREE, a support wheel with positive gap causes `UnsafeSupportLift`, a mismatched touchdown guard rejects the scheduled rollout, and a model with a wrong-sign support-load prediction fails qualification even if mean RMSE is small.
  ```python
  with pytest.raises(UnsafeSupportLift):
      classify_mode({"RL": 0.02, "FR": 0.02}, {"RL": 0.0, "FR": 0.0}, "FR_FREE")
  assert not validate_predictor(bad_sign_model, held_out, margins)["qualified"]
  ```
- [ ] **Step 2: Confirm red.** Run `python -m pytest tests/test_contact_predictor.py -q`.
- [ ] **Step 3: Implement gray-box dynamics.** Model body heave/roll/pitch and four unsprung vertical positions/velocities, passive spring/damper, bounded active seat force, jounce/rebound stops and unilateral tyre law: no tensile tyre force, zero force when separated. Identify stiffness, damping, inertias, delay and tyre law on disjoint native runs. For each candidate schedule, propagate a nominal trajectory, linearize `A[k],B[k],d[k]` and output maps, and apply gap/Fz guards; a disagreement invalidates the prediction. If native impact data require a velocity reset, estimate and document it rather than assuming continuity.
  ```python
  tyre_force_n = 0.0 if gap_m > 0.0 else max(0.0, k_tyre_npm * (-gap_m) - c_tyre_ns_pm * gap_rate_mps)
  if measured_mode != predicted_mode:
      raise ContactGuardMismatch(measured_mode, predicted_mode)
  ```
- [ ] **Step 4: Green and Gate 3.** Run tests, then fit on one group of complete native runs and validate on other runs with held-out signs, amplitudes and simultaneous commands. Report worst-case multi-step errors for `Fz`, roll/rates, jounce, CG margin and envelope gap; derive constraint tightening. Wrong sign, missed switch or uncertainty larger than its safety margin rejects the predictor. Stop before MPC if unqualified.
- [ ] **Step 5: Commit task source/tests/model card; record model-training hashes and data IDs.**

### Task 5: Implement the discrete lift scheduler and independent supervisor

**Files:** Create `src/ddevsim/lift_scheduler.py`, `src/ddevsim/lift_safety.py`, `tests/test_lift_scheduler.py`, `tests/test_lift_safety.py`. Retain `DeepPotholeExpertController` as baseline.

**Interfaces:** `LiftScheduler.advance(t_s: float, observation: Mapping[str,float], preview: TerrainPreview, prediction: Prediction | None) -> str`; `LiftSafety.check(observation: Mapping[str,float], applied_u: Sequence[float], solver_status: str) -> SafetyDecision`, where `SafetyDecision.action` is `CONTINUE`, `RECOVER` or `STOP` and includes the first violated constraint.

- [ ] **Step 1: Red tests.** PRELOAD from a four-contact stance on the future triangle edge must *command posture trim* rather than deadlock or zero all forces. Actual FR unload below a hysteretic threshold switches to measured three-support checks. RR_PRELOAD cannot start until FR has landed, all four wheel loads recover, and yaw/path gate passes. A support wheel lift or stale solve yields RECOVER/STOP.
  ```python
  assert scheduler.advance(0.0, four_contact_on_edge, preview, safe_future_prediction) == "FR_PRELOAD"
  assert scheduler.advance(2.0, recovered_but_yawed, preview, None) != "RR_PRELOAD"
  assert safety.check(rl_lifted, eight_inputs, "OK").action == "RECOVER"
  ```
- [ ] **Step 2: Confirm red.** Run `python -m pytest tests/test_lift_scheduler.py tests/test_lift_safety.py -q`.
- [ ] **Step 3: Implement phase/guard logic.** Use `APPROACH→FR_PRELOAD→FR_LIFT→FR_CROSS→FR_TOUCHDOWN→FOUR_CONTACT_RECOVERED→RR_PRELOAD→RR_LIFT→RR_CROSS→RR_TOUCHDOWN→DONE`; `RECOVER/STOP` never become success states. During four-contact preload use four-contact safety plus *predicted future* three-contact margin. Once the target unload guard persists for its calibrated dwell, require actual three-support load, geometry and dynamic-risk bounds at every 0.5 ms callback. Return typed cause and bounded recovery action.
  ```python
  if phase.endswith("PRELOAD") and target_fz_n > lift_guard_n:
      require_four_contact_safety(observation)
      require_future_three_contact_margin(prediction)
  elif phase.endswith(("LIFT", "CROSS")):
      require_three_support_safety(observation)
  ```
- [ ] **Step 4: Green and replay.** Run focused tests, replay iter10 and existing failing histories through supervisor, and verify no historical failed run is reclassified as successful. Record transitions and abort reasons.
- [ ] **Step 5: Commit task source/tests.**

### Task 6: Solve the bounded suspension plan and coordinate wheel torque

**Files:** Create `src/ddevsim/lift_mpc.py`, `src/ddevsim/lift_control.py`, `tests/test_lift_mpc.py`, `tests/test_lift_control.py`. Use Task 4 predictor, Task 5 scheduler/supervisor, existing `src/ddevsim/suspension_actuator.py`, and the established four torque channels.

**Interfaces:** `solve_suspension_mpc(model: ContactModel, x0: np.ndarray, schedule: Sequence[str], reference: np.ndarray, bounds: MPCBounds, warm_start: np.ndarray | None) -> MPCResult`; `LiftControl.__call__(time_s: float, exports: Sequence[float]) -> tuple[float,...]` returns exactly eight inputs in `IMPORT_NAMES` order. `MPCResult` includes `status`, four force commands, active constraints, predicted minima, solve time and nonlinear recheck status.

- [ ] **Step 1: Red tests.** Test force/slew and support-load constraints, no safety slack application, expired solve deadline, failed optimizer, trust-region breach and nonlinear lip-gap violation. Assert lifted-corner torque is zero and left/right differential torque is bounded by predicted `Fz·μ·R`, with yaw correction not stealing necessary traction.
  ```python
  result = solve_suspension_mpc(model, x0, schedule, ref, bounds, None)
  assert result.status == "REJECTED_CLEARANCE"  # candidate collides with exit lip
  assert control(t_s, exports)[1] == 0.0  # FR torque port while FR is free
  ```
- [ ] **Step 2: Confirm red.** Run `python -m pytest tests/test_lift_mpc.py tests/test_lift_control.py -q`.
- [ ] **Step 3: Implement the optimisation.** Benchmark update periods 10/20/40 ms; choose only after observing force and contact transients. Derive horizon from measured force-build/settling and next contact event; the 1.6 m trigger is ~2.06 s before the lip at 2.8 km/h, so use a precomputed phase reference/deadline if a short MPC horizon cannot span it. Minimise a quadratic force/posture/clearance tracking cost subject to hard linearized, uncertainty-tightened force, slew, jounce, three-load, attitude and signed-gap constraints. Use sequential affine QPs in a trust region with warm start; re-evaluate the full nonlinear gap/triangle constraints before applying any result. A solver failure produces a bounded recovery command, never the previous hazardous command.
  ```python
  solution = scipy.optimize.minimize(objective, warm_start, jac=gradient,
                                     constraints=linear_constraints,
                                     bounds=variable_bounds, method="SLSQP")
  if not solution.success or elapsed_s > deadline_s or not nonlinear_safe(solution.x):
      return MPCResult.rejected("SOLVER_OR_SAFETY", recovery_force_n)
  ```
- [ ] **Step 4: Green and Gate 4.** Run focused tests and replay synthetic/held-out transitions. Report p95/p99 solve time, deadlines, chosen period/horizon, nonlinear recheck failures and force-vs-`FsExt` tracking. Initially require p99 solve time <80% of the chosen update period with zero missed deadlines across the held-out replay; this is a project timing gate, not a hardware rating. If deadlines or safety margins fail, stop and redesign the optimizer; do not run an unqualified MPC in TruckSim.
- [ ] **Step 5: Commit task source/tests/benchmark report.**

### Task 7: Integrate native A/B runs and stringent physical acceptance

**Files:** Extend `scripts/run_expert_pothole.py` with explicit `--controller legacy|contact_mpc` and contract/model paths; extend `src/ddevsim/traversal_metrics.py`, `tests/test_traversal_metrics.py`; create `scripts/report_contact_mpc.py`, `tests/test_contact_mpc_runner.py`. Do not alter video textures or visuals.

**Interfaces:** `evaluate_contact_traversal(rows, scenario, *, contract, phase_events, geometry, thresholds) -> dict` accepts 5 ms CSV plus 0.5 ms native event trace and returns `passed`, FR/RR per-window results and exact failure reasons. `run_exit_code()` remains nonzero on any acceptance failure.

- [ ] **Step 1: Red tests.** Assert a run with 90% low target `Fz` but pit-floor contact fails, passive RR unloading without commanded RR phase fails, a high landing spike visible only in 0.5 ms history fails, and a completed solver with `DONE` but yaw >2° fails.
  ```python
  report = evaluate_contact_traversal(rows, scenario, contract=contract,
                                      phase_events=events, geometry=geometry,
                                      thresholds=thresholds)
  assert not report["passed"]
  assert "FR_pit_floor_contact" in report["failure_reasons"]
  ```
- [ ] **Step 2: Confirm red.** Run `python -m pytest tests/test_traversal_metrics.py tests/test_contact_mpc_runner.py -q`.
- [ ] **Step 3: Implement immutable run metadata and acceptance.** Register exact entry/interior/exit windows before each A/B group. Require ≥90% unload over the extended window, continuous unsupported/uncertainty-tightened positive envelope gap through the pit interior, no pit-floor contact, all three support loads above their tightened floor (never <300 N), calibrated CG margin ≥0.01 m only if its error ≤0.005 m, dynamic-risk pass, roll ≤7°, yaw ≤2°, lateral drift ≤0.15 m, run-specific travel limit and landing load ≤4× largest static load. Record target vertical velocity immediately before touchdown, peak roll rate and body vertical acceleration, and evaluate impact peaks on `log_decimation=1` (0.5 ms) CSV rather than the 5 ms routine log. Require commanded FR and RR phases separated by observed four-contact recovery. Store requested/applied eight inputs, native history, hashes, solver status, safety events and model card in `manifest.json`; do not copy an old history into a new result.
  ```python
  reasons = []
  if any(sample.tyre_gap_lower_bound_m <= 0.0 for sample in fr_interior):
      reasons.append("FR_pit_floor_contact")
  if not (fr_touchdown_t < recovered_four_contact_t < rr_lift_t):
      reasons.append("sequence_not_verified")
  ```
- [ ] **Step 4: Green and native Gate 5.** Run `python -m pytest -q`, then isolated FR and RR native trials, then sequential trial, each in a unique directory. If any gate fails, retain raw data and state `controller_failed_on_frozen_plant` with the first active constraint; never label the video successful. If all pre-registered criteria pass, run identical-hash A/B cases (legacy, contact MPC, contact MPC without preview), uncertainty/depth sensitivity cases separately, and export a video from that successful native history using `scripts/export_native_video.py`.
- [ ] **Step 5: Commit only implementation/tests/report code.** Archive large solver histories as artifacts outside Git unless the repository policy explicitly requires them.

## Final self-check before any success claim

- [ ] `python -m pytest -q` exits zero on the implemented branch.
- [ ] All five gate reports exist, refer to the same frozen primary plant/scenario hashes and include failed attempts.
- [ ] FR and RR each have native-solver evidence of commanded lift, true clearance, three support loads, safe attitude and landing; four-contact recovery lies between them.
- [ ] The acceptance result was computed from registered windows and 0.5 ms impact data, not a camera view or FSM `DONE` alone.
- [ ] A successful video is created **only** from a passing native run; otherwise provide the blocking constraint and `feasibility_unresolved`/`model_unqualified`/`controller_failed_on_frozen_plant` classification.
