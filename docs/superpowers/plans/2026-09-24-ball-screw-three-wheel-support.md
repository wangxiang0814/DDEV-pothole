# Ball-Screw Three-Wheel Support Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the existing TruckSim corner-module DDEV complete FR then RR wheel-lift traversal while retaining safe three-wheel support and reporting measured dynamics.

**Architecture:** Keep TruckSim's passive springs, dampers and stops; inject bounded actuator thrust through the existing four `IMP_FS` channels. Identify per-corner signed response, derive posture and load targets from real contact geometry, enforce plant/configuration consistency, and accept only a native-solver run whose dynamics and video demonstrate both crossings.

**Tech Stack:** Python 3, pytest, TruckSim 2019 VS Solver API, existing `ddevsim` modules.

**Spec:** `docs/superpowers/specs/2026-09-24-ball-screw-three-wheel-support-design.md`

## Global Constraints

- Retain the current Compact Utility Truck (I_I), its mass and its four independent wheel-torque and four independent suspension-force channels.
- Retain passive suspension spring and damper terms; do not label the existing `--neutralize-springs` branch as the paper's plant.
- Preserve all user-uncommitted code and videos. New runs go to distinct run directories.
- Do not claim a ball-screw electric-motor model without measured or sourced motor/electromechanical parameters.
- Do not reproduce unpublished T-S/LMI gains by assertion; identify the actual classical controller used.
- A passing unit-test suite or a visually pleasing video is not sufficient evidence of stable traversal.

## Review Focus

- Controller flag says ball-screw while model manifest says normal springs: fail before solver launch.
- Gain file and vehicle parsfile have different hashes: reject the gain file and do not silently apply stale calibration.
- A support tire's measured `Fz` falls below its minimum while the target wheel is lifted: hold or recover, not progress to crossing.
- CG projection crosses a triangle boundary even if all sampled loads momentarily appear positive: hold or recover.
- Current 0.20 m pothole passes but an enlarged pothole does not: report the maximum verified depth and limiting constraint without relabeling the baseline.

---

### Task 1: Preserve and Reproduce the Current Plant

**Files:**
- Modify: `scripts/run_expert_pothole.py`
- Modify: `tests/test_expert_controller.py`
- Modify: `tests/test_hd_ddev_case.py`
- Read: `models/corner_module_ddev/run_all.par`, `models/corner_module_ddev/source_manifest.json`

**Interfaces:**
- Consumes: source manifest `generated_sha256`, `neutralize_springs.applied`, and actual `run_all.par` bytes.
- Produces: `validate_plant_mode(manifest: dict, run_all: bytes, ball_screw_mode: bool) -> None` raising `ValueError` on mode/hash mismatch.

- [ ] Add a failing test that passes `neutralize_springs.applied=false` with `ball_screw_mode=true` and expects `ValueError`; add another for a mismatched SHA-256.
  ```python
  with pytest.raises(ValueError, match="plant mode"):
      validate_plant_mode({"generated_sha256": hashlib.sha256(b"plant").hexdigest(),
                           "neutralize_springs": {"applied": False}}, b"plant", True)
  ```
- [ ] Run `python -m pytest tests/test_expert_controller.py tests/test_hd_ddev_case.py -q` and record the expected failure.
- [ ] Implement `validate_plant_mode` and call it before calibration or solver launch. The approved primary configuration uses normal springs and no `--ball-screw` flag; leave the flag available only for explicitly matched comparison models.
  ```python
  if bool(manifest["neutralize_springs"]["applied"]) != ball_screw_mode:
      raise ValueError("plant mode does not match controller mode")
  if hashlib.sha256(run_all).hexdigest() != manifest["generated_sha256"]:
      raise ValueError("plant hash does not match source manifest")
  ```
- [ ] Run the focused tests, then archive a fresh untouched-plant baseline with a unique run ID and record source/model hashes, command flags, and the original safe-stop reason.
- [ ] Commit only source and test files for this task.

### Task 2: Identify the Four Force Channels and Actuator Limits

**Files:**
- Modify: `scripts/probe_actuator_gain.py`
- Create: `src/ddevsim/suspension_actuator.py`
- Create: `tests/test_suspension_actuator.py`
- Modify: `src/ddevsim/channels.py`

**Interfaces:**
- Produces: `SuspensionActuator.step(requested_n: float, dt_s: float) -> float`, with explicit force, slew and first-order response bounds; zero command leaves the passive suspension active.
- Produces: one signed probe record per `IMP_FS` corner containing command, `Jnc`, `Fz`, `Roll_E`, `Pitch_E`, `Fs`, `Fd`, `FsExt`, model hash and probe duration.

- [ ] Write failing actuator tests: zero input yields zero active force; positive and negative requests obey force and slew bounds; repeated steps approach the requested value without overshoot. Write a probe-schema test requiring eight signed experiments (four corners × two signs).
  ```python
  actuator = SuspensionActuator(force_min_n=-5000, force_max_n=5000,
                                slew_n_per_s=1000, time_constant_s=0.05)
  assert actuator.step(0.0, 0.01) == 0.0
  assert 0.0 < actuator.step(5000.0, 0.01) <= 10.0
  assert len({(r["corner"], r["sign"]) for r in report["experiments"]}) == 8
  ```
- [ ] Run `python -m pytest tests/test_suspension_actuator.py -q` and confirm expected failures.
- [ ] Implement the bounded actuator and expand the probe recorder without changing the native vehicle spring tables. Unit-test the force balance using exported passive and external force channels.
  ```python
  limited = min(self.force_max_n, max(self.force_min_n, requested_n))
  lagged = self.actual_n + (limited - self.actual_n) * (1.0 - math.exp(-dt_s / self.time_constant_s))
  self.actual_n += min(self.slew_n_per_s * dt_s,
                       max(-self.slew_n_per_s * dt_s, lagged - self.actual_n))
  ```
- [ ] Run the probe through TruckSim for all eight experiments; save measured signs, cross-couplings and time constants in a new immutable report. Stop if force injection lacks distinct per-corner effect or force telemetry cannot close the balance.
- [ ] Run focused and full tests; commit source/tests but not generated data or video.

### Task 3: Three-Contact Posture, Load and Force Allocation

**Files:**
- Modify: `src/ddevsim/expert_controller.py`
- Modify: `src/ddevsim/bounded_allocation.py`
- Modify: `tests/test_expert_controller.py`
- Modify: `tests/test_bounded_allocation.py`

**Interfaces:**
- Produces: `support_triangle_margin_m(contact_xy: Mapping[str, tuple[float, float]], cg_xy: tuple[float, float], lifted: str) -> float`, positive only inside all three contact edges.
- Consumes: actual wheel contact XY, measured `Fz`, body roll/pitch, spring/damper/external forces, and identified actuator matrix.
- Produces: per-step commanded/real actuator force, three-contact margin, minimum support load, phase and safety reason.

- [ ] Add failing geometry tests for inside, boundary and outside CG projections, and a controller test where diagonal support load is 522 N against a 1000 N operating floor: PRELOAD must not enter LIFT.
  ```python
  xy = {"FL": (-1.0, 1.0), "RL": (1.0, 1.0), "RR": (1.0, -1.0)}
  assert support_triangle_margin_m(xy, (0.2, 0.2), "FR") > 0.0
  assert support_triangle_margin_m(xy, (-1.1, -1.1), "FR") < 0.0
  assert not support_ready(loads_n={"FL": 6700, "RL": 522, "RR": 6100},
                           minimum_n=1000, triangle_margin_m=0.05)
  ```
- [ ] Run `python -m pytest tests/test_expert_controller.py tests/test_bounded_allocation.py -q`; confirm only the new tests fail.
- [ ] Implement the triangle margin and a feasible target generator using current contact geometry, actual vehicle mass/CG, travel and force bounds. Preserve paper's sign pattern but recompute numeric targets. Separate spring/damper equilibrium feedforward from active actuator feedback; remove any claim that the paper actuator replaces the spring.
  ```python
  # For each counter-clockwise edge p -> q, positive signed distance is inside.
  edge_distance = ((qx - px) * (cgy - py) - (qy - py) * (cgx - px)) / math.hypot(qx - px, qy - py)
  margin_m = min(edge_distances_m)
  sd_error_m = target_sd_m - measured_sd_m
  active_request_n = equilibrium_n + sd_error_m / measured_sd_gain_m_per_n \
      - measured_sd_rate_m_s * damping_gain_n_s_per_m
  ```
- [ ] Add bounded correction to protect all three measured support loads and triangle margin. Do not weaken existing roll/yaw/landing safety gates. Test an infeasible target leads to HOLD/RECOVER and never to an extra lift.
- [ ] Run focused and full tests; commit only task source/tests.

### Task 4: Native Solver Validation and Evidence

**Files:**
- Modify: `scripts/run_expert_pothole.py`
- Modify: `src/ddevsim/cosim.py`
- Modify: `tests/test_cosim.py`
- Modify: `docs/strategy_envelope.md`
- Read: `models/corner_module_ddev/single_wheel_deep_pothole/scenario.json`

**Interfaces:**
- Produces: unique run directory with raw TruckSim history, detailed CSV, manifest, per-stage metrics and video; never overwrites `runs/corner_module_expert_pothole`.
- Produces: machine-checkable verdict for FR crossing, four-wheel recovery, RR crossing, minimum three-support load, minimum triangle margin, travel/force bounds, roll/yaw/path, landing loads and stop state.

- [ ] Add failing metrics tests with a synthetic incomplete FR-only trace and a trace whose third support load goes to zero; both must fail the verdict despite `safe_stop=false`.
  ```python
  assert not evaluate_traversal(fr_only_trace, scenario).passed
  assert not evaluate_traversal(trace_with_zero_rl_load, scenario).passed
  ```
- [ ] Run `python -m pytest tests/test_cosim.py -q`, then implement explicit measured-dynamics verdicts and run provenance.
  ```python
  passed = (fr_crossed and four_contact_restored and rr_crossed
            and min_support_load_n > 0.0 and min_triangle_margin_m > 0.0
            and not safe_stop and force_and_travel_within_limits)
  ```
- [ ] Run static, flat-road lift and current 0.20 m single-track pothole in that order. After each run inspect measured force/position/load traces; change one controller parameter or model hypothesis per iteration. Verify both FR and RR contact trajectories intersect the pit, each lifted wheel is unloaded before the lip, other three loads and triangle margin stay positive, and no unsafe stop occurs.
- [ ] Export native history and video to the unique run directory. Inspect frames at FR approach/cross/landing and RR approach/cross/landing; report video and CSV together.
- [ ] Only after the baseline passes, increase pothole depth in isolated runs and document the first active constraint and largest verified depth. Run `python -m pytest -q`, record exit code and test count, and update `docs/strategy_envelope.md` with actual measured values and known limits.
- [ ] Commit source/tests/docs only; leave generated runs and all pre-existing user media untouched.

## Plan self-review

The four tasks cover model/configuration integrity, signed actuator calibration, physically feasible three-wheel posture, and native-solver evidence. Generated runtime output is kept separate from source commits. Each acceptance test uses actual contact/load dynamics rather than phase flags alone. If the TruckSim actuator-point experiment fails, stop with evidence and revisit the plant architecture before attempting CarSim.
