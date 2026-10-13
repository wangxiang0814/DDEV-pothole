# Shallow physical MPC continuation

> **For agentic workers:** Use superpowers:executing-plans. Continue the user-approved MPC validation in this checkout; strategy-specific diagnostics only.

**Goal:** Test an actual native physical predictor with height/slope preview and unilateral tyre contact before releasing a pit MPC controller. If the physical predictor still fails the frozen gates, document its usable domain and the next preview feedforward/feedback route; do not imply MPC in general is impossible.

**Architecture:** Add shallow-only physical prediction and native instrumentation modules. Keep the nine-channel public control contract and vehicle generation unchanged. Extra ADD-zero road ports are explicitly versioned diagnostic instrumentation in exclusive run models. Full native histories and rear completion are mandatory. No posture truncation, no video exports.

**Files:** `physical_prediction.py` owns affine SI propagation/contact correction; `native_physical_probe.py` owns diagnostic road ports/state snapshots; `scripts/run_shallow_physical_probe.py` collects one process/run; `scripts/analyze_shallow_physical_probe.py` validates immutable sources, computes holdout gates and plots. Evidence lives in `evidence/shallow_physical_mpc_20261013/`.

## Task 1: Physical predictor and instrumentation

- [x] RED tests: affine drift, road disturbance, unilateral load and corresponding acceleration, dimensional rejection, port-count edits and final END insertion.
- [x] Implement `physical_rollout(model, state_delta, inputs, dt, unilateral=False, max_step_s=.001)` with no measured-state resets within horizon; freeze the flat physical model. Road channels: heights, longitudinal and lateral slopes FL/FR/RL/RR, internal SI. Correction maps missing tensile force back through the height-input dynamics; identify this explicitly as an approximation.
- [x] Instrument a fresh 10 s flat run; obtain 67-state matrices with 21 inputs and state/output equilibrium. Verify ADD-zero equivalence against an independently generated nine-input run with identical commands.
- [x] Run focused tests and record source hashes.

## Task 2: Independent physical prediction screen

- [x] Collect new passive two-type 2/4/6 cm at 2 km/h, full 45 s. Record native states, contact-patch positions and model-map preview. No fitting against these holdouts.
- [x] Predict 20/100/200/400 ms with fixed flat matrices, exact initial native state (explicitly oracle research input), known applied command sequence and causal constant-speed map preview. Compare linear and unilateral variants to frozen prior gates and persistence; include critical edges and rear exit.
- [x] If qualified, move to bounded research MPC; if not, keep expert NOT_IMPLEMENTED and state exactly what blocked release. Do not relax gates or use future true states in propagation.
- [x] Produce summary, plot, source/run/code hashes and indexed native replay paths; run full pytest and diff checks, obtain independent code review and save Git version.

Execution record: 14 successful native runs (2 flat references,6 distinct pit cases each repeated to add derivative snapshots),3 preserved failed reference explorations. Added current affine/gap anchoring and a14-state vertical reduction after fixed67-state failure, still with the same frozen gates. These were diagnostic candidates, not a nonlinear or LTV MPC controller. No candidate passed; closed-loop release condition not met. Final evidence `release_screen_05ms`/`release_screen_1ms`, report `docs/SHALLOW_PHYSICAL_MPC_20261013.md`. Independent review's acceleration/source findings fixed with RED→GREEN tests; timestep finite guard regraded as an important API invariant, persistence omission as a required plan deliverable. Propagated-state count clarified.16 new focused tests;545 full-suite passes.

## Review focus

- Diagnostic ADD-zero ports must not alter vehicle physics or the public import contract.
- Raw native state pointers are internal SI; exports need explicit conversions.
- Clamp correction must affect state dynamics, not merely hide negative output loads.
- Pit preview must use only current contact-patch pose and current speed.
- Oracle state initialization/applied future commands are not an online MPC controller.
- Unknown real chassis geometry remains UNASSESSED; a model gate pass is not a vehicle safety pass.
