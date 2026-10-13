# Shallow MPC Route Validation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Test whether a classical dynamic predictor supports constrained shallow-pit MPC, using independent native TruckSim data rather than controller-design assumptions.

**Architecture:** Keep validation code exclusively under shallow_press and new diagnostic scripts. A classical regularized MIMO dynamic model is a first falsifiable predictor; if independent multi-step prediction fails at critical pit events, do not enable an MPC expert. Native linearization is an additional extraction feasibility probe, not a substitute for nonlinear validation.

**Tech Stack:** Existing Python/NumPy/SciPy, TruckSim2019 DLL, pytest; no new runtime dependency.

**Spec:** docs/SHALLOW_PRESS_CLASSICAL_DESIGN_20261012.md; user authorized model validation first, adaptive1–3km/h, known pit, aligned expert-one capability.

## Global Constraints

- Same tuned_ii_20261005 plant and separate per-run native process; preserve existing untracked playback evidence.
- No imports of wheel_lift/static_wheel_lift or scripts from shallow controller/model modules.
- ±18200N, 45400N/s outer force rate, Jnc[-.149,.155]m; ±500Nm,1000Nm/s; SW±220deg,400deg/s;20ms.
- Keep ShallowPressStrategy NOT_IMPLEMENTED until full expert acceptance; research gate PASS never means expert task PASS.
- Completed solver and rear exit are independent conditions; no attitude-triggered truncation.
- Use complete trajectories and independent runs, not shuffled adjacent samples, for training/validation.

## Review Focus

- Commands measured at time t predict the subsequent plant interval; avoid a one-sample timing leak.
- Road preview must come from configured spatial ground, not future observed wheel ground/contact signals.
- Free-run predictions must not reset from truth inside the horizon; invalid/flight predictions must not be hidden.
- Report rank/conditioning and per-output/event errors; do not accept average fit that hides contact-loss errors.
- Freeze gates before looking at holdout; preserve failed native runs and source/model hashes.

### Task 1: Predictor and metric contract

**Files:** src/ddevsim/strategies/shallow_press/prediction.py; tests/shallow_press/test_prediction.py.

**Interfaces:** fit_dynamic_model(episodes, ridge) -> DynamicModel; rollout(model, history, future_inputs, future_road) -> ndarray; prediction_metrics(truth,prediction,contact_truth) -> dict.

- [x] Write tests for a hand-defined stable dynamic system, multi-step free-run, command interval alignment and lost-contact false-support classification.
- [x] Run tests RED, implement scaled ridge predictor and metrics, run GREEN and architecture checks.

### Task 2: Native identification/validation collection

**Files:** src/ddevsim/strategies/shallow_press/validation.py; scripts/run_shallow_mpc_validation.py; configs/shallow_press/mpc_validation_20261013.json; tests/shallow_press/test_mpc_validation.py.

**Interfaces:** ValidationExcitation per-instance callback; collect_episode/worker; independent frozen train/holdout matrix.

- [x] Test zero-start, bounded final commands, distinct input excitation, no overwrite, spatial road consistency and control timing.
- [x] Run RED, implement GREEN. Separate deterministic train seeds and withheld speed/depth/run conditions.
- [x] Run native extraction probe and training runs, preserve all artifacts.

### Task 3: Independent prediction and MPC feasibility gate

**Files:** scripts/analyze_shallow_mpc_validation.py; docs/SHALLOW_MPC_VALIDATION_20261013.md; evidence/shallow_mpc_validation_20261013/.

- [x] Evaluate 20/100/200/400ms predictions on full held-out pit event windows, including entry/exit and both axles, and against persistence baseline.
- [x] Freeze research thresholds: Jnc RMSE≤5mm and maximum≤15mm; attitude RMSE≤.5deg/max≤1.5deg; wheel-load RMSE≤250N/max≤750N; speed RMSE≤.05m/s/max≤.15m/s. Critical event errors and false-support intervals gate load-aware MPC separately; these are research thresholds, not vehicle safety standards.
- [x] Check step sensitivity for selected small-pit conditions; use independent seeds and full duration. Evaluate supported horizons and model validity range.
- [x] Predictors fail; record controller gate/reasons, no MPC closed-loop release or expert PASS.
- [x] Write Chinese report with successes/failures, source identities, limitations, next model revisions; full suite and diff check, independent review, local git save.

## Execution rulings

User explicitly approved execution. Continue in existing project feature branch with new shallow-only files and exclusive runs; no simultaneous strategy-one edits. This preserves the user's established in-project workflow. Predictor order/feature choices may be revised using training-only data; held-out results must not be used to tune and re-label the same set as unseen.
