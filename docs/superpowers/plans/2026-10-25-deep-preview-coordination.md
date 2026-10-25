# Deep-pit preview coordination implementation plan

> **For agentic workers:** Use superpowers:executing-plans inline; read-only audits may use superpowers:dispatching-parallel-agents. Implement task-by-task with test-driven-development and verification-before-completion.

**Goal:** Improve expert-two deep-pit contact/body control and determine finite depth/speed coverage without changing expert one or the nominal capability.

**Architecture:** Preserve preview + classical feedback, independent native workers and immutable evidence. Separate model reach, wheel kinematics/contact diagnostics and control changes; retain legacy control/metrics behind explicit versioned choices.

**Tech Stack:** Python3.8, numpy/scipy, pytest, TruckSim2019/MF52, PowerShell, local Git.

**Spec:** ../specs/2026-10-25-deep-preview-coordination.md

**Iteration result:** Frozen coverage72/72 completed,20 research passes; confirmation8/8 completed,2 passes; intermediate4/4 completed,3 passes. Original requested8–10cm /1–6km/h robust operation is NOT achieved. Model applicability, deep-pit step sensitivity and delay sensitivity remain open. Code checkpointdf61599; evidence checkpoint is tagged research/shallow-deep-contact-20261025.

## Global constraints

- Expert-one files, source vehicle, native histories and verified plans unchanged.
- SI FL/FR/RL/RR; outer20 ms, motor2 ms; separate DLL process per run, maximum two concurrent native jobs.
- ±18200 N/45400 N/s; ±500 Nm/1000 Nm/s; explicit diagnostic-only faster torque.
- Full configured simulation; no posture truncation or video export. Task pass requires all versioned criteria, not process completion.
- Real geometry/hardware unknown: full acceptance=false. Distinguish finite search failure from physical no-go.

## Review focus

- Contact tangent speed versus horizontal body speed, including wheel vertical motion at steep edges.
- Positive/negative suspension force and wheel travel signs, static preload versus remaining extension.
- Delayed measurements, recontact torque restoration, slew saturation and hidden state coupling.
- Front/rear completion and full duration even at failed deep cases; true actual speed.
- Unmatched source/config/plant hashes and old evidence reclassification.

## T1: Depth and failure diagnosis

- [x] Freeze original V2 12 deep cases at8/10/12 cm,1/3 km/h, two pit types; verify full logs.
- [x] Audit model stops/static headroom; run minimal current-MF52 stationary force/geometry checks and label reach-only results.
- [x] Analyze loaded slip, native Kappa/contact moments, normal/tangent kinematics and force/torque timing.
- [x] Record a causal hypothesis and select one structural fix; no stacked blind gain sweeps.

## T2: Correct measured defect and coordinate forces

**Files:** shallow_press/{config,controller,actuation,preview,assessment}.py and focused new module(s) only as justified; tests/shallow_press/.

- [x] Add failing physical-consistency tests for the identified defect and explicit legacy compatibility.
- [x] Implement minimal correction; run focused and full tests.
- [x] Compare same-target deep anchors against V2; complete4cm/3kmh and10cm/1kmh full/traction comparisons, repeat, half-step and80ms delay; report actual speeds rather than claiming exact speed-matched trajectories.
- [x] Commit tested control and diagnostic changes independently.
- [ ] Achieve robust joint passes for the requested8–10cm deep anchors. Currently failed; cannot infer a physical stroke no-go from these failures.

## T3: Coverage and robustness

**Files:** run_shallow_preview[_matrix].py only if instrumentation needs it; new configs/evidence, without replacing old snapshots.

- [x] Freeze chosen controller then cover depths2/4/6/8/10/12 cm and requested1–6 km/h in two pit types using unique runs.
- [x] Add intermediate speeds, step-halving, delay and repeated checks around boundary; report failures.
- [x] Analyze RMS/impact, roll/pitch, loaded slip/contact loss, load peaks, travel, input caps and full crossing/stop.
- [ ] Establish continuous-speed and uncertain-map/delay robustness. Wide4cm/2.5kmh and80ms delay still fail; deep10cm is step-sensitive.

## T4: Versioned delivery

- [x] Scientific plots, concise algorithm description, exact case playback index and honest depth/speed boundary report.
- [x] Full pytest, git diff --check, expert-one file identity/entry checks, source/input/plant/archive SHA verification.
- [x] Archive superseded trial configs, retain original native outputs, commit on existing feature branch and report remaining limits.
