# Mapped force preview implementation plan

> Use executing-plans inline, test-driven-development and a final fresh-context review. Read-only force-map audit is independent; native scheduling belongs only to the parent agent.

**Goal:** Replace under-scaled spring preview with an opt-in model-coordinate force prediction, then resolve timing using a small deep-first experiment sequence.

**Architecture:** Strategy-specific passive map and reference-path state; conditional initialized observations; existing feedback/traction/steering and native actuator projector retained. Historical default controller remains available.

**Tech:** Python3.8/numpy/pytest, native TruckSim/MF52; maximum2 independent native processes.

**Spec:** ../specs/2026-10-25-mapped-force-preview-design.md

## Constraints and review focus

- SI FL/FR/RL/RR; no shared model or expert-one edits; no original result changes.
- Force18200N/45400N/s, torque500Nm/1000Nm/s; no threshold relaxation.
- Full seat-map endpoint extrapolation and virtual-work signs; reference hysteresis continuity; no live passive-force cancellation.
- Legacy mode requires no extra channels; mapped modes reject missing initialization; stale feedback still unloads through old projector.
- Config defaults preserve legacy trajectory; snapshots include every new runtime dependency; passing means joint research screen only.

## Tasks

- [x] T1: tests/shallow_press/test_force_preview.py: motion-ratio segments, flat-zero, spring generalized-force restoration, rear path continuity, damper asymmetry; expose tests failing before module exists. Implement force_preview.py with explicit source assumptions. Failed branch subsequently archived.
- [x] T2: config.py/controller.py/native.py opt-in FF modes and reference bounds, conditional CmpS/Fs inputs, diagnostics; worker captures force_preview.py source. Test missing channels/default compatibility, then full pytest. Failed mapped mode removed from active interface after experiments.
- [x] T3: freeze each source/config before native run. Initial five decisions led to bounded wheel-load feedback and torque coordination isolation rather than a broad coverage grid; extension decisions are recorded below. Compare actual speeds and phase/force/contact metrics against retained V2; no claim of an improved validated envelope.
- [x] T4: default-off load research hook retained, mapped/exit failures archived; exact failures and playback IDs recorded, source/input/plant/raw history verified and fresh-context reviewed. Two native legacy regressions are byte-identical;586 tests pass. Commit on existing branch is the delivery step; this completes this experiment plan, not the unresolved10cm/1–6km/h controller objective.

## Ledger

- Ruling: use complete g(j) tables rather than local .782/.956 constants; the static audit proved slopes vary below20mm.
- Ruling: new model table calculations are control-side approximations only; native plant remains the user-adopted3114 research baseline.
- Decision experiments1–5: mapped spring/passive, preview0.65s, reference bandwidth20, and P-only motor at the new force setting all fail joint screening. Earlier preview moves the reference but raises speed excursions; removingD worsens slip. Do not expand these settings into a coverage grid.
- Evidence-led extension: instead of remaining half-step/right/high-speed trials on an already regressed force setting, test a bounded2ms load-feedback loop using the already explicit motor-feedback sensor delay. Static flat load anchor;10ms filter; spring-seat virtual-work conversion; spatial pit window; ±2000/3000N incremental caps; original total working and hard force/rate bounds. No real sensor-bandwidth claim. Two targeted gain cases on the original legacy FF distinguish wheel-load feedback from mapping changes. If useful, confirm half-step and expand deep anchors; otherwise archive failures. This is a control-side branch under existing authorization, not model adoption.
- Additional root isolation: fastload0.5 improves nominal comfort but worsens slip; tested side-conserving exit torque bias alone/combined (both worse), then groundhook0 alone/combined (both fail). Confirmed fastload0.5 at half step and right10cm; neither jointly passes.13 control runs total; stop the parameter branch and do not expand a failed candidate to1–6km/h.
- Official VS two-point averaging is internal-tire geometry, not MF-Swift enveloping. Existing guard rejects mixing it with externalMF52; retain that guard. OriginalPAC52 point/150mm average at0.5/0.25ms were independently compared and all failed. No alternative model adopted; MF52 baseline unchanged.
- Final disposition: mapped force and exit torque modes/test implementations archived, activity removed; retain only disabled-by-default fastload research hook and reusable explicit seat map. Add provenance coverage ofvehicle_case.py, source-path normalization and explicit unadopted-model result scope. Final native legacy regression and clean-tree delivery checks required before commit.
