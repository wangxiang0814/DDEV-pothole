# Expert two preview feedback implementation

> Execute inline with superpowers:executing-plans and test-driven-development. Spec: ../specs/2026-10-24-shallow-preview-feedback-design.md. User explicitly selected implementation on2026-10-24.

- [x] T1 RED→GREEN: finite-radius preview, contact/load-aware torque, instance-isolated SI strategy, signed feedback, stale observations, amplitude/rate projection and rear-exit dwell. Files `shallow_press/{preview,traction,config,controller}.py`; tests `test_preview_control.py`.
- [x] T2 Native worker and independent acceptance; integrate run_strategy shallow branch before wheel-lift imports; preserve native display and camera infrastructure. Exact manifest/config/code hashes, unique paths, full duration; native force sign/response and initial closed loops. Tests of SI bridge and result gates.
- [x] T3 Initial frozen22-case matrix and matched passive ablations completed:18 research screens pass, all failures retained; repeated case, integration and delay comparisons included. See evidence/shallow_preview_feedback_20261024. This does not qualify untested speeds/depths or real safety.
- [x] T4 Initial version0ab1eb2 archives MPC-only code/evidence and redundant tuning configs with hashes, keeps expert-one files unchanged, passes551 tests. First independent review findings fixed; latest review request hit tool quota and is explicitly uncompleted in the report.
- [x] T5 Executed and reported finite qualification limits (not all conditions passed): qualify actual1/2/3/5km/h with one explicit controller configuration, improve roll/heave via stronger preview damping and body-mode feedback, retain force/torque rate limits. Freeze a new matrix after development; report actual speed, matched passive contributions and failed boundaries. User adopted MF52 for expert-two research only.

Review focus: forward/rear event tracking (not merely station at finish), static singularities in torque, no stale-data force kick, no cross-strategy imports, output directory overwrite protection, unknown chassis geometry distinct from suspension jounce. Native runs one DLL process each.

T5 evidence: V1 multispeed26/13 screens, V2 multispeed30/18 screens, independent torque slew3/0 joint diagnostic screens; all native runs full duration. V2 wide4cm1.5/2/2.5kmh and80ms remain failures, so continuous-speed robustness is explicitly unachieved. Research optimization continues to need contact-transition coordination; no full safety acceptance claimed.
