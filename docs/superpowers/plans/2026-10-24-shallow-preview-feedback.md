# Expert two preview feedback implementation

> Execute inline with superpowers:executing-plans and test-driven-development. Spec: ../specs/2026-10-24-shallow-preview-feedback-design.md. User explicitly selected implementation on2026-10-24.

- [ ] T1 RED→GREEN: finite-radius preview, contact/load-aware torque, instance-isolated SI strategy, signed feedback, stale observations, amplitude/rate projection and rear-exit dwell. Files `shallow_press/{preview,traction,config,controller}.py`; tests `test_preview_control.py`.
- [ ] T2 Native worker and independent acceptance; integrate run_strategy shallow branch before wheel-lift imports; preserve native display and camera infrastructure. Exact manifest/config/code hashes, unique paths, full duration; native force sign/response and two initial closed loops. Tests of SI bridge and result gates.
- [ ] T3 Tune only on explicit development cases, freeze parameters then independent full matrix and matched passive ablations; report non-passing cases, numerical and delay limitations. No unsafe full-PASS with unknown geometry.
- [ ] T4 Archive obsolete MPC-only code/evidence with hashes and restore instructions, remove active duplicate paths/tests; regression expert one, full pytest, diff check, fresh review, Git commits and final usage/report.

Review focus: forward/rear event tracking (not merely station at finish), static singularities in torque, no stale-data force kick, no cross-strategy imports, output directory overwrite protection, unknown chassis geometry distinct from suspension jounce. Native runs one DLL process each.
