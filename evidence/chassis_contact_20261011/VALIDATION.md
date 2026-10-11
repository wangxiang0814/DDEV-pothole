# Validation record

Executed with F:/anaconda3/python.exe (Python 3.8) in the project workspace.

- Full suite: `PYTHONPATH=src python -m pytest -q`: 504 passed, 10 third-party deprecation warnings, 52.72 s.
- `git diff --check`: passed.
- `scripts/summarize_chassis_contact.py`: assertions passed for all 90 historical CSV identities, all 9 native completion states, extended model identities, common base vehicle, sub-micrometre reference-frame agreement, exact 126-channel zero-force reproduction, 100 N s force probe and nonpositive friction work.
- Two 6-second force-interface checks are excluded from the seven full 30-second simulation count.
- `scripts/analyze_chassis_clearance.py --run runs/shallow_press/wide/chassis_contact_20261011_wide_d200_h150 --output evidence/chassis_contact_20261011/contact_geometry_check`: five clearance hypotheses completed on the contact-force trajectory; the matching 15 cm hypothesis is marked COMPLIANT_POINT_MODEL_TRAJECTORY.
- Independent code review found no blocking geometry/force-port error; point-grid convergence, corner energy behaviour and unmeasured geometry remain explicit limitations.
- Plot visually inspected. No visualizer playback or video export performed.

Current audit code hashes identify the final implementation, not the earlier run generation revision. Native run records and earlier evidence are immutable. Geometry, stiffness, damping and body friction remain assumptions and do not certify real vehicle safety or a strategy-switch threshold.
