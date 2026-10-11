# Passive problem study (2026-10-12)

Report: `docs/PASSIVE_PROBLEM_STUDY_20261012.md`.

- 18 immutable original baseline sources, 20 full reference-point diagnostic sources, 38 phase analyses.
- 13 new native simulations, all full 30 seconds; all rear axles passed the trailing envelope boundary. No video playback/export.
- `phase_summary.json`: per-axle and post-exit metrics, wheel-by-wheel episodes, source hashes and censoring.
- `baseline_table.md`: all original conditions; +1 s censoring is explicit. Compare speeds with common +0.5 s JSON metrics.
- `case_index.md`: run names and key crossing/stopping times for future playback.
- `convergence.json` / `convergence_table.md`: grid, step, stiffness and zero-force controls. All six pure step/grid numerical screens failed; stiffness comparisons are constitutive sensitivity, not convergence.
- `contact_exact_geometry.json`: exact planar-rectangle clearance on five representative contact trajectories at the corresponding assumed clearance.
- `representative_phases.png` / `contact_sensitivity.png`: visually inspected scientific figures.
- `verification.json`: planned completion, actual model step, 1 ms sampling, shared base identity and tests.

Control remains passive additional suspension force and equal wheel torque speed PI, with straight steering feedback. No shallow expert control algorithm is implemented. Chassis geometry/contact parameters are assumptions; contact loads and deep-wide-pit numerical peaks are not converged real-vehicle predictions.
