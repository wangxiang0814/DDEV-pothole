# Shallow MPC predictor route gate, 2026-10-13

This evidence verifies native model extraction and tests predictors; it is not an implemented expert or MPC closed-loop safety PASS.

- `collection.json`: frozen 10 training / 18 holdout / 2 timestep native jobs.
- `dynamic_model.json`: classical ARX2 fitted using training only, before holdout prediction metrics were read.
- `verified_analysis/prediction_summary.json`: authoritative independent gate, including critical edges and rear clearance.
- `verified_analysis/native_linearization.m`, `native_linear_model.json`: native 67-state, 9-input local model at flat settled 5.001 s.
- `model_diagnostics_verified.json`: physical local responses, diagnostic-only oracle future-pose road and timestep sensitivity.
- `prediction_table.md`, `prediction_examples.png`, `case_index.md`: human inspection.
- `verification.json`: final native source, vehicle identity and input/timing checks.

Original root analysis and `model_diagnostics.json` are retained as intermediate audit artifacts. The verified analysis corrects qualification/overwrite guards without refitting the model or changing numeric thresholds. Per-case NPZ arrays contain 200 ms target predictions, true outputs, persistence and edge masks; output channels are listed in summary. False-support sample counts are summed wheel-node targets, not independent accidents.

All 18 holdouts fail wheel-load/contact prediction. Exact future applied commands are supplied for plant prediction testing; formal road preview uses current pose/speed only. Oracle preview is separately labelled and never used to tune or release the controller. No original or wheel-lift model/config/history was overwritten.
