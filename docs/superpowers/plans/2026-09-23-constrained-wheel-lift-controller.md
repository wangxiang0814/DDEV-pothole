# Constrained Wheel-Lift Controller Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement and validate a reproducible front-right then rear-right deep-pothole wheel-lift expert controller for the existing four-corner DDEV.

**Architecture:** Refine the phase manager into observable preload/lift/cross/touchdown states, generate feasible three-wheel references from the actual vehicle, and close classic feedback loops through bounded four-corner suspension and torque allocators. A safety supervisor gates progression and performs smooth recovery when load, travel, attitude, path, or slip margins are violated.

**Tech Stack:** Python 3, unittest/pytest, TruckSim 2019 native solver, existing `ddevsim` co-simulation API.

**Spec:** `docs/superpowers/specs/2026-09-23-constrained-wheel-lift-controller-design.md`

## Global Constraints

- Preserve the current `Compact Utility Truck (I_I)` four-independent-suspension DDEV and its eight actuator channels.
- Use a 0.01 s controller period and 2.8 km/h reference speed.
- Use deterministic dependency-free bounded allocation suitable for later Simulink porting.
- Do not claim numerical reproduction of unpublished paper gains.
- Preserve the approved textured road, field, pothole, sky, camera, and cone visualization.
- Work in the current feature branch because its uncommitted control/visual baseline is required; never stage unrelated generated videos.

## Review Focus

- A target wheel reaches the pit station before preload feasibility is established: remain in preload/hold and never unload a second wheel.
- A support wheel load drops below its floor for more than 50 ms: enter recovery and do not advance to rear lift.
- Yaw correction needs the sign unavailable to positive-only drive: allocate bounded negative torque without driving the lifted wheel.
- Wheel position is longitudinally over the pit but laterally outside it: do not claim crossing or advance touchdown from station alone.
- Suspension approaches either travel stop during recovery: taper the offending force without reversing its sign.

---

### Task 1: Scenario Geometry Contract

**Files:**
- Modify: `src/ddevsim/pothole_case.py`
- Modify: `models/corner_module_ddev/single_wheel_deep_pothole/scenario.json`
- Test: `tests/test_pothole_case.py`

**Interfaces:**
- Produces: `PotholeScenario.width_m == 0.90` for the corner-module case and `covers_point(station_m, lateral_m)` as the authoritative wheel-in-pit test.
- Consumes: existing tire/track geometry encoded by the selected model.

- [ ] Add a failing test asserting a 0.90 m right-track pit covers `y=-0.63` but excludes the complete left tire footprint with at least 0.70 m clearance.
- [ ] Run `PYTHONPATH=src python -m pytest tests/test_pothole_case.py -q` and confirm the width assertion fails at 0.60 m.
- [ ] Change only the corner-module scenario override and checked-in scenario JSON to 0.90 m; keep the visual material and mesh settings unchanged.
- [ ] Run the test file and confirm it passes.
- [ ] Commit the scenario contract without generated video artifacts.

### Task 2: Observable Phase Manager and Safety Gates

**Files:**
- Modify: `src/ddevsim/expert_controller.py`
- Modify: `tests/test_expert_controller.py`

**Interfaces:**
- Produces: phase constants for approach, FR preload/lift/cross/touchdown, RR preload/lift/cross/touchdown, done; `PhaseObservation`; transition reasons and safety mode in `step_summary()`.
- Consumes: `PotholeScenario.covers_point`, wheel XYZ/load channels, yaw/yaw-rate/lateral states, and controller time.

- [ ] Add failing table-driven tests proving one-stage-per-call progression, lateral crossing checks, front four-wheel recovery before rear preload, and support-load timeout recovery.
- [ ] Run the focused tests and verify failures name the missing phases/gates.
- [ ] Introduce the nine-stage constants and a single `_phase_observation()`/`_advance_step()` transition function with explicit conditions and reason logging.
- [ ] Add `CONTINUE/HOLD/RECOVER/STOP` safety modes, 50 ms support-load debounce, and smooth recovery entry without instant force clearing.
- [ ] Run all expert-controller tests and then the full suite.
- [ ] Commit the phase manager and gates.

### Task 3: Bounded Four-Corner Suspension Allocation

**Files:**
- Create: `src/ddevsim/bounded_allocation.py`
- Modify: `src/ddevsim/expert_controller.py`
- Modify: `tests/test_expert_controller.py`
- Create: `tests/test_bounded_allocation.py`

**Interfaces:**
- Produces: `bounded_weighted_least_squares(matrix, target, lower, upper, effort_weight) -> list[float]`; per-cycle suspension force allocation and constraint-margin telemetry.
- Consumes: measured local gain matrices, load/posture references, previous command, force/slew/travel bounds.

- [ ] Add failing allocator tests with hand-derived diagonal and coupled fixtures, including active bounds and infeasible targets.
- [ ] Run the allocator tests and confirm import/function failures.
- [ ] Implement deterministic active-set enumeration for four variables with regularized least squares and a residual/active-bound result.
- [ ] Add failing controller tests proving the lifted wheel unload target cannot push another support wheel below its floor and force commands obey per-cycle slew/travel bounds.
- [ ] Replace long-held static inverse commands with per-cycle constrained incremental allocation while retaining classic PI/PD outer feedback and S-curve references.
- [ ] Run allocator, controller, then full test suites.
- [ ] Commit suspension allocation.

### Task 4: Bidirectional Torque and Path-Stability Allocation

**Files:**
- Modify: `src/ddevsim/expert_controller.py`
- Modify: `tests/test_expert_controller.py`

**Interfaces:**
- Produces: four torque commands tracking total crawl torque and yaw moment under `mu*Fz*R`, slip, lift, absolute, and slew bounds.
- Consumes: speed, yaw, yaw rate, lateral position/velocity, per-wheel load/slip, and active lifted corner.

- [ ] Add failing tests for yaw-rate/lateral-error feedback, negative corrective torque, lifted-wheel zero torque, friction bounds, and slip derating.
- [ ] Run focused tests and confirm positive-only legacy allocation fails.
- [ ] Implement two-equation bounded weighted allocation for total torque and yaw moment, allowing limited regenerative torque on grounded wheels.
- [ ] Run focused and full suites.
- [ ] Commit torque/path allocation.

### Task 5: Full Co-simulation, Metrics, and Evidence

**Files:**
- Modify: `scripts/run_expert_pothole.py`
- Modify: `src/ddevsim/channels.py` if telemetry fields are absent
- Modify: `tests/test_channel_contract.py`
- Modify: `runs/corner_module_expert_pothole/manifest.json`
- Create: `reports/constrained_wheel_lift_validation.md`

**Interfaces:**
- Produces: native TruckSim history, detailed CSV, manifest metrics, validation report, and a review video using the existing visual configuration.
- Consumes: completed controller, scenario, TruckSim solver and saved visual assets.

- [ ] Add failing metric tests for actual FR/RR pit intersection, ordered phase completion, support-wheel load duration, yaw/lateral/roll limits, travel margin, touchdown speed/load, slip and speed bands.
- [ ] Extend telemetry/manifest generation only as needed to compute those metrics.
- [ ] Rebuild the case, run static calibration and actuator checks, then run the native expert simulation.
- [ ] Analyze the generated CSV. If a metric fails, diagnose from state/constraint traces and fix through a new RED→GREEN regression cycle rather than weakening safety gates.
- [ ] Export a TruckSim Visualizer video only after the dynamic sequence is accepted; keep the approved textures and camera.
- [ ] Run `PYTHONPATH=src python -m pytest -q`, record exact result and generated artifact paths in the report.
- [ ] Commit source, tests, manifest and report; do not commit transient probe files or redundant videos.

## Self-review

- Spec coverage: geometry, phase sequencing, suspension allocation, torque/yaw control, safety recovery, telemetry and native simulation each map to one task.
- Placeholder scan: no implementation step relies on TBD/TODO or unspecified behavior.
- Type consistency: Task 1 produces the geometry predicate used by Task 2; Task 2 phase state feeds Tasks 3/4; Tasks 3/4 telemetry feeds Task 5.
- Review-focus coverage: each of the five failure modes has an explicit test in its owning task.

