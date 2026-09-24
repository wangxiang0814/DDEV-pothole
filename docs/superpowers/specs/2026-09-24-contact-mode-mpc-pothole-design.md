# Contact-Mode MPC for Sequential Wheel Lift — Design Specification

## Purpose and success boundary

The user wants the existing TruckSim/Python four-corner DDEV to crawl across one
right-wheel-track pothole by deliberately lifting the front-right wheel, restoring
four-wheel support, then lifting the rear-right wheel. At every lift, the other
three wheels must carry the vehicle and its projected centre of gravity (CG)
must remain within a measured support triangle. This is a classical-control
expert policy for future data generation, not a claim to reproduce unpublished
paper gains.

The reference scenario is 0.20 m deep, 0.80 m long, 0.90 m wide, friction 0.7,
and 2.8 km/h. Success means a native TruckSim run passes measured-dynamics
acceptance; solver completion, state-machine DONE, `safe_stop=False`, a unit-test
pass, and an animation alone are insufficient. If the current plant cannot pass
within declared actuator and suspension limits, report the failed constraint
with native evidence and do not weaken it to manufacture success.

## Frozen plant and provenance

Start from `models/corner_module_ddev` as it exists at design approval. It is
the independent-suspension `I_I` compact utility truck, not a calibrated heavy
4.4 t vehicle. Its current generated source manifest specifies +160 mm jounce
and -100 mm rebound limits; its vehicle mass, passive springs, damping, tyres,
four independent `IMP_MYUSM` torque inputs and four independent `IMP_FS` force
inputs remain unchanged during the controller comparison. `IMP_FS` is an ideal
additive spring-seat force interface, not a validated electromechanical
ball-screw actuator. The bounded actuator approximation must be labelled as
such. Every dataset and run records the source-vehicle hash, scenario hash,
gain/model-training hashes, control configuration, actual limits and software
revision. Do not compare a +121/-61 mm run directly with a +160/-100 mm run as
if only the controller changed.

The terrain preview begins as simulator truth (`source=ground_truth`). Its
distance, width and depth are provided to the high-level expert scheduler,
with the source recorded. Perception uncertainty experiments are separate and
cannot be described as sensor-verified operation.

## Approaches considered

1. **Static four-by-four gain inversion or diagonal QP:** low cost, but already
   failed in native runs. The preload branch currently sends zero active force,
   the right-front wheel unloads before the CG margin exists, and sign-asymmetric
   flat-road probes show that one static matrix is not a reliable contact model.
2. **Mode-scheduled linear time-varying MPC (selected):** retain the discrete
   expert sequence and use a validated dynamic predictor within each contact
   mode. It can plan a preload before unloading and enforce anticipated force,
   slew, wheel load, travel, attitude and clearance constraints. It is simpler
   and more diagnosable than a single large mixed-integer optimizer.
3. **Full contact-implicit nonlinear or mixed-integer MPC:** potentially more
   faithful at landing, but model uncertainty and solve cost are unjustified
   before option 2 is validated. This is a later comparison, not the first build.

## Modules and data flow

### 1. Native dynamics experiments and model identification

Use a stationary flat-road case and controlled low-speed approach/lift/landing
cases on the frozen plant. Record all four commanded and measured `FsExt`, all
four `Fz`, wheel-centre XYZ, `Jnc/JncR`, body position, roll/pitch/yaw and rates,
speed, tyre contact indicators, torque commands and road height at the native
logging interval. Include positive and negative pulses at multiple amplitudes,
simultaneous corner commands, and the three relevant contact modes:
`FOUR_CONTACT`, `FR_FREE`, `RR_FREE`. The old single-amplitude flat-road gain
matrix is only a diagnostic reference. The current extended probes have
substantial positive/negative asymmetry; a mode-specific predictor must be
tested against held-out sign, amplitude and multi-corner experiments.

Fit a discrete predictor at a declared controller sample period (initially
20 ms): `x[k+1] = A_q x[k] + B_q u[k] + E_q r[k] + d_q` plus a separately
validated output map for contact load and wheel/lip clearance. `q` is the
observed or scheduled contact mode. Candidate state channels are body heave,
roll and pitch with rates, four jounce positions/rates, measured wheel loads,
applied actuator forces and planar yaw/lateral state. If raw state regression
overfits or is not observable from the exports, use a documented lower-order
state/output predictor; never silently substitute the old static matrix.

Hold out whole native runs, not random adjacent time rows. Validate multi-step
predictions over at least the planned preload-to-lift horizon, including the
contact transitions. Publish per-mode errors for `Fz`, CG triangle margin,
roll/pitch, jounce and clearance. A predictor that gives the wrong sign of any
support-wheel load or misses a contact transition is rejected regardless of
average RMSE. If the contact-mode data are insufficient, collect more data
before enabling MPC in closed loop.

### 2. Discrete expert scheduler

Keep the sequence `APPROACH → FR_PRELOAD → FR_LIFT → FR_CROSS →
FR_TOUCHDOWN → FOUR_CONTACT_RECOVERED → RR_PRELOAD → RR_LIFT →
RR_CROSS → RR_TOUCHDOWN → DONE`. `RECOVER/STOP` are failure states, not
progress states. PRELOAD must actively establish a positive *predicted and
measured* CG/support margin before target-wheel unload; a mere timer with zero
force is not preload. Do not demand a three-point support margin before the
target wheel begins unloading while all four tyres carry load, but require a
safe predicted trajectory as it unloads. RR_PRELOAD cannot start until the
front tyre has regained ground contact and the recovery ramp and path/yaw gate
are satisfied.

### 3. Suspension MPC and force application

At 20 ms intervals (initial setting), solve a receding-horizon constrained
quadratic program for the four `IMP_FS` commands. Initial horizon is 1.2 s;
preview/scheduler triggers PRELOAD early enough for the measured actuator and
body-settling time. The first command is held between MPC updates; the native
solver still integrates at its original step. The solver must return a finite
feasible solution or a typed failure with active constraints and predicted
violation; never silently reuse a hazardous stale command.

The objective is lexicographic in effect: support and tip safety first,
travel/clearance and landing safety second, yaw/path stability third, then
wheel-lift completion, speed and comfort. Enforce configured force and slew
bounds, per-corner jounce/rebound limits with margin, three support-wheel load
floors, an estimated CG triangle margin, target-wheel unloading only after
preload, and tyre/exit-lip clearance. `Fz≈0` alone is not proof that a wheel
cleared the hole: it could be resting on the pit bottom. At landing, limit
vertical velocity and forecast load impulse. Small slacks may be used to
diagnose infeasibility, but the safety supervisor rejects any candidate that
uses safety slack in applied control.

Do not declare the CG-margin estimate exact: present code derives it from
`Xo/Yo`, attitude, estimated CG height and wheel-centre XY rather than tyre
contact-patch coordinates. Calibrate its geometry and uncertainty against
TruckSim exports; reserve an uncertainty margin in the constraint.

### 4. Torque/yaw and safety layers

Keep four independent torque channels. A separate low-speed allocation QP or
existing traction controller tracks crawl force and uses left/right torque
difference to limit yaw and lateral drift while respecting tyre friction and
slip. Wheel torque at a deliberately lifted corner tends to zero. The
suspension and torque layers exchange predicted normal loads and feasible
traction; neither may treat the other's output as unlimited.

An independent safety supervisor checks *measured* roll, yaw, lateral offset,
all wheel loads, triangle margin, jounce, force, slew and solver health at
every control update. A violated hard limit enters controlled RECOVER or STOP;
the state machine cannot promote recovery to touchdown success. Log every
phase transition, MPC status, active constraint, predicted minimum margins,
requested/applied eight inputs and abort reason alongside native exports.

## Acceptance and comparison

The existing 0.20 m acceptance baseline is: FR and RR each traverse the
correct pit footprint with at least 90% target unloading coverage
(`Fz≤150 N`) during the interior; all three other wheels stay at `Fz≥300 N`;
CG projection stays at least 0.01 m inside the calibrated support triangle
while the target is unsupported; roll ≤7°, yaw ≤2°, lateral drift ≤0.15 m;
travel stays inside the current model's actual per-run limits; and post-pit
target-wheel load stays below four times the largest static corner load.
Additionally verify wheel-to-pit-bottom/exit-lip clearance and that FR
touchdown/four-wheel recovery precedes RR lift. These numerical thresholds are
research acceptance settings, not certified safety limits. Any tightening or
relaxation must be recorded before an A/B run.

Run identical scenario/vehicle hashes for: current diagonal allocator,
validated mode-scheduled MPC, and MPC without preview. Record both failures
and successes. Only after a native trace passes all checks should the video
be labelled a successful traversal. A passing unit test is necessary for code
integrity but not sufficient for the physical claim.

## Failure handling and explicit stop conditions

- If model prediction fails held-out contact-mode validation, do not run its
  MPC closed loop; preserve the run and report which output/mode failed.
- If the constrained planner is infeasible before the pit, reduce speed or
  stop; do not command a partial lift hoping the supervisor will save it.
- If a contact transition deviates from prediction, switch to RECOVER and
  re-identify that mode rather than hiding it with gain changes.
- If force, stroke or clearance remains physically infeasible after validated
  model and controller trials, report a no-go *for that frozen plant and
  scenario*. Any change to actuator capacity, travel, mass or pit geometry
  requires a separate sensitivity run and explicit user decision.
