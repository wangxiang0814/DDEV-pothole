# Contact-Mode MPC for Sequential Wheel Lift — Design Specification

## Purpose and success boundary

The user wants the existing TruckSim/Python four-corner DDEV to crawl across one
right-wheel-track pothole by deliberately lifting the front-right wheel, restoring
four-wheel support, then lifting the rear-right wheel. At every lift, the other
three wheels must carry the vehicle; its projected centre of gravity (CG)
must remain within a validated three-contact support region, while roll,
roll rate and dynamic load-transfer limits are independently satisfied.
CG projection is a quasi-static proxy, not by itself proof of dynamic
stability. This is a classical-control expert policy for future data
generation, not a claim to reproduce unpublished
paper gains.

The reference scenario is 0.20 m deep, 0.80 m long, 0.90 m wide, friction 0.7,
and 2.8 km/h. Success means a native TruckSim run passes measured-dynamics
acceptance; solver completion, state-machine DONE, `safe_stop=False`, a unit-test
pass, and an animation alone are insufficient. If the current plant does not
pass within declared actuator and suspension limits, report the failed
constraint with native evidence and do not weaken it to manufacture success.
Failed search is **not** proof of physical infeasibility; a no-go claim needs a
separately justified reachability bound for the frozen plant.

## Frozen plant and provenance

Freeze the exact `models/corner_module_ddev/run_all.par` bytes and
`single_wheel_deep_pothole/scenario.json` before experiments. At this review,
the generated vehicle SHA-256 is
`a5bf133c1b7312c897861e2da45ab9d7f887a5e765b78d4b8ff201775ecb2f73`;
if it differs at execution, stop and re-baseline. The model is
the independent-suspension `I_I` compact utility truck, not a calibrated heavy
4.4 t vehicle. Its current generated source manifest specifies +160 mm jounce
and -100 mm rebound limits; its vehicle mass, passive springs, damping, tyres,
four independent `IMP_MYUSM` torque inputs and four independent `IMP_FS` force
inputs remain unchanged during the controller comparison. The force and slew
bounds must be frozen and reported as **simulation-authority limits**, not
hardware ratings: no motor or screw ratings have been supplied. `IMP_FS` is an ideal
additive spring-seat force interface, not a validated electromechanical
ball-screw actuator. The bounded actuator approximation must be labelled as
such. Every dataset and run records the source-vehicle hash, scenario hash,
gain/model-training hashes, control configuration, actual limits and software
revision. Do not compare a +121/-61 mm run directly with a +160/-100 mm run as
if only the controller changed. Success here demonstrates the compact model
only; a heavy-duty DDEV claim requires a separately parameterised vehicle,
new identification and new acceptance runs.

The terrain preview begins as simulator truth (`source=ground_truth`). Its
distance, width and depth are provided to the high-level expert scheduler,
with the source recorded. Perception uncertainty experiments are separate and
cannot be described as sensor-verified operation.

## Approaches considered

1. **Static four-by-four gain inversion or diagonal QP:** low cost, but already
   failed in native runs. The preload branch currently sends zero active force,
   the right-front wheel unloads before the CG margin exists, and sign-asymmetric
   flat-road probes show that one static matrix is not a reliable contact model.
2. **Gray-box contact model with scheduled LTV-MPC (selected):** retain the
   discrete expert sequence, identify physical body/unsprung/actuator terms,
   model unilateral tyre contact and stops, then linearise along a scheduled
   contact trajectory. This can plan preload before unloading and anticipate
   load, travel and clearance. A fixed-mode linear model alone cannot predict
   a lift/landing transition; the schedule, guard and relinearisation are part
   of the controller, not optional additions.
3. **Full contact-implicit nonlinear or mixed-integer MPC:** potentially more
   faithful at landing, but model uncertainty and solve cost are unjustified
   before option 2 is validated. This is a later comparison, not the first build.

## Modules and data flow

### 1. Native dynamics experiments and model identification

First demonstrate a *candidate* stationary four-contact posture trim for FR
and RR separately: while all four tyres remain supported, move the predicted
future three-contact stability margin positive; then unload only the target
wheel while the other three retain their load floors, without crossing travel
or force bounds. Continue to a low-speed lip/clearance trial. A failed search
halts MPC development and reports `feasibility_unresolved`; only a validated
conservative bound may justify `physically_infeasible`. Record command and
state trajectories for both positive and negative outcomes. This is a
reachability gate, not a controller acceptance run.

Use a stationary flat-road case and controlled low-speed approach/lift/landing
cases on the frozen plant. Record all four commanded and measured `FsExt`, all
four `Fz`, wheel-centre XYZ, `Jnc/JncR`, body position, roll/pitch/yaw and rates,
speed, tyre contact indicators, torque commands and road height. The current
probe script logs at 50 ms; that file is insufficient to identify a 20 ms
predictor. New experiments log at 5 ms or faster, preserve the 0.5 ms native
solver history around contact events, and retain event timestamps. Include
positive and negative pulses at multiple amplitudes,
simultaneous corner commands, and the five scheduled contact modes:
`FOUR_CONTACT`, `FR_FREE`, `RR_FREE`, `FR_TOUCHDOWN` and `RR_TOUCHDOWN`.
Any non-target support-wheel lift is an **unsafe unmodelled mode** and triggers
recovery, not an allowable scheduled mode. The old single-amplitude flat-road gain
matrix is only a diagnostic reference. The current extended probes have
substantial positive/negative asymmetry; a mode-specific predictor must be
tested against held-out sign, amplitude and multi-corner experiments.

Build a gray-box full-car vertical model: sprung-body heave, roll and pitch;
four unsprung vertical positions/velocities; passive spring/damper and active
spring-seat forces; unilateral tyre normal contact (`gap≥0`, `Fz≥0`,
`gap·Fz=0` in the rigid-contact idealisation, or an identified compliant
law that produces no tensile force and zero force for a separated tyre);
and the jounce/rebound stops. Use exported yaw/lateral state in the coordinated
planar controller. Identify uncertain stiffness, damping, inertia, force delay
and tyre parameters from the frozen native cases. The active-force model is an
ideal `IMP_FS` injection plus *measured or explicitly assumed* command slew;
do not invent ball-screw electrical dynamics. Linearise the gray-box model
along a nominal position/time trajectory to obtain `A[k], B[k], E[k], d[k]`
and an output map for `Fz`, body posture and clearance. The scheduled mode
`q[k+j]` is a candidate schedule from wheel station and the
tyre-envelope/road geometry; predicted gap/load guards determine the actual
switch and touchdown transition in the multi-step predictor. Measured guard
disagreement at runtime invalidates that schedule and forces replanning or
recovery, never silent continuation with the wrong contact mode.
The state remains continuous through contact unless the native data require a
documented impact update. If gray-box identification fails, a lower-order
data-driven predictor is allowed only after the same cross-mode validation;
the old static gain matrix is never silently substituted.

Hold out whole native runs, not adjacent time rows. Validate one-step and
multi-step predictions over the selected MPC horizon, including lift and
landing. Publish per-mode absolute and worst-case errors for support-wheel
`Fz`, roll/pitch and rates, jounce, CG-margin estimate and tyre-envelope
clearance. Before an MPC run, derive constraint-tightening amounts from the
held-out worst-case errors; require each uncertainty bound to be smaller than
its available safety margin. Wrong sign of a support-load change, a missed
contact transition, or error larger than a safety margin rejects the model
regardless of average RMSE. If this gate fails, collect more data or shorten
the valid operating region; do not enable MPC closed loop.

### 2. Discrete expert scheduler

Keep the sequence `APPROACH → FR_PRELOAD → FR_LIFT → FR_CROSS →
FR_TOUCHDOWN → FOUR_CONTACT_RECOVERED → RR_PRELOAD → RR_LIFT →
RR_CROSS → RR_TOUCHDOWN → DONE`. `RECOVER/STOP` are failure states, not
progress states. PRELOAD must actively change the body posture/loads; a timer
with zero force is not preload. While all four tyres carry load, enforce
four-contact load/travel safety and require the **predicted** trajectory to
retain three-contact stability as the target unloads. Do not require a
three-contact *measured* margin as a PRELOAD→LIFT gate. When target `Fz` falls
below the registered lift threshold with a calibrated hysteresis/dwell guard,
activate the measured three-contact margin and three-support load floor;
otherwise use the four-contact envelope.
This distinction prevents the prior chicken-and-egg deadlock while retaining
an abort for an actually unsafe lift. In a nearly symmetric four-wheel stance,
the CG may start on the edge of the future three-wheel triangle; positive
interior margin therefore requires a measured, bounded posture/CG shift and
cannot be assumed from wheel-load redistribution alone. RR_PRELOAD cannot
start until the
front tyre has regained ground contact and the recovery ramp and path/yaw gate
are satisfied.

### 3. Suspension MPC and force application

Start by benchmarking 10, 20 and 40 ms MPC updates on the identified model and
held-out native traces; choose the slowest period that still resolves the
measured force/contact transients. The experiment logger must sample at least
four times faster than that period. Choose the prediction horizon from the
measured force-build and posture-settling times plus the time to the next
contact event, not from a fixed 1.2 s guess. At 2.8 km/h the existing 1.6 m
preload trigger is about 2.06 s before the lip; either the MPC horizon reaches
its required preload terminal state in time or an offline phase reference
sets a deadline that the shorter tracking MPC enforces. Record the resulting
period, horizon, solver p95/p99 time and deadline misses. All four
`IMP_FS` commands are held between MPC updates; the native solver retains its
0.5 ms integration. A failed solve returns a typed status, active constraints
and predicted violation; never reuse a hazardous stale command.

Use hard, uncertainty-tightened actuator, travel, support-load and tip-risk
constraints first; minimise progress/comfort cost only inside that feasible
set. This is a hierarchy, not merely a weighted sum called "lexicographic".
Force and slew limits refer to applied `FsExt` and are checked against the
requested values. During the target-free mode, constrain all three other
`Fz` above their floors; keep target `Fz` near zero only after a safe preload.
For road/tyre clearance, calculate a signed gap from the wheel-centre position,
effective tyre radius and the actual pit-floor/leading/exit-lip geometry,
including the fore-aft tyre envelope. `Fz≈0` alone is not clearance: the tyre
could be at the pit bottom. Landing constraints include target wheel vertical
velocity before lip contact and a calibrated short-window peak load. Keep the
full native-solver trace around impact; a 20 ms optimizer output must not
conceal a sub-sample spike.

CG-margin and lip-gap constraints are nonlinear/piecewise. On each scheduled
mode, linearise their signed-distance functions about the nominal trajectory,
constrain the optimizer to a tested trust region, and relinearise at the next
sample. Tighten each affine constraint by the validated prediction/geometry
error. If the candidate leaves the trust region or a nonlinear recheck finds
it unsafe, reject it and enter RECOVER/STOP. Diagnostic slacks may identify
which safety constraint is infeasible, but a candidate using safety slack is
never applied.

Do not declare the CG-margin estimate exact: present code uses `Xo/Yo`,
attitude, a sprung-mass CG approximation and wheel-centre XY, not combined
vehicle/payload CG and contact-patch coordinates. Reconstruct the full-vehicle
CG and tyre contact points from documented TruckSim outputs/parameters;
compare them with the current approximation on held-out runs. Combine the
calibrated quasi-static triangle margin with all three support loads, roll
angle/rate and a dynamic ground-reaction/COP or ZMP risk indicator. The
1 cm geometric margin cannot be treated as a hard safety constraint until
its worst-case estimation error is substantially below 1 cm; otherwise use a
larger justified margin or report that this acceptance item is unverifiable.

### 4. Torque/yaw and safety layers

Keep four independent torque channels. A separate low-speed allocation QP or
existing traction controller tracks crawl force and uses left/right torque
difference to limit yaw and lateral drift while respecting tyre friction and
slip. Wheel torque at a deliberately lifted corner tends to zero. The
suspension and torque layers exchange predicted normal loads and feasible
traction; neither may treat the other's output as unlimited.

An independent safety supervisor checks *measured* roll and rate, yaw and
rate, lateral offset, all wheel loads, calibrated triangle/dynamic-risk
margin, jounce, force, slew, tyre gap and solver health at each native
solver callback (not only the slower MPC update). A violated hard limit enters
controlled RECOVER or STOP;
the state machine cannot promote recovery to touchdown success. Log every
phase transition, MPC status, active constraint, predicted minimum margins,
requested/applied eight inputs and abort reason alongside native exports.

## Acceptance and comparison

Pre-register the exact longitudinal evaluation windows from the leading and
trailing edge, the effective tyre radius and wheel-centre station. During the
extended evaluation window, FR and RR must each have at least 90% target
unloading coverage (`Fz≤150 N`). Within the pre-registered pit interior,
after the entry lift transient and before the planned exit touchdown, the
target must remain unsupported and have strictly positive uncertainty-tightened
tyre-envelope clearance from pit floor and lips; any measured pit-floor
contact is a failure, not a tolerated percentage. All three other wheels
stay above a load floor derived from prediction uncertainty and dynamic
stability, never below `Fz=300 N`. The calibrated CG projection must
stay at least 0.01 m inside the support triangle while the target is
unsupported, together with a passing dynamic-risk and wheel-load criterion.
If CG/contact-point uncertainty exceeds this 0.01 m margin, the geometric
item fails verification rather than silently being skipped. Roll ≤7°,
yaw ≤2°, lateral drift ≤0.15 m; travel stays inside the frozen run's actual
per-corner limits. The current research impact threshold is target-wheel
load ≤4 times the largest static corner load within a registered landing
window; also report peak vertical velocity, roll rate and body acceleration.
These thresholds are research acceptance settings, not certified limits or
paper-derived ratings. Exact time/station windows and limits must be frozen
*before* A/B runs. Verify FR touchdown and four-wheel recovery precede any
commanded RR lift; passive RR unloading from the road is not proof of a
controlled rear lift.

Run identical source-vehicle and scenario hashes and identical force/slew
authority for: current diagonal allocator, validated scheduled MPC, and MPC
without preview. Also perform depth and parameter-uncertainty sweeps without
mixing vehicle versions. Record failures as well as successes. Only a native
trace passing every pre-registered physical criterion permits a video to be
labelled a successful traversal. A passing unit test or a single favourable
   camera view is not sufficient.

## Execution gates and required evidence

The following gates are sequential. Each gate writes a machine-readable
result and a short human audit note; a failed gate stops downstream claims.

1. **Plant freeze and observability:** verify the generated-vehicle and
   scenario hashes; demonstrate independent torque and `IMP_FS` sign/response
   on all four corners; document units, coordinate frames, command latency,
   contact-point and full-CG reconstruction, tyre radius and all physical
   limits. Output: frozen manifest, native traces and signal dictionary.
2. **Reachability before MPC:** on the frozen plant, attempt FR and RR
   four-contact trims, target unloading, three-support loading and safe
   clearance with bounded commands. Report admissible traces and the first
   binding constraint for each failed attempt. A failed numerical search is
   `feasibility_unresolved`, not a physical no-go. If the required 1 cm CG
   margin cannot be measured more accurately than its size, stop and
   recalibrate before asserting success.
3. **Predictor qualification:** identify all scheduled contact modes from
   event-resolved native data. Hold out complete runs with different pulse
   signs, magnitudes and corner combinations. Publish worst-case multi-step
   errors and tighten every hard constraint by those errors. Reject the model
   for wrong load-response sign, missed mode switch or error exceeding the
   available safety margin.
4. **Controller-in-the-loop qualification:** test preload-to-free and
   free-to-touchdown switches, solver failure, trust-region rejection and
   safety abort on replay/synthetic cases before connecting the controller
   to TruckSim. The chosen MPC period and horizon must meet measured
   contact-response and solve-time deadlines. No safety-slack solution may
   reach a native actuator.
5. **Native sequential trial:** progress from isolated FR and isolated RR
   trials to FR→four-contact recovery→RR on the exact frozen plant and pit.
   Judge the native dynamics, not scheduler labels: support loads, calibrated
   CG/dynamic risk, clearance, landing impulse, roll/yaw/path and independent
   commands must pass the pre-registered acceptance windows. Preserve every
   failing trace. Only then compare with baseline and export a success video.

The implementation can be *attempted* with TruckSim's independent corner
force and torque interfaces, but the prescribed 0.20 m-pit success is not
guaranteed by architecture alone. Gate 2 determines whether a safe trajectory
is found within the frozen model and control authority; Gates 3–5 determine
whether the proposed expert controller can reproduce it robustly.

## Failure handling and explicit stop conditions

- If stationary posture/lift feasibility is not demonstrated, stop at
  `feasibility_unresolved`; do not infer that a stronger motor must solve it.
- If model prediction fails held-out contact-mode validation, do not run its
  MPC closed loop; preserve the run and report which output/mode failed.
- If the constrained planner is infeasible before the pit, reduce speed or
  stop; do not command a partial lift hoping the supervisor will save it.
- If a contact transition deviates from prediction, switch to RECOVER and
  re-identify that mode rather than hiding it with gain changes.
- If closed-loop trials fail, report `controller_failed_on_frozen_plant` with
  the active constraints. Do not call the plant physically impossible unless
  an independently validated conservative reachability bound rules out all
  admissible trajectories. Any change to force authority, travel, mass or pit
  geometry requires a separate sensitivity run and explicit user decision.
