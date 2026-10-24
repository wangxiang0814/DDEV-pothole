# Native five-camera demonstration capture

Scope: independent acquisition for the existing tuned I_I vehicle. No controller,
plant, integration-step, acceptance-threshold or training-code changes.
The shallow_press research controller is implemented with its own worker and
recording configuration; only wheel_lift currently supplies fully accepted
expert episodes. A shallow SIMULATION_PASS or comparison plant is not a fully
accepted, trainable episode. Strategy and plant identities travel with captures.

## Implementation plan and synchronization contract

1. Probe installed native cameras and Shared Buffer API v2. Use the header shipped
   with the DLL, opaque handles, exclusive locking, finite connection deadlines.
2. Add configurable body-mounted front/FL/FR/RL/RR cameras. Render the existing
   run-owned terrain and native vehicle history; retain hidden driver.
3. Observe the original integration boundary without altering commands. Save
   pre-step physical observations and the exact commands submitted for that step,
   including distinct observation and action-end timestamps. Preserve raw units
   and SI conversion metadata. Keep inaccessible/privileged state out of policy inputs.
4. After native solver termination, automatically render the same history in VS
   Visualizer. Five buffers must agree on native simulation timestamp; lock/copy
   each buffer, reject mixed frames and duplicate/stale timestamps. No wall-clock
   timestamp is used to join images with actions.
5. Join native image time to bracketing recorded states (linear interpolation)
   and the command applicable to that integration interval (zero-order hold).
   Report the interpolation bracket and action interval. Retain the high-rate
   record so downstream users can choose their own resampling.
6. Store PNG RGB, per-frame native camera pose/FOV/intrinsics, configured body
   extrinsics, units/action schema, instruction, scene, hashes, expert result and
   collection-quality result in an exclusive episode directory.
7. Verify temporal rejects and causal action association in unit tests. Run the
   same recommended episode with acquisition off/on, compare native trajectories,
   and inspect five views in settle/lift/crawl/lower phases.

This is two-pass simulation-time synchronized acquisition, not a live camera
feedback implementation. The local Camera Sensor S-function manual explicitly
warns about live timestamp correspondence and asynchronous delay. The offline
native timestamp is the authoritative image time. Rendering throughput changes
wall-clock generation cost, never the expert's simulated timing.

Native frame cadence is asynchronous. Requested FPS defines sampling bins;
actual timestamps and achieved cadence are saved, with gaps rejected against a
configured bound. Do not label wall-clock frames as exact fixed-grid samples.

## Local interface evidence

- VS Visualizer 32-bit file version 2019.0.76261; 64-bit runtime log 2019.0.10722.
- Installed Help/Memos/Camera_sensors.pdf supports multiple named buffers.
- Help/Memos/VsSensor_S_function.pdf pp.5–8 documents asynchronous rendering
  and simulation-time limitations in live mode.
- Programs/vs_sb/vs_SharedBuffer.h: API v2, RGB type=1, frame timestamp, FOV,
  global camera pose; explicit lock/unlock and disconnect.
- Actual DLL reports default shared-memory format 8; do not hard-code format 2
  merely because the API version is 2.
- Native five-view RGB read succeeded on 2026-10-20; camera placement and full
  episode acceptance remain subject to the separate validation report.

Official references:
https://www.carsim.com/users/pdf/release_notes/trucksim/TruckSim2019_1_New_Features.pdf
https://www.carsim.com/products/supporting/vehiclesim/vs_api.php
