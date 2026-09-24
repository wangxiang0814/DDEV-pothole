# Contact-Mode MPC Gate 2a: Native Geometry and Clearance Audit

Date: 2026-09-24. Scope: the frozen 1,360 kg compact utility truck and the
0.20 m deep, 0.80 m long, 0.90 m wide right-track pit. This is a **geometry
gate**, not a successful wheel-lift or reachability claim. It does not imply
validation of a heavy-duty truck.

## Native channels and coordinate frame

The TruckSim 2019 supplied `Animator/Arrows/Arrows_fbf40567-16ac-4434-8406-9d4bfb88c9e0.par`
places skid-mark reference frames at `Xctc_?i/Yctc_?i/Zgnd_?i` and explicitly
calls these tyre-ground contact coordinates. Its supplied export dataset
`IO_Channels/O_Channels/Export_fc250dd9-cae3-4275-874a-94df128933bc.par`
labels `XCG_TM/YCG_TM` the global X/Y coordinates of the instant *total
vehicle* CG. In an isolated generated model copy, we appended these ten
exports after the existing 116 channels and set `PORTS_EXP=126`; all eight
actuator imports, physical parameters and road remained unchanged. Native
solver echoes confirm each new channel and its metre unit. The original
Task-1 frozen case and its 116-channel contract remain untouched.

`native_support_geometry()` refuses absent/nonfinite channels; it never
substitutes wheel-centre `X_?/Y_?` for contact coordinates. The independent
mass reconstruction uses sprung mass, all three payload centres, four
unsprung wheel positions and measured body roll/pitch/yaw. The Echo's
`LX_CG_TL` is a local-body distance; direct comparison with global XY
without an attitude transform creates a spurious ~16 mm initial discrepancy.

| Native case (5 ms CSV) | Rows | Worst mass-reconstructed vs `XCG_TM/YCG_TM` | Largest loaded-wheel contact vs wheel-centre offset |
|---|---:|---:|---:|
| Moving pit, zero actuator command | 2,201 | 0.000000938 m | 0.252997 m |
| Flat, FR spring-force +1,000 N pulse at 0.25–0.50 s | 201 | 0.000000092 m | 0.010533 m |

CSV evidence: `runs/contact_mpc_moving_coordinate_probe_a/cg_contact_probe.csv`
and `runs/contact_mpc_contact_coordinate_probe_a/fr_force_cg_contact_probe.csv`.
Only wheel samples with `Fz > 300 N` enter the contact-offset maxima; a
geometric point reported for an unloaded tyre is not a support vertex.
The moving baseline even includes 50 FR-unloaded samples with all three
other wheel loads above 300 N while the native CG lies **outside** the
FL–RL–RR triangle (signed margins −0.04346 to −0.00312 m). This is an
unsafe no-control reference, not evidence of a lift strategy.

The direct native contact and CG outputs remove the large wheel-centre
substitution error. Their 9-significant-digit CSV quantisation around a
100 m station is at most about 0.000001 m per coordinate; the independent
CG reconstruction discrepancy is below 0.000001 m across these two runs.
For subsequent internal-simulation constraints, reserve a conservative
0.001 m geometry allowance (not a real-world sensing guarantee), below the
gate's 0.005 m maximum and leaving 0.009 m of a nominal 0.010 m triangle
margin. External perception, tyre-model and parameter uncertainty are not
bounded by this audit and require separate validation.

## Physical pit and tyre envelope

The collision road is the `ROAD_DZ_CARPET 2D_LINEAR` table in `run_all.par`,
with longitudinal knots 101.10, 101.15, 101.85 and 101.90 m. The Animator
mesh is visual only. A 21-point circular tyre-envelope scan is **unsafe as a
clearance certificate**: among 360 moving FR-wheel samples near the pit it
overestimated the true piecewise-linear minimum by up to 0.026936 m. The
new `pothole_tyre_envelope_gap_m()` evaluates each road-segment boundary
and the exact stationary point of the tyre-circle-minus-road gap. Tests
compare it with a 10,001-point reference at both lips and multiple lateral
locations. Future clearance checks must use this exact routine (or an
independently validated conservative bound), not the 21-point generic scan
or `Zgnd` alone.

## Gate disposition and next step

**Geometry gate passed for these native TruckSim outputs and this frozen
plant**, conditional on using the ten added outputs in an isolated,
versioned case. `geometry_unverified` remains the correct status if a run
lacks those outputs, changes mass/road/tyre parameters without re-audit, or
uses wheel centres as support contacts. Next: bounded FR/RR reachability
search. This report makes no claim that the 1 cm margin can actually be
achieved, or that the vehicle has completed FR→RR wheel-lift crossing.
