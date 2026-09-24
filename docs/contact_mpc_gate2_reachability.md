# Contact-Mode MPC Gate 2b: Bounded FR/RR Reachability

Date: 2026-09-24. **Disposition: `feasibility_unresolved`.** This finite
search did not produce a safe FR or RR trajectory. It does not establish
physical infeasibility. Per the approved sequential plan, predictor/MPC and
closed-loop success claims stop at this gate unless a new reachability
candidate is justified and validated.

## Frozen experimental conditions

- Physical plant and pit: the Task-1 source hashes, 1,360 kg compact utility
  truck, right-track 0.20 m-deep × 0.80 m-long × 0.90 m-wide pit, μ=0.70.
  The plant has four independent `IMP_FS` spring-seat force imports and four
  independent wheel torque imports; this scan commanded zero wheel torques.
- Only observation channels were added: four `Xctc`, four `Yctc`, and
  `XCG_TM/YCG_TM` (126 exports total). Raw 5 ms CSV and native TruckSim
  histories were retained for each candidate; all 48 FR and 48 isolated RR
  candidates also triggered full-rate 0.5 ms reruns with separate histories.
- Applied force authority ±17,626.744 N per corner; slew ≤44,066.861 N/s.
  The registered profiles use 0.5/0.8 s support preload ramps, 1.0 s target
  ramps, support-right bias 0/6/12/17 kN, target command −4/−8/−12 kN and
  crawl references 2.8/2.0 km/h. A profile clips requests to both limits.
  The profile targets only one lifted wheel per trial; actual unintended
  support-wheel lift is counted as failure.
- Native acceptance screen in the pre-registered pit interior
  `[101.45, 101.55] m` of the target wheel centre: target `Fz≤150 N`, each
  other `Fz≥300 N`, native contact-point/instant-CG triangle margin ≥0.010 m
  after a 0.001 m geometry allowance, exact tyre/pit-envelope gap >0 after
  a 0.001 m allowance, roll ≤7°, yaw ≤2°, lateral offset ≤0.15 m and all
  jounces strictly inside [−0.100, +0.160] m. This is a **search screen**, not
  the full 90% traversal acceptance test.

RR isolation correction: the first 16 RR trials from the ordinary initial
station had the uncontrolled FR tyre traverse the same pit first. They are
preserved under `runs/contact_mpc_reachability_rr_a/` but **excluded** from
the RR result. The corrected RR run starts at body station 102.21 m: its
initial front wheel centre is 102.217882 m, leaving 0.054882 m between the
front tyre envelope and the exit lip at 101.90 m. This changes initial
condition only, not the pit geometry or vehicle physical parameters.

## Native results

| Trial | Admissible / registered | First binding constraint counts | Native evidence |
|---|---:|---|---|
| FR approach | 0 / 48 | target not unloaded: 33; another support wheel unloaded: 15 | `runs/contact_mpc_reachability_fr_a/` |
| Isolated RR approach | 0 / 48 | target not unloaded: 14; another support wheel unloaded: 29; CG outside safe triangle: 5 | `runs/contact_mpc_reachability_rr_isolated_a/` |

`registered_scan.json` was written before solver calls; `results.json`
records every candidate and first failure, and candidate directories contain
the input/output CSVs and independent native history. The two complete scan
directories together occupy about 3.4 GB; no histories were deleted.

The closest examples remain unsafe. FR case 21 has a best pit-interior
conservative CG margin of about −0.09 m, target peak 1,318 N and a support
wheel at zero. Isolated RR case 21 achieves target `Fz=0` and a positive
~0.05 m pit gap, but another support wheel is at zero and the CG margin is
about −0.05 m. These examples show why target unloading or clearance alone
cannot certify three-wheel stability.

## Four-corner coupling diagnostic

Five separate flat-road native histories under
`runs/contact_mpc_static_coupling_a/` measured zero input and each corner's
+1 kN spring-seat pulse (0.25–0.50 s). At 0.4955 s, positive FR force moved
the instant CG about +3.42 mm left and increased its own wheel load about
+175 N, but changed FL/RL/RR loads by approximately −354/+385/−319 N.
The other corner responses are similarly coupled. A static bounded LP using
these measured gains produced a mathematical FR-unloading combination; its
native flat-road replay (`runs/contact_mpc_coupled_flat_fr_a/`) **invalidated
the linear prediction**: RL lost contact, roll reached about 11° and yaw
exceeded 11°. The gain matrix is not a safe dynamic predictor.

A smaller coupled flat-road force pattern under
`runs/contact_mpc_coupled_flat_nullspace_a/add_2.0/` gave a seemingly good
1.6–2.0 s interval: FR `Fz=0`, minimum other load ~325 N, positive triangle
margin ~0.061 m, roll ~5.8° and yaw ~1.0°. However the FR suspension jounce
was approximately −100.25 to −100.28 mm, *past the −100 mm rebound stop*.
It lifted by raising the body and hanging the wheel on a mechanical stop,
not by the required controlled ball-screw-like wheel retraction. It is
therefore **not admissible** and was not promoted to a pit candidate.

## Interpretation and stop rule

The available negative FR seat-force direction compresses the target
suspension, but the tested bounded profiles could not simultaneously reduce
target load, maintain three support loads and move the native CG into the
safe triangle. The flat-road coupling experiments show substantial
nonlinear stop/contact effects; the one-step gain matrix cannot resolve
them. This supports trying a *new, pre-registered* four-corner dynamic
trajectory search on the same frozen plant, followed by native validation.
It does **not** justify declaring the pit or the plant physically impossible.

The approved plan's Gate 2 condition was not met. Do not fit the scheduled
contact predictor for closed-loop use or implement MPC as though a safe
reference trajectory already exists. Increasing force authority, changing
actuator geometry, suspension stops, mass or pit dimensions would make a
different plant/scenario and requires an explicit sensitivity protocol and
fresh Gate-1 identity and Gate-2 evidence.
