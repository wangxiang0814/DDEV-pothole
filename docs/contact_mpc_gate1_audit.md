# Contact-Mode MPC Gate 1: Vehicle and Interface Audit

Date: 2026-09-24. This audit is limited to the current Compact Utility Truck
`I_I` corner-module model; it is not a heavy-duty-vehicle or pothole-traversal
validation.

## Frozen identities and authority

| Item | Frozen value / evidence |
|---|---|
| Primary vehicle | `models/corner_module_ddev/run_all.par`, SHA-256 `a5bf133c1b7312c897861e2da45ab9d7f887a5e765b78d4b8ff201775ecb2f73` |
| Primary pit scenario | `models/corner_module_ddev/single_wheel_deep_pothole/scenario.json`, SHA-256 `eb99ea9df7426102266e2ba32d258af27bcf72478a7911be029576a9251e2a52` |
| Stationary probe model | `runs/contact_mpc_interface_stationary_case_a/model/run_all.par`, SHA-256 `b828263623a8ecf297665aaab06e9be3c209d1041fb2f66cd14c0ab9df0f0286` |
| Eight imports | `IMP_MYUSM_L1/R1/L2/R2` (N·m), `IMP_FS_L1/R1/L2/R2` (N), ordered FL/FR/RL/RR in each group |
| Outputs checked | Requested eight `imp_*`; applied `FsExt_*` and `My_US_*`; `Fz_*`, wheel speed `AVy_*`, wheel XYZ, `Jnc_*`, `Zgnd_*i`, body pose/rates |
| Preview | TruckSim scenario ground truth; no onboard perception is implied |
| Nominal simulation force bound | ±17,626.744 N per corner (5× the measured 3,525.349 N maximum static corner load) |
| Nominal simulation slew bound | 44,066.861 N/s (nominal limit / 0.4 s force-build time) |

The force and slew numbers are **controller simulation authority**, not
measured actuator or ball-screw ratings. The same bounds must be used in
reachability, identification and A/B control trials; any change requires a
newly named experiment and contract. Machine-readable primary and stationary
probe contracts are stored with the run under
`runs/contact_mpc_interface_stationary_gate1_a/`.

The 0.8–1.0 s stationary zero-input window had average FL/FR/RL/RR loads
approximately 3474/3525/3100/3229 N; total-load range was 1.42% of the
mean. TruckSim output units verified through `ddevsim.units` are: `AVy_*`
rpm, `Fz_*` and `FsExt_*` N, `Jnc_*` mm, wheel XYZ and `Zgnd_*i` m,
`Roll_E`/`Yaw` degrees and body rates degrees/s. The coordinate-frame
declaration remains `+X` forward, `+Y` left, `+Z` up; future contact-point
geometry must verify its origin/attitude transformation independently.

## Native pulse result and validator correction

The first 17-case run used the moving 11 s pit simfile. All solver cases
completed and all eight command-integrity checks were true, but polarity
analysis was invalid because the baseline front-right load changed from
~3466 N at 0.30 s to zero at 2.80 s in the pit. Its `FAIL` report is
preserved in `runs/contact_mpc_interface_gate1_a/`.

A fresh stationary 1.0 s case then ran the same 17 cases. The old analyzer
still returned `FAIL`: it selected the largest absolute wheel-speed deviation
over the *entire* run, including post-pulse rebound. For example, during the
0.25–0.35 s command window the rear-left wheel at 0.2995 s was +21.2 rpm
for +500 N·m and −30.2 rpm for −500 N·m, versus +2.0 rpm baseline. A
regression test now restricts polarity evaluation to the manifest's actual
command window; the initial stationary `FAIL` is preserved as
`verification_summary.pre_window_fix.json` and
`verification_report.pre_window_fix.md`.

Reanalysis of the **same** stationary CSVs after that correction gives
`PASS`: 17/17 solver cases completed, 17/17 input-integrity checks true,
4/4 wheel-torque polarity pairs and 4/4 spring-seat-force response pairs
passed. The positive/negative torque-window wheel-speed deltas are roughly
FL +45/−44, FR +46/−44, RL +45/−45 and RR +44/−44 rpm. The positive/negative
force-window own-wheel-load deltas are roughly FL +704/−571, FR +693/−573,
RL +263/−175 and RR +248/−182 N. Cross-corner responses remain coupled
through body and tyres; independent **command authority** does not mean
independent vehicle dynamics.

Gate 1 therefore verifies the available eight input paths and their
observable responses on the frozen vehicle. It does **not** establish
three-wheel-support reachability, 0.20 m pit clearance, safe FR→RR sequencing
or a valid contact-mode prediction model; those remain Gates 2–5.
