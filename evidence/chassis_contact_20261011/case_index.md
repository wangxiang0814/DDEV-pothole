# Contact diagnostic run index

Assumed underside geometry and uncalibrated contact parameters. No physical safety certification.

| Case | Run name | Depth cm | Clearance cm | Contact | Duration s | Rear cleared |
|---|---|---:|---:|---|---:|---|
| right_track | `chassis_contact_20261011_right_d200_h100` | 20 | 10 | True | 30 | True |
| right_track | `chassis_forcecheck_20261011_up1000` | 4 | 15 | False | 6 | False |
| right_track | `chassis_forcecheck_20261011_zero` | 4 | 15 | False | 6 | False |
| wide | `chassis_check_20261011_wide_d200_off` | 20 | 15 | False | 30 | True |
| wide | `chassis_contact_20261011_wide_d080_h100` | 8 | 10 | True | 30 | True |
| wide | `chassis_contact_20261011_wide_d160_h150` | 16 | 15 | True | 30 | True |
| wide | `chassis_contact_20261011_wide_d200_h150` | 20 | 15 | True | 30 | True |
| wide | `chassis_contact_20261011_wide_d200_h150_grid45` | 20 | 15 | True | 30 | True |
| wide | `chassis_contact_20261011_wide_d200_h150_stiff10` | 20 | 15 | True | 30 | True |

The two 6-second runs only validate the force interface. Full parameters and immutable CSV hashes: contact_runs.json.
Legacy applied-force capability flags in disabled runs are corrected in this index; original records are retained.