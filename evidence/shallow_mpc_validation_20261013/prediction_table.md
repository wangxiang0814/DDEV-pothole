# 200 ms prediction gate (held-out native runs)

Jounce/load/attitude errors are the worst corner/component RMSE and maximum absolute error; all are prediction diagnostics, not actual vehicle motion peaks.

| Case | cm | km/h | Jnc RMSE mm | Jnc max mm | Load RMSE N | Load max N | Attitude RMSE deg | False-support samples | Rear exited | Gate |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| right_track | 2 | 1 | 1.66 | 7.70 | 323.1 | 3104.1 | 0.122 | 4 | True | False |
| right_track | 2 | 2 | 2.12 | 9.61 | 382.0 | 3088.6 | 0.162 | 3 | True | False |
| right_track | 2 | 3 | 2.61 | 11.15 | 440.4 | 2551.5 | 0.202 | 1 | True | False |
| right_track | 4 | 1 | 6.87 | 38.91 | 1573.0 | 8055.8 | 0.277 | 311 | True | False |
| right_track | 4 | 2 | 5.11 | 31.65 | 717.0 | 4674.8 | 0.357 | 6 | True | False |
| right_track | 4 | 3 | 5.60 | 30.14 | 701.6 | 4072.1 | 0.358 | 4 | True | False |
| right_track | 6 | 1 | 10.54 | 52.17 | 2128.3 | 5720.8 | 0.466 | 397 | True | False |
| right_track | 6 | 2 | 7.57 | 59.07 | 1102.3 | 11818.7 | 0.336 | 13 | True | False |
| right_track | 6 | 3 | 8.27 | 60.92 | 1082.6 | 9014.4 | 0.398 | 9 | True | False |
| wide | 2 | 1 | 2.69 | 11.75 | 604.5 | 3104.4 | 0.140 | 3 | True | False |
| wide | 2 | 2 | 2.68 | 10.07 | 459.6 | 3402.1 | 0.136 | 2 | True | False |
| wide | 2 | 3 | 4.08 | 15.80 | 587.2 | 2709.3 | 0.182 | 2 | True | False |
| wide | 4 | 1 | 11.53 | 64.14 | 1402.5 | 8348.9 | 0.815 | 85 | True | False |
| wide | 4 | 2 | 7.02 | 49.66 | 919.5 | 7059.5 | 0.267 | 17 | True | False |
| wide | 4 | 3 | 7.89 | 37.74 | 856.8 | 8026.4 | 0.323 | 8 | True | False |
| wide | 6 | 1 | 12.43 | 79.35 | 1580.4 | 10977.9 | 0.425 | 120 | True | False |
| wide | 6 | 2 | 12.30 | 68.55 | 2361.2 | 12822.9 | 0.464 | 56 | True | False |
| wide | 6 | 3 | 13.82 | 75.31 | 1438.9 | 17428.4 | 0.617 | 29 | True | False |
