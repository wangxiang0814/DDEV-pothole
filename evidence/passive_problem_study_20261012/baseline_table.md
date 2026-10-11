# Passive baseline phase diagnostics

No active suspension or wheel-specific torque allocation; speed PI and steering feedback remain.

Window: crossing through rear exit +1 s, or available prefix. Censored rows are not comparable at +1 s; use crossing_plus_05s in JSON for speed comparisons.

| Case | km/h | Depth cm | Az 10ms m/s2 | Roll deg | Pitch deg | Wheel-loss sum s | Loaded slip sum s | Compression-stop sum s | Minimum support | +1s censored |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| right_track | 3 | 8 | 9.70 | 3.23 | 1.71 | 0.345 | 2.296 | 0.000 | 3 | True |
| right_track | 3 | 12 | 16.46 | 5.21 | 2.63 | 0.544 | 3.473 | 0.007 | 3 | True |
| right_track | 3 | 20 | 45.31 | 9.77 | 4.53 | 1.544 | 6.020 | 0.007 | 2 | True |
| right_track | 10 | 4 | 2.90 | 0.96 | 0.95 | 0.167 | 0.583 | 0.000 | 3 | False |
| right_track | 10 | 8 | 10.04 | 2.21 | 1.72 | 0.400 | 0.261 | 0.000 | 3 | False |
| right_track | 10 | 10 | 15.04 | 2.92 | 2.07 | 0.493 | 0.422 | 0.007 | 3 | False |
| right_track | 10 | 12 | 28.37 | 3.62 | 2.43 | 0.590 | 0.409 | 0.007 | 3 | False |
| right_track | 10 | 16 | 46.63 | 4.38 | 3.09 | 0.831 | 0.868 | 0.007 | 2 | False |
| right_track | 10 | 20 | 59.54 | 4.57 | 3.65 | 0.913 | 0.784 | 0.006 | 2 | False |
| wide | 3 | 8 | 21.53 | 2.28 | 4.08 | 0.788 | 7.173 | 0.006 | 2 | True |
| wide | 3 | 12 | 23.15 | 2.12 | 6.02 | 1.572 | 11.273 | 0.014 | 2 | True |
| wide | 3 | 20 | 58.78 | 7.88 | 10.32 | 3.583 | 30.987 | 0.030 | 1 | True |
| wide | 10 | 4 | 7.57 | 1.14 | 2.17 | 0.313 | 0.605 | 0.000 | 2 | False |
| wide | 10 | 8 | 53.06 | 1.95 | 3.93 | 1.027 | 1.526 | 0.014 | 2 | False |
| wide | 10 | 10 | 79.29 | 2.18 | 5.26 | 1.510 | 1.857 | 0.016 | 2 | False |
| wide | 10 | 12 | 98.04 | 2.63 | 9.01 | 2.310 | 2.248 | 0.021 | 1 | False |
| wide | 10 | 16 | 115.11 | 9.29 | 25.97 | 3.968 | 2.920 | 0.172 | 0 | False |
| wide | 10 | 20 | 142.40 | 12.59 | 43.35 | 7.181 | 2.058 | 0.259 | 0 | False |