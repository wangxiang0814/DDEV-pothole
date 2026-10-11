# 工况名称与后续重放索引

本轮不播放或导出视频。以下是18个工况的最终分析来源；原截断及15秒重跑保留供溯源。

所有工况：纵长0.8 m、坑前12 m、μ=0.7、被动悬架、直行转向反馈。单侧宽0.9 m，宽坑宽1.8 m。

| 工况名称 | 运行名称 | 前轮入坑/后轮出坑 s | 冲击/俯仰峰值时刻 s |
|---|---|---|---|
| RIGHT_v03kph_d08cm_W900mm_L800mm | `speed_study_20261011_v03_d080` | 21.020 / 24.936 | 24.622 / 23.899 |
| RIGHT_v03kph_d12cm_W900mm_L800mm | `speed_study_20261011_v03_d120` | 21.020 / 24.938 | 22.361 / 21.753 |
| RIGHT_v03kph_d20cm_W900mm_L800mm | `speed_study_20261011_v03_d200` | 21.020 / 24.997 | 22.370 / 21.794 |
| RIGHT_v10kph_d04cm_W900mm_L800mm | `speed_study_20261011_v10_d040` | 10.912 / 12.055 | 11.851 / 13.178 |
| RIGHT_v10kph_d08cm_W900mm_L800mm | `speed_study_20261011_v10_d080` | 10.912 / 12.053 | 11.976 / 11.313 |
| RIGHT_v10kph_d10cm_W900mm_L800mm | `speed_study_20261011_v10_d100` | 10.912 / 12.046 | 11.332 / 11.307 |
| RIGHT_v10kph_d12cm_W900mm_L800mm | `speed_study_20261011_v10_d120` | 10.912 / 12.046 | 11.328 / 11.309 |
| RIGHT_v10kph_d16cm_W900mm_L800mm | `speed_study_20261011_v10_d160` | 10.912 / 12.050 | 11.322 / 11.315 |
| RIGHT_v10kph_d20cm_W900mm_L800mm | `speed_study_20261011_v10_d200` | 10.912 / 12.059 | 11.319 / 11.316 |
| WIDE_v03kph_d08cm_W1800mm_L800mm | `speed_study_20261011_v03_d080` | 21.018 / 25.058 | 24.642 / 23.945 |
| WIDE_v03kph_d12cm_W1800mm_L800mm | `speed_study_20261011_v03_d120` | 21.018 / 26.062 | 23.634 / 22.537 |
| WIDE_v03kph_d20cm_W1800mm_L800mm | `speed_study_20261011_v03_d200` | 21.018 / 38.312 | 24.933 / 25.331 |
| WIDE_v10kph_d04cm_W1800mm_L800mm | `speed_study_20261011_v10_d040` | 10.912 / 12.060 | 11.977 / 11.864 |
| WIDE_v10kph_d08cm_W1800mm_L800mm | `speed_study_20261011_v10_d080` | 10.912 / 12.069 | 11.322 / 11.309 |
| WIDE_v10kph_d10cm_W1800mm_L800mm | `speed_study_20261011_v10_d100` | 10.912 / 12.053 | 11.319 / 12.243 |
| WIDE_v10kph_d12cm_W1800mm_L800mm | `speed_study_20261011_v10_d120` | 10.912 / 12.063 | 11.319 / 12.296 |
| WIDE_v10kph_d16cm_W1800mm_L800mm_FULL30s | `speed_study_full30_20261011_v10_d160` | 10.912 / 12.065 | 11.318 / 12.322 |
| WIDE_v10kph_d20cm_W1800mm_L800mm_FULL30s | `speed_study_full30_20261011_v10_d200` | 10.912 / 12.521 | 11.322 / 12.014 |

运行目录为 `runs/shallow_press/<right_track或wide>/<运行名称>/`。
后续打开方式：`F:/anaconda3/python.exe scripts/open_run_visualizer.py runs/shallow_press/<坑型>/<运行名称>`，一次打开一个工况。
JSON保存完整路径、原生历史哈希和关键时刻。动画采样25 ms，峰值判断以1 ms CSV为准。
