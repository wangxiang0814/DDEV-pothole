# 固定参数验证工况索引

所有视频为原生历史，未导出视频文件。MF52采纳身份以各运行plant元数据为准；SIMULATION_PASS仅指参数化研究筛查，能力扫描不计入基线通过。

|工况名称|类别|坑深cm|目标km/h|筛查|未通过项目|
|---|---|---:|---:|---|---|
|intermediate_v2yaw_wide_d04_v15_20261025_001|wide/intermediate|4|1.5|SIMULATION_PASS|—|
|intermediate_v2yaw_wide_d04_v25_20261025_001|wide/intermediate|4|2.5|FAIL|loaded_wheel_speed_error|
|intermediate_v2yaw_right_track_d04_v15_20261025_001|right_track/intermediate|4|1.5|SIMULATION_PASS|—|
|intermediate_v2yaw_right_track_d04_v25_20261025_001|right_track/intermediate|4|2.5|SIMULATION_PASS|—|

播放：`python scripts/open_run_visualizer.py runs/shallow_press/<case>/<run_id>`。
详细速度、出坑停顿、姿态、轮速误差、轮载、净空假设及哈希见`verified_analysis.json`。
