# 固定参数验证工况索引

所有视频为原生历史，未导出视频文件。MF52采纳身份以各运行plant元数据为准；SIMULATION_PASS仅指参数化研究筛查，能力扫描不计入基线通过。

|工况名称|类别|坑深cm|目标km/h|筛查|未通过项目|
|---|---|---:|---:|---|---|
|confirm_v2yaw_repeat_wide_d10_v10_20261025_001|wide/repeat|10|1|FAIL|loaded_wheel_speed_error, no_backward_motion, contact_retention, wheel_load_peak, backward_distance|
|confirm_v2yaw_step025_wide_d10_v10_20261025_001|wide/step025|10|1|FAIL|loaded_wheel_speed_error, no_backward_motion, contact_retention, wheel_load_peak, backward_distance, no_exit_pause, speed_tracking|
|confirm_v2yaw_delay80_wide_d10_v10_20261025_001|wide/delay80|10|1|FAIL|path, loaded_wheel_speed_error, no_backward_motion, contact_retention, wheel_load_peak, backward_distance, no_exit_pause, speed_tracking|
|confirm_v2yaw_traction_wide_d10_v10_20261025_001|wide/traction|10|1|FAIL|pitch, loaded_wheel_speed_error, no_backward_motion, contact_retention, wheel_load_peak, stop_hold, backward_distance, speed_tracking|
|confirm_v2yaw_repeat_wide_d04_v30_20261025_001|wide/repeat|4|3|SIMULATION_PASS|—|
|confirm_v2yaw_step025_wide_d04_v30_20261025_001|wide/step025|4|3|SIMULATION_PASS|—|
|confirm_v2yaw_delay80_wide_d04_v30_20261025_001|wide/delay80|4|3|FAIL|loaded_wheel_speed_error, wheel_load_peak|
|confirm_v2yaw_traction_wide_d04_v30_20261025_001|wide/traction|4|3|FAIL|loaded_wheel_speed_error|

播放：`python scripts/open_run_visualizer.py runs/shallow_press/<case>/<run_id>`。
详细速度、出坑停顿、姿态、轮速误差、轮载、净空假设及哈希见`verified_analysis.json`。
