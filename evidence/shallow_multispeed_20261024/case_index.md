# 固定参数验证工况索引

所有视频为原生历史，未导出视频文件。MF52采纳身份以各运行plant元数据为准；SIMULATION_PASS仅指参数化研究筛查，能力扫描不计入基线通过。

|工况名称|类别|坑深cm|目标km/h|筛查|未通过项目|
|---|---|---:|---:|---|---|
|preview_multispeed_validation_right_track_d04_v10_20261024_001|right_track/validation|4|1|SIMULATION_PASS|—|
|preview_multispeed_validation_right_track_d04_v20_20261024_001|right_track/validation|4|2|SIMULATION_PASS|—|
|preview_multispeed_validation_right_track_d04_v30_20261024_001|right_track/validation|4|3|SIMULATION_PASS|—|
|preview_multispeed_validation_right_track_d04_v50_20261024_001|right_track/validation|4|5|FAIL|loaded_wheel_speed_error|
|preview_multispeed_validation_right_track_d02_v30_20261024_001|right_track/validation|2|3|SIMULATION_PASS|—|
|preview_multispeed_validation_right_track_d02_v50_20261024_001|right_track/validation|2|5|SIMULATION_PASS|—|
|preview_multispeed_validation_right_track_d06_v10_20261024_001|right_track/validation|6|1|SIMULATION_PASS|—|
|preview_multispeed_validation_right_track_d06_v30_20261024_001|right_track/validation|6|3|FAIL|loaded_wheel_speed_error, wheel_load_peak|
|preview_multispeed_delay80_right_track_d04_v30_20261024_001|right_track/delay80|4|3|SIMULATION_PASS|—|
|preview_multispeed_step025_right_track_d04_v30_20261024_001|right_track/step025|4|3|SIMULATION_PASS|—|
|preview_multispeed_traction_right_track_d04_v30_20261024_001|right_track/traction|4|3|SIMULATION_PASS|—|
|preview_multispeed_passive_right_track_d04_v30_20261024_001|right_track/passive|4|3|FAIL|loaded_wheel_speed_error|
|preview_multispeed_repeat_right_track_d04_v30_20261024_001|right_track/repeat|4|3|SIMULATION_PASS|—|
|preview_multispeed_validation_wide_d04_v10_20261024_001|wide/validation|4|1|SIMULATION_PASS|—|
|preview_multispeed_validation_wide_d04_v20_20261024_001|wide/validation|4|2|FAIL|loaded_wheel_speed_error|
|preview_multispeed_validation_wide_d04_v30_20261024_001|wide/validation|4|3|FAIL|loaded_wheel_speed_error|
|preview_multispeed_validation_wide_d04_v50_20261024_001|wide/validation|4|5|FAIL|loaded_wheel_speed_error|
|preview_multispeed_validation_wide_d02_v30_20261024_001|wide/validation|2|3|SIMULATION_PASS|—|
|preview_multispeed_validation_wide_d02_v50_20261024_001|wide/validation|2|5|SIMULATION_PASS|—|
|preview_multispeed_validation_wide_d06_v10_20261024_001|wide/validation|6|1|FAIL|loaded_wheel_speed_error, wheel_load_peak|
|preview_multispeed_validation_wide_d06_v30_20261024_001|wide/validation|6|3|FAIL|loaded_wheel_speed_error, contact_retention, wheel_load_peak|
|preview_multispeed_delay80_wide_d04_v30_20261024_001|wide/delay80|4|3|FAIL|loaded_wheel_speed_error, wheel_load_peak|
|preview_multispeed_step025_wide_d04_v30_20261024_001|wide/step025|4|3|FAIL|loaded_wheel_speed_error|
|preview_multispeed_traction_wide_d04_v30_20261024_001|wide/traction|4|3|FAIL|loaded_wheel_speed_error|
|preview_multispeed_passive_wide_d04_v30_20261024_001|wide/passive|4|3|FAIL|loaded_wheel_speed_error|
|preview_multispeed_repeat_wide_d04_v30_20261024_001|wide/repeat|4|3|FAIL|loaded_wheel_speed_error|

播放：`python scripts/open_run_visualizer.py runs/shallow_press/<case>/<run_id>`。
详细速度、出坑停顿、姿态、轮速误差、轮载、净空假设及哈希见`verified_analysis.json`。
