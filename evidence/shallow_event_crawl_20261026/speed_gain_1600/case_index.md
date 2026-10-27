# 固定参数验证工况索引

所有视频为原生历史，未导出视频文件。MF52采纳身份以各运行plant元数据为准；SIMULATION_PASS仅指参数化研究筛查，能力扫描不计入基线通过。

本任务允许关键坑沿主动降速；接近速度不代表恒定坑内速度能力。

|工况名称|类别|坑深cm|接近速度km/h|实际坑区均速/峰值km/h|筛查|未通过项目|
|---|---|---:|---:|---:|---|---|
|event_gain1600_wide_d04_a30_20261027_001|wide/gain1600_approach3_edge08|4|3|0.992 / 4.276|FAIL|rear_and_stop, loaded_wheel_speed_error, no_backward_motion, stop_hold, backward_distance, event_speed_envelope, event_reference_overspeed|
|event_gain1600_right_track_d04_a30_20261027_001|right_track/gain1600_approach3_edge08|4|3|0.937 / 4.276|FAIL|rear_and_stop, loaded_wheel_speed_error, no_backward_motion, stop_hold, backward_distance, event_speed_envelope, event_reference_envelope, event_reference_overspeed|
|event_gain1600_wide_d10_a30_20261027_001|wide/gain1600_approach3_edge08|10|3|1.090 / 4.367|FAIL|rear_and_stop, loaded_wheel_speed_error, no_backward_motion, contact_retention, wheel_load_peak, stop_hold, backward_distance, event_speed_envelope, event_reference_overspeed|
|event_gain1600_right_track_d10_a30_20261027_001|right_track/gain1600_approach3_edge08|10|3|0.997 / 4.105|FAIL|rear_and_stop, loaded_wheel_speed_error, no_backward_motion, contact_retention, wheel_load_peak, stop_hold, backward_distance, event_speed_envelope, event_reference_overspeed|

播放：`python scripts/open_run_visualizer.py runs/shallow_press/<case>/<run_id>`。
详细速度、出坑停顿、姿态、轮速误差、轮载、净空假设及哈希见`verified_analysis.json`。
