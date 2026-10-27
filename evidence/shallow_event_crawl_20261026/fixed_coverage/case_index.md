# 固定参数验证工况索引

所有视频为原生历史，未导出视频文件。MF52采纳身份以各运行plant元数据为准；SIMULATION_PASS仅指参数化研究筛查，能力扫描不计入基线通过。

本任务允许关键坑沿主动降速；接近速度不代表恒定坑内速度能力。

|工况名称|类别|坑深cm|接近速度km/h|实际坑区均速/峰值km/h|筛查|未通过项目|
|---|---|---:|---:|---:|---|---|
|event_fixed_speed_right_track_d04_a1_20261027_001|right_track/speed|4|1|0.872 / 1.075|SIMULATION_PASS|—|
|event_fixed_speed_wide_d04_a1_20261027_001|wide/speed|4|1|0.828 / 1.137|SIMULATION_PASS|—|
|event_fixed_speed_right_track_d04_a2_20261027_001|right_track/speed|4|2|0.877 / 1.443|SIMULATION_PASS|—|
|event_fixed_speed_wide_d04_a2_20261027_001|wide/speed|4|2|0.829 / 1.491|SIMULATION_PASS|—|
|event_fixed_speed_right_track_d04_a4_20261027_001|right_track/speed|4|4|0.870 / 1.441|SIMULATION_PASS|—|
|event_fixed_speed_wide_d04_a4_20261027_001|wide/speed|4|4|0.822 / 1.488|SIMULATION_PASS|—|
|event_fixed_speed_right_track_d04_a5_20261027_001|right_track/speed|4|5|0.869 / 1.446|SIMULATION_PASS|—|
|event_fixed_speed_wide_d04_a5_20261027_001|wide/speed|4|5|0.818 / 1.494|SIMULATION_PASS|—|
|event_fixed_depth_right_track_d06_a3_20261027_001|right_track/depth|6|3|0.863 / 1.452|SIMULATION_PASS|—|
|event_fixed_depth_wide_d06_a3_20261027_001|wide/depth|6|3|0.762 / 2.209|FAIL|no_backward_motion, backward_distance, event_speed_envelope, event_reference_overspeed|
|event_fixed_depth_right_track_d08_a3_20261027_001|right_track/depth|8|3|0.847 / 1.471|FAIL|loaded_wheel_speed_error, event_speed_envelope, event_reference_overspeed|
|event_fixed_depth_wide_d08_a3_20261027_001|wide/depth|8|3|0.686 / 3.302|FAIL|loaded_wheel_speed_error, no_backward_motion, backward_distance, event_speed_envelope, event_reference_overspeed|
|event_fixed_delay_right_track_d04_a3_20261027_001|right_track/delay|4|3|0.876 / 1.440|SIMULATION_PASS|—|
|event_fixed_delay_wide_d04_a3_20261027_001|wide/delay|4|3|0.852 / 2.280|FAIL|no_backward_motion, backward_distance, event_speed_envelope, event_reference_overspeed|
|event_fixed_halfstep_right_track_d04_a3_20261027_001|right_track/halfstep|4|3|0.873 / 1.446|SIMULATION_PASS|—|
|event_fixed_halfstep_wide_d04_a3_20261027_001|wide/halfstep|4|3|0.825 / 1.492|SIMULATION_PASS|—|

播放：`python scripts/open_run_visualizer.py runs/shallow_press/<case>/<run_id>`。
详细速度、出坑停顿、姿态、轮速误差、轮载、净空假设及哈希见`verified_analysis.json`。
