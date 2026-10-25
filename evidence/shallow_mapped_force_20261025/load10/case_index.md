# 固定参数验证工况索引

所有视频为原生历史，未导出视频文件。MF52采纳身份以各运行plant元数据为准；SIMULATION_PASS仅指参数化研究筛查，能力扫描不计入基线通过。

|工况名称|类别|坑深cm|目标km/h|筛查|未通过项目|
|---|---|---:|---:|---|---|
|fast_load10_wide_d10_v10_20261025b_001|wide/legacy_fast_load10|10|1|FAIL|rear_and_stop, loaded_wheel_speed_error, no_backward_motion, wheel_load_peak, stop_hold, backward_distance, speed_tracking|

播放：`python scripts/open_run_visualizer.py runs/shallow_press/<case>/<run_id>`。
详细速度、出坑停顿、姿态、轮速误差、轮载、净空假设及哈希见`verified_analysis.json`。
