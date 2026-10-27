# 固定参数验证工况索引

所有视频为原生历史，未导出视频文件。MF52采纳身份以各运行plant元数据为准；SIMULATION_PASS仅指参数化研究筛查，能力扫描不计入基线通过。

本任务允许关键坑沿主动降速；接近速度不代表恒定坑内速度能力。

|工况名称|类别|坑深cm|接近速度km/h|实际坑区均速/峰值km/h|筛查|未通过项目|
|---|---|---:|---:|---:|---|---|
|event_callback_passthrough_flat_d04_a30_20261027_001|wide/callback_passthrough_flat_prefix_only|4|3|2.105 / 2.999|MODEL_DIAGNOSTIC_ONLY|crossing_observed, rear_and_stop, stop_hold, phase_completion, event_speed_envelope, event_motion_reference, event_reference_envelope, event_reference_overspeed, finite_progress|
|event_callback_circle_flat_d04_a30_20261027_001|wide/callback_circle_flat_prefix_only|4|3|2.105 / 2.999|MODEL_DIAGNOSTIC_ONLY|crossing_observed, rear_and_stop, stop_hold, phase_completion, event_speed_envelope, event_motion_reference, event_reference_envelope, event_reference_overspeed, finite_progress|

播放：`python scripts/open_run_visualizer.py runs/shallow_press/<case>/<run_id>`。
详细速度、出坑停顿、姿态、轮速误差、轮载、净空假设及哈希见`verified_analysis.json`。
