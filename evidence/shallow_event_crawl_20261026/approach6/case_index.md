# 固定参数验证工况索引

所有视频为原生历史，未导出视频文件。MF52采纳身份以各运行plant元数据为准；SIMULATION_PASS仅指参数化研究筛查，能力扫描不计入基线通过。

本任务允许关键坑沿主动降速；接近速度不代表恒定坑内速度能力。

|工况名称|类别|坑深cm|接近速度km/h|实际坑区均速/峰值km/h|筛查|未通过项目|
|---|---|---:|---:|---:|---|---|
|event_approach6_wide_d04_a60_20261027_001|wide/high_approach_staged_slowdown|4|6|0.818 / 1.490|SIMULATION_PASS|—|
|event_approach6_right_track_d04_a60_20261027_001|right_track/high_approach_staged_slowdown|4|6|0.867 / 1.443|SIMULATION_PASS|—|

播放：`python scripts/open_run_visualizer.py runs/shallow_press/<case>/<run_id>`。
详细速度、出坑停顿、姿态、轮速误差、轮载、净空假设及哈希见`verified_analysis.json`。
