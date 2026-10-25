# 本轮工况索引

均为MF52研究模型、原软件能力下的原生运行。只统计本轮新运行；retained_v2为历史对照。实际均速是闭环结果，不是严格同轨迹的单独悬架辨识。

|组别|工况名|目标km/h|实际均速km/h|RMS m/s²|冲击10ms m/s²|轮载/静载|轮速差m/s|最长失载ms|未通过|
|---|---|---:|---:|---:|---:|---:|---:|---:|---|
|retained_v2|coverage_v2yaw_wide_d10_v10_20261025_001|1|1.121|1.550|9.677|3.306|0.744|166|loaded_wheel_speed_error, no_backward_motion, contact_retention, wheel_load_peak, backward_distance|
|balance|exit_balance_wide_d10_v10_20261025b_001|1|1.841|3.024|16.511|5.396|12.707|332|path, loaded_wheel_speed_error, impact_10ms, no_backward_motion, contact_retention, wheel_load_peak, backward_distance, speed_tracking|
|combined|exit_combined_wide_d10_v10_20261025b_001|1|1.503|2.003|13.503|6.214|9.387|277|loaded_wheel_speed_error, no_backward_motion, contact_retention, wheel_load_peak, backward_distance, speed_tracking|
|fastref|mapped_fastref_wide_d10_v10_20261025b_001|1|1.288|1.610|6.692|2.937|7.003|128|loaded_wheel_speed_error, no_backward_motion, wheel_load_peak, backward_distance, speed_tracking|
|load05|fast_load05_wide_d10_v10_20261025b_001|1|1.138|1.329|6.464|2.807|4.065|123|loaded_wheel_speed_error, no_backward_motion, wheel_load_peak, backward_distance|
|load10|fast_load10_wide_d10_v10_20261025b_001|1|1.386|1.231|6.421|3.019|3.769|112|rear_and_stop, loaded_wheel_speed_error, no_backward_motion, wheel_load_peak, stop_hold, backward_distance, speed_tracking|
|load_confirmation|load05_halfstep_wide_d10_v10_20261025b_001|1|1.335|1.423|6.491|2.807|5.558|123|loaded_wheel_speed_error, no_backward_motion, wheel_load_peak, backward_distance, speed_tracking|
|load_confirmation|load05_right_right_track_d10_v10_20261025b_001|1|1.025|0.496|3.558|2.340|3.417|143|loaded_wheel_speed_error|
|load_noground|force_load_noground_wide_d10_v10_20261025b_001|1|1.389|1.287|6.267|2.960|7.161|124|loaded_wheel_speed_error, no_backward_motion, wheel_load_peak, backward_distance, speed_tracking|
|motorp|mapped_motorp_wide_d10_v10_20261025b_001|1|1.291|1.549|6.973|2.957|11.992|135|loaded_wheel_speed_error, no_backward_motion, wheel_load_peak, backward_distance, speed_tracking|
|noground|force_noground_wide_d10_v10_20261025b_001|1|2.303|2.947|16.186|7.684|11.040|395|pitch, loaded_wheel_speed_error, impact_10ms, no_backward_motion, contact_retention, wheel_load_peak, backward_distance, speed_tracking|
|passive|mapped_passive_wide_d10_v10_20261025b_001|1|1.274|1.541|6.737|2.963|3.838|129|loaded_wheel_speed_error, no_backward_motion, wheel_load_peak, backward_distance, speed_tracking|
|spring|mapped_spring_wide_d10_v10_20261025b_001|1|1.181|1.668|7.177|3.114|6.682|136|loaded_wheel_speed_error, no_backward_motion, wheel_load_peak, stop_hold, backward_distance, speed_tracking|
|timing|mapped_timing065_wide_d10_v10_20261025b_001|1|1.301|1.703|6.629|2.909|7.518|151|loaded_wheel_speed_error, no_backward_motion, contact_retention, wheel_load_peak, backward_distance, speed_tracking|

播放对应run目录：`python scripts/open_run_visualizer.py <run目录>`。原生视频历史保留，没有导出视频。
精确配置、失败项目及来源/原生历史哈希见 iteration_summary.json；源码ZIP不包括TruckSim安装、授权或完整仿真原始数据。
