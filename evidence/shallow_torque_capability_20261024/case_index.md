# 独立转矩能力扫描

原能力基线保持1000 Nm/s。以下只放宽该次诊断的转矩变化率，转矩幅值仍为±500 Nm；全部simulation_pass=false。

|工况名称|变化率Nm/s|实际均速km/h|轮速误差m/s|最大轮载/静载|诊断联合筛查|诊断未通过项目|
|---|---:|---:|---:|---:|---|---|
|preview_multispeed_rate5000_wide_d04_v50_20261024_001|5000|4.987|0.468|2.556|False|loaded_wheel_speed_error, wheel_load_peak|
|preview_multispeed_rate10000_wide_d04_v50_20261024_001|10000|4.997|0.253|2.584|False|wheel_load_peak|
|preview_multispeed_rate20000_wide_d04_v50_20261024_001|20000|4.999|0.254|2.581|False|wheel_load_peak|
