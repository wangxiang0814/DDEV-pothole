# 专家二配置

|文件|用途|
|---|---|
|development.json|策略、场景及研究基线元数据；不是worker控制参数|
|preview_feedback_20261024.json|首批22项低速冻结配置，对应Git版本0ab1eb2|
|preview_multispeed_20261024.json|初次多速度26项研究配置，保留失败边界，不作全面推荐|
|preview_multispeed_v2_20261024.json|最新较温和姿态反馈候选，对应独立30项矩阵；仅采用索引中实际通过的离散工况|
|five_camera.json|浅坑五相机采集选项，与控制参数独立|

运行时明确传入`--config`，不传时沿用控制器构造默认值，不能当作最新候选配置。单位及真实速度见[多速度报告](../../docs/SHALLOW_MULTISPEED_TUNING_20261024.md)。相同配置不保证所有坑形或速度通过；未通过结果不能用于安全放行。

更快转矩仅通过显式`--diagnostic-torque-rate-nm-s`启动独立诊断。该选项未写进任何常规配置；±500 Nm／1000 Nm/s原能力基线保留。被淘汰的调参文件在`archive/preview_development_20261024/`，各次运行还保留自己的control_config和source快照。
