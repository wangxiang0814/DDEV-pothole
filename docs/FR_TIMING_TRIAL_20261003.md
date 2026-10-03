# FR 卸载时间优化试验（2026-10-03）

**后续进展：**近起点/μ=0.6/seed=20261002 与远起点/μ=0.7/seed=20261003 均复核 PASS；集中配置 `COMPACT_CYCLE_FRONT.preload_max_reference_rate` 已更新为 3.0。三工况完整周期 87.68–88.48 s、FR 卸载 13.88–14.30 s、最大横移 2.17–2.23 cm。以下保留最初单工况诊断过程；“推荐不变”只描述当时状态。新证据见 `evidence/static_fr_ii/compact3_validation_20261003.json`。

## 结果

保持当前车型副本、compact / 横向表 scale=0、RR robust、FR 转向反馈、seed=20261001 和标称场景，仅覆盖 FR `preload_max_reference_rate=3.0`。原生结果 PASS。

| 指标 | 推荐基线（2.5） | 试验（3.0） |
|---|---:|---:|
| FR 卸载 | 17.02 s | 14.30 s |
| 双轮完整动作 | 90.80 s | 88.48 s |
| FR/RR 运动与停车共用路径最大横移 | 2.17 cm | 2.18 cm |
| RR 最低坑上净空 | 32.18 mm | 30.45 mm |
| RR 最小支撑轮载 | 766 N | 762 N |
| RR 最小 ZMP 三角形坐标 | 0.05748 | 0.05712 |

最大推进速度只在跟踪误差较小时生效，原有误差大时减速/暂停逻辑保留。两次三轮保持均满足 ≥5 s，停车后落轮并恢复四轮。FR 支撑反馈达到限幅最长约 0.10 s，未达到持续饱和门槛。

周期缩短 2.32 s，尚不能说明扰动鲁棒性改善。RR 净空/支撑裕度略降；推荐配置不变，下一步复核薄弱组合工况后再决定是否采用。FR 恢复时间未缩短。

## 复现

在仓库根目录 PowerShell 执行以下命令。它只在当前 Python 进程覆盖集中配置，不修改源文件或原始车型；输出配置和模型哈希保存在运行结果及精简证据中。

```powershell
$env:PYTHONPATH='src'
python -c "import sys; sys.path.insert(0,'scripts'); import run_right_side_full_cycle as m; from dataclasses import replace; m.COMPACT_CYCLE_FRONT=replace(m.COMPACT_CYCLE_FRONT,preload_max_reference_rate=3.0); sys.argv=['run_right_side_full_cycle.py','--output','runs/front_rate3_lateral0_noise_nominal','--efficiency-profile','compact','--lateral-ride-scale','0','--rear-control-profile','robust','--front-steering-feedback','--front-target-speed-kph','3.4','--front-min-pit-speed-kph','3','--front-brake-lead-m','0.5','--rear-target-speed-kph','3.35','--rear-min-pit-speed-kph','3','--measurement-noise','--noise-seed','20261001']; m.main()"
```

证据：[基线与试验完整配置、真值验收及阶段时间](../evidence/static_fr_ii/front_timing_trial_20261003.json)。推荐基线的四工况结果另见 `robustness_matrix_20261003.json`，不能与单工况试验混算。
