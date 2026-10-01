# FR→RR 测量噪声验证

## 实现与范围

沿用已批准方案，在本地独立 I_I 运行副本验证，固定质量与载荷位置，保留经典算法、0.5 ms 求解步长和 20 ms 控制更新。未提高保护阈值。

新增 `observation_noise.py` 和集中配置 `ObservationNoiseConfig`。`--measurement-noise` 默认关闭，`--noise-seed` 固定随机序列。每 20 ms 更新噪声偏置，求解器其余调用保持偏置；只改反馈观察，不改 TruckSim 原生输出。

| 通道 | 高斯标准差 |
|---|---:|
| 四轮 Fz | 20 N |
| Yo | 1 mm |
| Yaw、Roll_E、Pitch | 0.02° |
| Vx | 0.01 km/h |

偏置截断为 ±3σ。这是仿真测试设置，不是实测传感器规格。负轮载观察值投影至 0，满足现有估计器合同，但会在卸载轮附近产生正偏差。CoM、接触几何、净空、角速度、行程、轮速仍理想化；未验证这些通道的误差、延迟和丢帧。

## 独立验收

- `front_observed_20ms.csv`、`rear_observed_20ms.csv` 保存控制器观察。
- `front_control_20ms.csv`、`rear_control_20ms.csv` 保存同 tick 的真实状态；轮载原始值与独立 5 Hz 滤波值同时记录，重新计算 ZMP／CoM 裕度，保留实际 FSM 和下发命令。
- 初始路径在控制器首次捕获参考的同 tick 独立记录真实 Yo/Yaw，两级验收共用；控制器仍使用带噪声参考，结果同时记录两者。
- `observed_rr_ready`、`observed_rr_safe` 表示控制判定，不作为真实验收判定。
- 原生 5 ms CSV 始终记录 TruckSim 真值。

前 3 次运行完成后修复了路径基准隔离；基于未改变的真值控制 CSV 在初始参考捕获 tick 重算验收，JSON 中记录 `evaluation_note`。最终标称重复复核直接通过最终代码生成真值基准和评估。

## 原生结果

均用 balanced、RR robust、FR 转向反馈。FR 目标 3.4 km/h、远缘前 0.5 m 制动；RR 目标 3.35 km/h，两轮坑内最低要求 3 km/h。

| 工况 | 结果 | RR 最低净空 | RR 最弱支撑 | 最小 ZMP λ | 最大横移 |
|---|---|---:|---:|---:|---:|
| μ=0.7，标称起点，seed=20261001 | PASS | 30.76 mm | 822 N | 0.06166 | 2.75 cm |
| 同工况最终代码重复复核 | PASS | 30.76 mm | 822 N | 0.06166 | 2.75 cm |
| μ=0.6，起点提前 0.3 m，seed=20261002 | FAIL | 41.42 mm | 959 N | 0.07200 | 8.24 cm |
| 上述组合加静止转向反馈 | FAIL | 29.89 mm | 894 N | 0.06712 | 8.13 cm |

标称完整动作约 101.86 s，包含 FR/RR 各 5 s 保持、停车、落轮与四轮恢复；RR 坑内速度 3.045–3.203 km/h。两次标称 `native_5ms.csv` 字节相同，SHA-256：

`e69e5549b9e57acf6d85619e71dad575bfac56a022215449ee751b12e6dea4eb`

组合工况 FR 通过，RR 支撑、净空、速度、停车及四轮恢复通过，唯横移超过 5 cm。动作完成不等于验收通过。

## 定位及下一步

失败组合进入 RR_PRELOAD 时 Yo≈−0.07 cm、Yaw≈−0.059°；进入 RR_LIFTING 时已达 7.85 cm／2.26°，RR_CRAWL 开始约 8.08 cm／2.30°。偏移主要积累在静止预载阶段，移动后方向反馈逐渐收回。无噪声同一低附着／近起点工况进入 RR_LIFTING 为 3.77 cm／1.05°。成功预载轨迹对反馈及前阶段终态敏感，具体传递链仍需确认，不能归因于单一噪声通道。

静止转向反馈仅改善约 1 mm，默认关闭。暂缓 3.5–7 km/h：先对照各轮滚动角／速度与偏航，确定悬架调节时自由滚动的贡献；若证据支持，加入经典轮角保持／阻尼反馈抵消累积滚动，抬起轮保持零驱动转矩，蠕行入口平滑交接。仅复核失败组合与标称基线，通过后再提速和压缩周期。在线约束 QP 尚未完整接入。

测试：277 项通过。只读审查发现路径基准隔离问题，已补充先失败后通过的回归测试并修复；复核未发现剩余重要问题。测试工具通过不代表组合控制工况通过。

## 复核

在仓库根目录执行：

```powershell
$env:PYTHONPATH = 'src'
python scripts/run_right_side_full_cycle.py --output runs/noise_recheck --efficiency-profile balanced --rear-control-profile robust --front-steering-feedback --front-target-speed-kph 3.4 --front-min-pit-speed-kph 3 --front-brake-lead-m 0.5 --rear-target-speed-kph 3.35 --rear-min-pit-speed-kph 3 --measurement-noise --noise-seed 20261001
```

复现失败组合时改 seed 为 `20261002`，添加 `--road-friction 0.6 --vehicle-start-offset-m 0.3`。完整数据只留本机 runs，GitHub 保留四次精简证据。
