# 四项优化结果与当前推荐版本（2026-10-05）

用户批准的四项工作已按顺序完成：RR余量、动作效率、异常边界、代表工况与可复现入口。仍使用本地TruckSim、固定调参I_I、固定质量与载荷位置、经典专家控制；模型与联合仿真接口不重建，原生0.5 ms/控制20 ms不变。没有导出视频。

## 1. RR稳定余量

原来有反馈的平滑中止制动绑定了自动驶出开关。不允许自动驶出也应能在支撑安全、离地且有可靠反馈时使用平滑制动，因此将两者解耦。支撑/反馈条件不满足仍不使用该参考；自动驶出仍显式可选，没有因优化自动开启。

低附着/近起点/噪声/40 ms延迟，两种RR坑上中止均通过响应验收：

| 中止位置 | 响应 | 最大姿态 | 最低净空 | 备注 |
|---|---|---:|---:|---|
| 坑长50% | RECOVERED | 9.343° | 45.2 mm | 制动后到达固地，落轮恢复，无自动驶出 |
| 坑长20% | SAFE_HOLD | 9.679° | 44.8 mm | 停车悬空约23.8 s，未在坑上落轮 |

任务均为FAIL，恢复另验收。余量仍薄，没有达到任意故障鲁棒性的结论。另一条降低正常姿态目标的试验虽然完整PASS、roll8.48°，但最弱支撑820→757 N、ZMP λ0.06151→0.05675，净空43→32.5 mm，故没有保留。该实验配置完整保存在证据的controller_config中；它不在推荐入口中。

## 2. 卸载与恢复效率

新增显式 `efficient` 配置，保留原 `compact` 基线：

- FR卸载最大参考推进率4→5；仍由实际轮载跟踪误差减速/暂停，不是定时抬轮。
- FR四轮恢复参考12→9 s，仍为quintic；抬升、落轮和每轮至少5 s三轮保持不缩短。
- RR恢复保持既有6 s，正常速度与制动参数保持当前推荐组合。
- FR恢复验收新增全过程姿态、三支撑原始轮载、行程、横移、停稳和固地检查，缺数据或NaN不能PASS。

同一低附着工况完整周期81.201→77.141 s，减少4.060 s（约5.0%）。卸载10.28→9.18 s，FR落轮与四轮恢复总时长16→13 s；蠕行段基本不变。不能把它解释为车辆速度提高或QP计算提速带来的收益。

先尝试8 s恢复：虽最终完成四轮接地，但恢复过程中速度0.1021 km/h超过原0.1线，报告FAIL；该失败保留，没有放宽停稳线。9 s推荐组合四工况最大FR恢复速度约0.090–0.092 km/h。数据只证明有限组合可行，不代表更快恢复必然可用。

## 3. 运动中反馈缺口与有反馈中止

新增几何触发的RR运动包中断试验：坑长20%处，全部输出观测通道冻结，使用原始采样时间；控制器继续当前时间，独立plant truth验收。

| 缺包时长（叠加40 ms正常延迟） | 结果 | 关键事实 |
|---|---|---|
| 60 ms，包龄最高100.5 ms | 完整任务PASS | 未超过120 ms超时线，无误中止，最小摆动净空25.7 mm |
| 200 ms，包龄最高240 ms，初版 | 响应FAIL、任务FAIL | 停车但姿态10.135°，失败保留 |
| 同一200 ms，修复制动恢复交接后 | SAFE_HOLD、任务FAIL | 姿态9.730°，净空44.3 mm，横移2.269 cm，停车悬空23.54 s |

初版制动quintic时间在失联期间被消耗，恢复时目标速度已明显低于当前速度，支撑修正也达到限幅。修复为第一次恢复可信包时，以当前测量速度重新开始平滑制动；实际扭矩继续限速交接，积分不在失联期间增长，异常原因保持锁存。当前姿态余量仍约0.27°，不能称宽裕。

超时期间冻结实际悬架/转向、撤驱动，暂停QP/FSM；数据恢复后才继续真实反馈。没有使用旧数据判断落轮，没有重新进入正常蠕行。审查进一步要求：SAFE_HOLD必须确认缺包结束、最后数据新鲜，并在健康反馈下累计停车保持；30 s缺口被25 s观察窗截断的反例不能PASS。

这些是有限时长、随后恢复健康观测的试验，不是无限期失联、任意中断时刻、执行器故障或运动中盲停认证。

## 4. 固化正常代表工况

四工况均含观测噪声与40 ms延迟，所有正常及FR/RR恢复验收通过。坑槽只做一组宽深边界，没有遍历网格。

| 命名工况 | 内容 | 完整周期 | 共用路径最大横移 | RR最大roll |
|---|---|---:|---:|---:|
| nominal_bias | 标称，RR支撑FL −200 N偏差 | 76.521 s | 2.110 cm | 9.114° |
| low_mu | μ0.6，起点前移0.3 m | 77.141 s | 2.112 cm | 8.966° |
| boundary_bias | 低附着近起点，坑宽1.1 m/深0.25 m，RR支撑FL +200 N | 77.101 s | 2.150 cm | 8.973° |
| slow_rear | 同低附着，RR目标2.6 km/h | 77.601 s | 2.101 cm | 8.699° |

通常组合FR/RR坑内约3.06–3.45 km/h；slow_rear的RR约2.39–2.46 km/h。没有把这些数据写成3–7 km/h全范围通过。对应场景使用对应的辨识增益，宽深边界采用严格核验的源模型复用，不是重新辨识或车型泛化。

### 简洁复现入口

参数清单：[verified_right_side_20261005.json](../configs/verified_right_side_20261005.json)。源码入口：[run_verified_right_side.py](../scripts/run_verified_right_side.py)。命名入口合并参数而不重复速度选项，拒绝覆盖旧运行目录，记录实际argv与清单SHA；仿真过程报FAIL时退出失败码，不能仅凭进程成功认为控制PASS。

```powershell
$env:PYTHONPATH='src'
python scripts/run_verified_right_side.py --case low_mu --output runs/new_low_mu
python scripts/run_verified_right_side.py --case nominal_bias --output runs/new_nominal
python scripts/run_verified_right_side.py --case slow_rear --output runs/new_slow
python scripts/run_verified_right_side.py --case boundary_bias --output runs/new_boundary --reference-model runs/new_low_mu/model/run_all.par
```

边界必须显式提供原low_mu源模型。首次复现先运行low_mu；参考文件与目标文件差异仍由现有严格哈希/参数核验决定，不能绕过检查。加 `--dry-run` 可只查看参数。

运动包试验使用 `run_feedback_fault_trial.py --fault packet --fault-phase RR_CRAWL --packet-outage-s 0.06` 或 `0.2`，加清单low_mu的共同及工况选项、新输出目录；有反馈坑上中止使用 `run_pit_abort_trial.py --abort-trigger-fraction 0.2` 加同组选项。它们是独立异常试验，不纳入四正常PASS。

### 证据和测试

- [四正常工况](../evidence/static_fr_ii/four_step_validation_20261005.json)
- [RR余量与未保留方案](../evidence/static_fr_ii/rr_margin_refinement_20261005.json)
- [效率对照与8 s失败](../evidence/static_fr_ii/efficient_cycle_refinement_20261005.json)
- [运动丢包与初次失败](../evidence/static_fr_ii/moving_packet_validation_20261005.json)

401项Python测试通过。最终独立审查发现的“恢复数据尚未返回也可能SAFE_HOLD”验收漏洞，已通过失败反例→修复→完整测试验证。仅审计变化的原生记录重新验收，结果PASS/FAIL/SAFE_HOLD不变。

## 当前边界与后续取舍

四项限定工作已完成，不意味所有车辆/工况问题消失。正常RR姿态距离10°保护约0.89°（四工况最差），200 ms中断约0.27°；低附着FR恢复速度也接近0.1线。建议先使用已固化版本采集有限工况专家数据，并保留失败与异常标签。若继续扩大速度、模型或扰动范围，再针对RR姿态/行程、增益有效域、制动预见与滑移开展独立任务；不通过进一步降低门槛制造PASS。暂没有必须改成完整MPC的证据。
