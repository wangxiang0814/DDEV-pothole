# 观测超时与持续分配失败的验证

## 实际覆盖范围

固定现有调参I_I、质量与载荷、经典专家控制。TruckSim原生0.5 ms、控制20 ms、低附着μ0.6/起点前移0.3 m、观测噪声和40 ms延迟。故障发生在RR坑前静止HOLD的0.5 s时刻；FR此前正常过坑并落轮。未验证FR失联、运动中全观测丢失、永久失联、断电或支撑轮失效。

| 故障 | 首次中止 | 真实最弱支撑（异常全过程） | 最大姿态 | 抬轮最低净空 | 响应 |
|---|---:|---:|---:|---:|---|
| 冻结完整观测包1 s后恢复 | 触发后约0.080 s | 1195 N | 7.424° | 48.0 mm | RECOVERED |
| 持续注入支撑QP INFEASIBLE | 触发后约0.520 s | 1193 N | 7.424° | 47.6 mm | RECOVERED |

两轮任务始终FAIL，保存首次异常原因。全过程无CRAWL/ABORT_EXIT，横移约1.46 cm，CoM与准静态ZMP安全三角形、RR低载、净空、姿态、行程、力与变化率均通过。在RR接地点约100.608 m的坑前固地停车落轮，未进入坑槽。QP试验不是强行切FSM：分配器持续返回失败，现有0.5 s连续失败计时触发中止；固地落轮的反馈判据仍可由正常传感器验证，不要求失效QP替它判定。

正常完整FR→RR回归PASS：周期81.201 s，共用路径最大横移2.099 cm，RR最弱支撑820 N、最小ZMP λ0.06151、坑上净空43.0 mm；与上一版本同工况一致。正常≥5 s保持等判据没有改变。

## 问题与修复

原包装器记录`measurement_time_s/measurement_age_s`，主控却未使用数据年龄，可以把旧的“安全”数据当作新反馈推进状态。

新增`FullRightSideController.set_feedback_timestamp`，由`NoisyFeedbackTrial`每次调用传入真实采样时间。集中`FEEDBACK_HEALTH`为最大包年龄0.12 s、撤驱动变化率500 Nm/s、故障记录20 ms。正常40 ms延迟加采样保持约40.5 ms，低于失联阈值；这只是当前仿真配置，未声称硬件规范。

数据超时、缺失、非有限或来自未来时：

1. 锁存原阶段中止状态与首次原因。
2. 冻结上一个最终实际悬架力和转向，驱动扭矩限速向零收敛。
3. 不运行QP，不更新FSM进度，不根据旧数据驶出/落轮；日志仍由独立车辆真值替换旧观测，保持审计。
4. 每个原生回调维护allocator的实际输出与应用时间；数据恢复不能把整个失联间隔用于扩大瞬间允许的力变化。
5. 数据恢复后重新执行原反馈、安全门槛和dwell，任务保持FAIL。

原生同步接口未提供外部数据包时间时沿用现有运行方式；真正外部数据链需要调用timestamp入口。不能通过“数值没有变化”判断失联，静止情况下相同数值也可能是有效新数据。

冻结输出不是持续稳定闭环，撤去驱动也不是已验证的运动中紧急制动。这两组试验原本静止，不能用于承诺盲停或任意故障稳定。后续运动中失联必须明确哪些观测/制动通道仍可靠，再设计有条件的降级控制。

## 复现

项目根目录PowerShell，每轮新建output。以下为数据包故障；改`--fault qp`即永久分配失败试验（1 s参数不限制QP失败时长）。需要active分配与噪声包装器，脚本不会人为直接切中止FSM。

```powershell
$env:PYTHONPATH='src'
python scripts/run_feedback_fault_trial.py --fault packet --output runs/recheck_packet_fault --efficiency-profile compact --lateral-ride-scale 0 --rear-control-profile robust --front-steering-feedback --front-target-speed-kph 3.4 --front-min-pit-speed-kph 3 --front-accel-ramp-s 1 --front-preload-reference-rate 4 --front-brake-lead-m 0.5 --rear-target-speed-kph 3.6 --rear-min-pit-speed-kph 3 --rear-accel-ramp-s 2.5 --measurement-noise --noise-seed 20261005 --average-path-reference --support-allocation active --support-allocation-transitions --support-allocation-gains evidence/static_fr_ii/support_gains_mu06_close_20261004.json --feedback-delay-s 0.04 --road-friction 0.6 --vehicle-start-offset-m 0.3
```

故障开始与中断时长默认集中在`FEEDBACK_FAULT_TRIAL`，可显式传`--fault-after-s`、`--packet-outage-s`；不把未实际注入故障的正常循环判为故障试验成功。使用正常`run_right_side_full_cycle.py`复核不注入故障的完整流程。

第一次包故障运行在结果导出时出现numpy bool无法JSON序列化，日志保留但未宣称该轮验收成功。修复判据/输出的Python标量契约并重复同一原生工况，成功与错误均记录在[精简证据](../evidence/static_fr_ii/feedback_fault_validation_20261005.json)。最终372项测试通过（11.66 s），含超时锁存、未来/缺失时间戳、实际输出保持/撤驱动限速与恢复历史维护；代码审查未发现重要问题。

## 下一步

优先明确运动中失联的可观测/可执行条件，验证有限故障的安全降级；不能假装完全失联时仍有悬架稳定反馈。与此同时整理正常PASS与故障RECOVERED的专家数据，二者不能混成正常成功训练标签。当前验证范围继续固定车型与有限工况，不扩大成任意故障/速度/附着保证。
