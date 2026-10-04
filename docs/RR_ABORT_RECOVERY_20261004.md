# RR坑上中断的受控恢复

## 范围与实际结果

沿用独立调参的I_I仿真副本、固定质量和载荷、现有TruckSim plant和接口。原生步长0.5 ms，控制更新20 ms。经典控制，未引入RL或完整MPC。这里注入的是已知坑槽位置触发的流程中断，执行器、观测和反馈仍可用；不等同于断电、传感器丢失、支撑轮失效的恢复认证。

| 原生测试 | 响应 | 最大横滚 | 抬轮状态最低净空 | 恢复全程最大横移 | 实际进入低速驶出 |
|---|---|---:|---:|---:|---|
| 早期中断，μ0.6、起点前移0.3 m | RECOVERED | 9.222° | 14.6 mm | 2.15 cm | 是，约3.28 s |
| 早期中断，标称μ0.7 | RECOVERED | 9.245° | 14.3 mm | 1.93 cm | 是，约3.28 s |
| 坑中中断，μ0.6、近起点 | RECOVERED | 9.161° | 18.8 mm | 2.50 cm | 否，制动已到固地 |
| 默认停车悬空，μ0.6、近起点 | SAFE_HOLD | 9.914° | 42.7 mm | 1.92 cm | 否，终端连续停稳23.96 s |
| 正常完整FR→RR回归，标称、RR −200 N输出偏差 | PASS | RR 9.116° | RR 45.9 mm | 2.11 cm | 无异常，周期83.98 s |

以上均有观测噪声、40 ms反馈延迟；验收读取车辆真值。异常任务仍FAIL，首次原因不被恢复覆盖。RECOVERED只表示固地停车落轮并恢复四轮；SAFE_HOLD表示在有界观察时段保持安全停车悬空。保存完整失败/中间试验和配置、模型SHA、增益来源、阶段时间的[精简证据](../evidence/static_fr_ii/rr_abort_recovery_20261004.json)，大体积原生历史留在本机runs，不上传GitHub。

## 控制流程与阶段判据

1. **RR_ABORT_STOP**：保存最终实际悬架力快照，持续三支撑QP。可选恢复模式在支撑与QP健康时，以测得车速为起点给1 s quintic降速参考；速度PI负责制动。不安全时直接给零速目标，不瞬间清零悬架力。停止判据是实测速度，不依赖固定计时。
2. **连续2 s恢复准备**：实测停稳、三个支撑轮载≥500 N、CoM与准静态ZMP的三角形λ≥0.05、RR低载≤100 N、净空≥10 mm、姿态/行程安全、横移≤5 cm、QP反馈健康。任一失效重新累计。初始静稳角速度门槛不用于蠕行控制。
3. **RR_ABORT_EXIT**：仅在显式启用恢复、轮下仍是坑槽时进入。0.7 km/h速度目标用1 s quintic起步，速度PI和方向反馈持续修正，RR不施加驱动扭矩；三支撑QP保持悬架反馈。到坑末端+既有0.3 m固地余量+0.15 m停车缓冲后再制动。
4. **再次RR_ABORT_STOP**：重新保存实际力快照，在固地真实停车且原安全dwell满足后落轮。不得驶出后直接落轮。健康/支撑/路径门槛失败或驶出超过10 s，重新停车；最多一次驶出尝试，不自动反复起步。
5. **RR_LOWERING→RR_RETURN→RR_COMPLETE**：沿原平滑恢复逻辑落轮并恢复四接触。离地低载/净空判据只用于抬轮状态；落轮后检查四轮恢复。姿态、支撑轮载、行程、力限、原生力变化率及路径偏差审计覆盖整个异常后过程。

支撑力修正幅度与速率集中在config：正常和默认停车悬空修正包络500 N，启用受控恢复的异常模式1000 N，异常QP修正速率1200 N/s。执行层整个异常及落轮过程都保持既有±18.2 kN软件限幅和45.4 kN/s原生输出变化率，不是硬件额定值。底层保留实际输出限速，避免20 ms QP变化在0.5 ms回调中造成跳变。ZMP仍是轮载加权准静态估计，同时核验CoM，不宣称完整动态惯性ZMP。

## 诊断、失败与取舍

在同一较早中断的低附着工况，1 s制动、500 N修正的横滚9.614°，修正达到边界；增加到1000 N后9.222°。最低净空从39.7降到14.6 mm，仍通过10 mm门槛但余量更薄。因此仅显式开启恢复模式，不覆盖所有异常。

把1000 N直接用于默认硬制动曾FAIL：净空7.06 mm，即使停车与侧倾都通过也不能判成功。已恢复该分支500 N，原生重跑SAFE_HOLD，未放宽净空或姿态验收。较晚中断直接在固地停车，不能作为“停车后低速驶出”的证据；真正驶出由两组较早中断测试证明。

正常全循环回归的周期与原版本相同：该变更改善中断恢复，不用于宣称正常动作继续提速。每轮正常保持≥5 s不变。

最终`PYTHONPATH=src python -m pytest -q`：366 passed（11.80 s）。补充连续dwell、失效停止/单次尝试、力快照交接、固地停稳后落轮、阶段制动参考和默认/可选修正包络回归。只读代码审查未发现重要问题；所有保存的中断结果用最终全过程审计重核，拒绝试验仍因净空失败。

## 复现命令

在项目根目录PowerShell执行，使用新的输出目录。下面是较早中断的μ0.6近起点恢复；标称改为默认附着/起点、seed20261001与标称增益。

```powershell
$env:PYTHONPATH='src'
python scripts/run_pit_abort_trial.py --output runs/recheck_rr_early_recovery --rear-abort-exit --abort-trigger-fraction 0.25 --efficiency-profile compact --lateral-ride-scale 0 --rear-control-profile robust --front-steering-feedback --front-target-speed-kph 3.4 --front-min-pit-speed-kph 3 --front-accel-ramp-s 1 --front-preload-reference-rate 4 --front-brake-lead-m 0.5 --rear-target-speed-kph 3.6 --rear-min-pit-speed-kph 3 --rear-accel-ramp-s 2.5 --measurement-noise --noise-seed 20261005 --average-path-reference --support-allocation active --support-allocation-transitions --support-allocation-gains evidence/static_fr_ii/support_gains_mu06_close_20261004.json --feedback-delay-s 0.04 --road-friction 0.6 --vehicle-start-offset-m 0.3
```

试验至完成或中断后45 s停止。去掉`--rear-abort-exit`、触发比例改0.5，复核默认停车悬空（25 s观察）；必须更换输出目录。正常回归使用`run_right_side_full_cycle.py`，不传试验触发参数。受控恢复要求active在线支撑分配和匹配增益文件，脚本核验配置和来源。

## 剩余工作顺序

1. 整理可复现专家策略工况包、版本/配置/场景来源与日志字段，明确任务PASS、恢复RECOVERED与任务FAIL的区分，供后续训练筛选。
2. 选择少量重要的真实异常测试（例如反馈失效、持续QP无解），先明确可恢复条件；当前几何触发不能替代这些测试。重点保住净空和支撑余量，不靠放宽验收。
3. 在当前固定车型范围做有限补充复核，再决定是否扩大速度。当前已测完整过坑目标3.4/3.6 km/h，尚无4–7 km/h通用验证。

卸载初中段仍不是全阶段统一QP，10°姿态线下余量有限，局部增益依赖当前模型。当前成果不能写成任意载荷/车型/附着/故障泛化。无需新增用户决策即可继续整理数据与有限复核；改变研究对象或安全目标时再讨论。
