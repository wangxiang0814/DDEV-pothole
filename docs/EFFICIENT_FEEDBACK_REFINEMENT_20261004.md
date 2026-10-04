# 求解效率、异常净空与正常周期集中优化

## 研究范围

固定已独立调参的I_I副本、质量和载荷位置。现有TruckSim模型/联合仿真接口不变，原生dt=0.5 ms，控制更新20 ms。经典参考轨迹、轮载/姿态反馈、三支撑QP、速度PI和方向反馈，不用RL。标称μ0.7与μ0.6近起点是两组有限验证；控制目标用车辆真值验收，噪声/40 ms延迟只影响控制观测。

本轮按三个可定位问题集中修改，没有遍历坑宽深，也没有反复逐级小幅调参。

## 1. 同一个QP减少迭代调用

下层只有三个决策变量，目标是带正则的线性最小二乘。先用`numpy.linalg.lstsq`（SVD）求无约束全局最优解；仅当有限值、严格位于输入箱界内且全部线性约束在既有容差内满足时返回。轮载/CoM与ZMP三角形、姿态、行程、净空、力/变化率等检查保留。候选不满足时进入原可行性LP与SLSQP。没有普通inverse，也没有裁剪无约束解后冒充受约束最优解。

开关`SupportQPConfig.direct_feasible_solve=True`，关闭可复核旧迭代路径。summary增加`solver_counts`，`NONE`表示在迭代前已不可行，不能计作SLSQP调用。原数值迭代允许目标收敛误差，直接解更精确，因此完整轨迹可能有细微差异，需原生复核而不是假设逐步输出完全相同。

| 计算对照（同进程交替执行） | 原迭代p95 | 新路径p95 | 说明 |
|---|---:|---:|---|
| FR内部点 | 0.785 ms | 0.447 ms | 100次均直接返回 |
| RR内部点 | 0.799 ms | 0.451 ms | 100次均直接返回 |
| FR力箱约束激活 | 1.049 ms | 1.126 ms | 100次回退SLSQP |
| RR力箱约束激活 | 1.120 ms | 1.218 ms | 100次回退SLSQP |

该基准是合成接口工况，不是实车硬实时保证；最大输出差0.0086 N以内。它也显示回退场景稍慢，不能称任何情况都提速。原生低附着正常一轮1766/1797更新走直接路径（约98%），上下层分配p95为1.58 ms；最终6 s恢复工况1.48 ms，20 ms控制周期保持。

## 2. 异常恢复调整取舍

上一版本可选恢复1000 N包络降低侧倾，但最低净空只有14–19 mm。本轮试验750 N，在两组较早坑上中断中保持`RECOVERED`，三支撑轮载和三角形裕度更充分，净空29–31 mm，横滚仍低于10°。这是新求解器与750 N组合版本的对照，不能将全部变化归因于单个参数。默认不启用驶出时仍500 N，重新原生验证`SAFE_HOLD`；姿态9.913°离10°仍近，不宣称这一分支已拥有充分余量。

| 较早中断工况 | 版本 | 最低净空 | 最大横滚 | 最弱支撑 | 最小ZMP λ | 最大恢复横移 |
|---|---|---:|---:|---:|---:|---:|
| μ0.6/近起点 | 上版1000 N/迭代 | 14.6 mm | 9.222° | 933 N | 0.0696 | 2.15 cm |
| μ0.6/近起点 | 本版750 N/直接路径 | 29.0 mm | 9.235° | 1098 N | 0.0822 | 2.19 cm（最终6 s恢复） |
| 标称 | 上版1000 N/迭代 | 14.3 mm | 9.245° | 920 N | 0.0687 | 1.93 cm |
| 标称 | 本版750 N/直接路径 | 31.2 mm | 9.259° | 1107 N | 0.0828 | 1.98 cm（8 s恢复试验） |

最终低附着750 N/6 s恢复又完成原生复核，真实停稳后进入约3.28 s低速驶出，固地停车再落轮并恢复四轮。正常标称6 s恢复和异常标称750 N分别验证，未宣称最终标称组合又重复做过故障测试。故障任务仍FAIL，首次异常原因保留；RECOVERED只描述恢复动作。它们依赖反馈与执行器仍可用，不是断电/支撑失效认证。

## 3. 正常动作缩短2秒，并补全过程验收

只将compact配置的RR四轮恢复`COMPACT_REAR_RETURN_RAMP_S`从8改6 s。正常抬轮前卸载、4 s抬升/落轮、每轮≥5 s保持、停止dwell与速度目标均不变。baseline/balanced/fast配置未采用该缩短。正常完整循环都保留前轮过坑停稳落轮，再后轮过坑停稳落轮。

| 完整循环 | 周期 | 共用路径横移 | RR最弱支撑 | RR最小ZMP λ | RR坑上最低净空 |
|---|---:|---:|---:|---:|---:|
| 本版标称、RR −200 N偏差 | 81.98 s | 2.11 cm | 815 N | 0.06113 | 45.8 mm |
| 本版μ0.6、近起点 | 81.20 s | 2.10 cm | 820 N | 0.06151 | 43.0 mm |

低附着在同一求解器/参数下先8 s后6 s的对照周期83.20→81.20 s，其过坑段轨迹相同；恢复段最大姿态7.16→7.09°、最大速度0.053→0.044 km/h，支撑不卸载。标称上一版83.98→本版81.98 s也通过，但兼有求解器变化，不用于纯参数因果证明。提速约2.4%，不能把计算基准约43%的改善写成车辆动作提速43%。

发现原验收只检查最终四轮轮载，可能漏掉中间瞬态，新增正常恢复全过程审计：LOWERING/RETURN/COMPLETE的姿态、三支撑轮载、四角行程、共用路径偏差和固地停稳。阶段接地后不要求RR保持零载或三角形，避免把离地判据误用到落轮阶段；正常蠕行也不要求初始静稳角速度门槛。新试验保存的真值已重新验收，正常PASS，异常任务仍FAIL。

## 复现

在项目根目录执行，每轮更换output。以下为最终低附着正常配置；标称用默认附着/起点、seed20261001及标称增益，已测标称可额外加`--support-force-pulse-n -200`。

```powershell
$env:PYTHONPATH='src'
python scripts/run_right_side_full_cycle.py --output runs/recheck_efficient_mu06 --efficiency-profile compact --lateral-ride-scale 0 --rear-control-profile robust --front-steering-feedback --front-target-speed-kph 3.4 --front-min-pit-speed-kph 3 --front-accel-ramp-s 1 --front-preload-reference-rate 4 --front-brake-lead-m 0.5 --rear-target-speed-kph 3.6 --rear-min-pit-speed-kph 3 --rear-accel-ramp-s 2.5 --measurement-noise --noise-seed 20261005 --average-path-reference --support-allocation active --support-allocation-transitions --support-allocation-gains evidence/static_fr_ii/support_gains_mu06_close_20261004.json --feedback-delay-s 0.04 --road-friction 0.6 --vehicle-start-offset-m 0.3
```

异常复核改用`run_pit_abort_trial.py`并增加`--rear-abort-exit --abort-trigger-fraction 0.25`；默认悬空去掉驶出flag、比例0.5。当前集中配置为750 N/6 s，无需临时覆盖。中间试验通过本机tmp覆盖config运行，其完整实际配置保存在证据中；这些覆盖不是新默认。

```powershell
python scripts/benchmark_support_increment.py --samples 100 --output runs/recheck_support_benchmark.json
python -m pytest -q
```

最终370项Python测试通过（17.83 s），含可行直接解避开迭代、约束激活回退、20组小目标对照、恢复中途超限不能被最终接地掩盖。代码审查未发现重要问题。见[全部原生对照与配置](../evidence/static_fr_ii/efficient_feedback_refinement_20261004.json)、[计算基准](../evidence/static_fr_ii/support_increment_benchmark_20261004.json)。旧失败证据保留在前版恢复文档/证据，没有删除或重标。

## 当前限制与下一步

已测工况正常过坑无失败，横移在几厘米范围；不能保证未测任意速度、附着、载荷、车型或故障都成功。当前增益依赖匹配模型，卸载初中段还未统一进入三接触QP；RR最小λ约0.061仍接近0.05门槛，默认硬制动侧倾余量薄。

后续优先整理带明确有效范围的专家数据与版本来源，然后针对反馈失效/持续QP无解验证安全分支。继续使用少量有区分度的实验，保存失败、同参数比较、一次改动有明确物理目标。暂无必要优先扩大到4–7 km/h或遍历坑深宽，真实故障恢复若不可观测/不可驱动，不应盲目继续驶出。
