# 扰动验证、卸载效率与坑上中止闭环

## 本轮范围

沿用当前独立调参的I_I副本、固定质量和载荷位置。求解器dt=0.0005 s，控制/QP=0.02 s；未修改原始plant、轮序或接口。采用经典参考轨迹、轮载反馈、三支撑约束QP、姿态阻尼、速度PI与转向反馈。所有比较保持现有真实车辆验收线，不使用初始化静稳判据限制蠕行。

## 改动和算法

1. `SupportForcePulse`在分配后给一个支撑悬架加未知输出偏差，默认200 N；控制器只通过车辆反馈发现它。HOLD进入0.5 s后触发，quintic升0.5 s、保持1 s、降0.5 s，仅一次。FR选RL、RR选FL。日志区分原控制命令和实际Fact；偏差未完整施加、裁剪或被中断不能标作有效通过。异常快照保存最终实际力，避免重复加偏差。
2. RR起步由2→2.5 s，减轻加速载荷转移。目标仍3.35 km/h时最低坑内2.935 km/h，违反本轮≥3要求而FAIL；目标改3.6后实测约3.09–3.45 km/h并通过。没有降低速度验收线。
3. 新可选FR参考最大推进速率4：轮载误差<200 N时最多4倍推进；200–500 N回到原速；≥500 N暂停。因此缩短跟踪良好段，不是纯时间强行抬轮。默认compact仍3，抬升、保持、恢复和dwell不变。
4. RR坑上异常原逻辑冻结加速期间的悬架力，硬制动导致横滚12.57°和行程越界。现在RR_ABORT_STOP继续使用三支撑QP，并保持既定−7.2°姿态目标和角速度阻尼。异常入口只清一次已有修正：快照已包含原修正；下一次求解从该快照累计新增修正，防止双加。首次故障原因不被后续无解覆盖，也不反复重置停车dwell。
5. 正常新增修正速率400 N/s不能在制动瞬间及时阻尼，试验11.34°仍FAIL。异常单独1200 N/s、修正幅值仍500 N，力18200 N、姿态10°、行程[-149,155] mm、支撑500 N、λ0.05、净空10 mm等保持。执行层按每个真实callback的dt限制最终输出45400 N/s，而不是把20 ms平均速率当瞬时速率；落轮从最后实际修正平滑释放。

这仍是局部准静态QP，不是动态MPC。ZMP为轮载加权压力中心，CoM同时核验；尚未完整表达惯性动态稳定点。

## 原生证据与取舍

| 工况/调整 | 结果 | FR卸载 | 总周期 | RR最低轮载 / λ | 共用路径横移 |
|---|---|---:|---:|---:|---:|
| 标称，40 ms，RR 2.5 s/3.6，FR速率3 | PASS | 14.34 s | 88.12 s | 827 N / 0.06204 | 2.50 cm |
| 同工况，FR速率4 | PASS | 10.28 s | 83.98 s | 820 N / 0.06149 | 2.11 cm |
| μ0.6、起点提前0.3 m、40 ms、RR +200 N、FR速率4 | PASS | 10.28 s | 83.20 s | 826 N / 0.06196 | 2.10 cm |
| 标称同新配置，RR −200 N | PASS | 10.28 s | 83.98 s | 816 N / 0.06119 | 2.11 cm |

卸载缩短约28%，整个周期约4.7%；不是通过增加蠕行速度取得主要收益。标称FR4相比FR3的RR最低轮载略减约8 N，并非所有指标改善。未压缩每轮5 s三轮保持。

原配置下FR +200 N、RR +200 N分别完整PASS；低附着/延迟组合RR +200 N也PASS。相同工况配对分析：RR脉冲最大轮载偏离约99–103 N、姿态偏离约0.37–0.39°，控制力确有反向修正；脉冲后窗口轮载平均残差仍约60–73 N，不能称精确恢复原受力状态。FR对应偏离较小。窗口只是描述统计，不是额外验收或统计鲁棒性证明。见两份`force_pulse_*_20261004.json`。

坑上中止最终有限试验：任务FAIL、响应SAFE_HOLD。故障约67.9605 s，坑槽中段触发制动；最大横滚9.914°、最弱支撑1278 N、最小ZMP λ0.09547、最低净空42.7 mm；无持续QP无解，实际执行层最大45400 N/s，停稳连续约23.96 s。RR最后约102.034 m，坑远缘101.9 m，落轮需102.2 m，故不落轮。这是制动悬空保持的验证，不是自动驶出和四轮恢复。

同一最新配置的RR_HOLD坑前中止已原生复核：四轮恢复PASS、任务FAIL，首次原因保留；恢复阶段最大速度0.0488 km/h、20 ms实际悬架力最大变化20.55 N。全套348项Python测试通过，包含原生步长力速率与实际修正释放的回归。测试环境视频编码器缺失有提示，现有测试通过备用路径；本轮没有导出视频，不据此宣称H.264导出正常。

## 复现

在项目根目录运行，输出目录必须全新。普通运行：

```powershell
$env:PYTHONPATH='src'
python scripts/run_right_side_full_cycle.py --output runs/new_trial --efficiency-profile compact --lateral-ride-scale 0 --rear-control-profile robust --front-steering-feedback --front-target-speed-kph 3.4 --front-min-pit-speed-kph 3 --front-accel-ramp-s 1 --front-preload-reference-rate 4 --front-brake-lead-m 0.5 --rear-target-speed-kph 3.6 --rear-min-pit-speed-kph 3 --rear-accel-ramp-s 2.5 --measurement-noise --noise-seed 20261001 --average-path-reference --support-allocation active --support-allocation-transitions --support-allocation-gains evidence/static_fr_ii/support_allocation_gains_20261003.json --feedback-delay-s 0.04
```

标称−200 N试验加`--support-force-pulse-n -200`；FR脉冲另加`--support-force-pulse-stage FR`。低附着/近起点加`--road-friction 0.6 --vehicle-start-offset-m 0.3 --noise-seed 20261005`，增益改为`evidence/static_fr_ii/support_gains_mu06_close_20261004.json`，可加`--support-force-pulse-n 200`。

坑上故障入口替换为`python scripts/run_pit_abort_trial.py`，其余采用低附着新配置、无力脉冲。按RR轮心处于坑长中点触发，最多观察25 s。响应审计用车辆真值，区分RECOVERED、SAFE_HOLD和FAIL；必须完整观测、最终连续停稳≥5 s、最终模式仍RR_ABORT_STOP才允许SAFE_HOLD；未结束的LOWERING/RETURN、移动或原生失败不能误判安全。

坑前恢复入口替换为`python scripts/run_rear_recovery_trial.py --abort-phase RR_HOLD`。测试视频沿用已有VS Visualizer脚本按轮次打开，不导出。

## 下一步与限制

- 保留本轮通过配置为明确可复现候选，默认旧基线仍可比较；只发布实际测试范围。
- 坑上中止只证明当前工况停车悬空；最大横滚距10°仅约0.086°，需继续降低制动瞬态并验证受控驶出后的落轮，不能据此宣布全面恢复成功。
- 宽深边界旧配置已测，但本轮QP组合尚未重测。全文件哈希严格匹配，不能给改变场景的模型伪造增益来源；先准备匹配复现实物，再做一组边界，不遍历网格。
- 卸载初中段仍用现有轮载反馈，未完成全阶段统一协同。更大延迟、更多脉冲方向/幅值、支撑丢失/传感器失效、其他车辆及4–7 km/h范围未验证。
- 训练数据批量生成在稳定范围确认之后；当前不训练AI。成功和中止分别标注，完整runs只在本机保存，Git保留配置、代码、精简成功/失败证据。
