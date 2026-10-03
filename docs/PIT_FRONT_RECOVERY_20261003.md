# RR 坑前阶段中止恢复验证

## 发现与修复

原生试验在 RR PRELOAD 开始 1 s 后注入阶段中止。原代码把 ABORT 当作已经有抬轮时间的模式，尚未进入 LIFTING 时会抛出 `lift_start_s` 未初始化异常。这也是此前只测试手工设置的 ABORT 状态未覆盖的问题。

修复：初始化抬轮时间为空，只在实际开始抬轮后计算其轨迹；进入 ABORT_STOP 时保存当前实际输出的四角悬架力。中止期间冻结这四个力，不继续推进卸载参考；在固地、停车、支撑稳定条件及 dwell 满足后，支撑力用原恢复时长的 quintic 逐渐恢复，抬起角按原落轮时长恢复。原因始终保留。正常非异常循环使用原指令逻辑。

新增回归测试先复现未初始化异常，再验证力冻结、恢复连续性及四轮恢复；原生修复后复核：

- 恢复 PASS，最终 RR_COMPLETE、四轮正载恢复。
- 完整过坑任务 FAIL，原因 `injected pit-front preload abort`；RR 没有完成过坑，不能将恢复标作正常成功。
- 中止入口前一条记录至结束，最大速度约 0.00268 km/h；相邻 20 ms 悬架力最大变化约 0.355 N，包含中止入口。
- 本实验只验证坑前阶段中止，不是支撑丢失、传感器失效或坑上恢复证明。

## 复现

```powershell
$env:PYTHONPATH='src'
python scripts/run_rear_recovery_trial.py --output runs/pit_front_recovery_recheck --efficiency-profile compact --lateral-ride-scale 0 --rear-control-profile robust --front-steering-feedback --front-target-speed-kph 3.4 --front-min-pit-speed-kph 3 --front-brake-lead-m 0.5 --rear-target-speed-kph 3.35 --rear-min-pit-speed-kph 3 --measurement-noise --noise-seed 20261001
```

注入只在本试验入口生效，正常完整循环入口不会自动注入故障。修复后带噪正常完整循环已复核 PASS（88.48 s、共用路径横移 2.18 cm），见 `evidence/static_fr_ii/normal_after_recovery_fix_20261003.json`。其他故障恢复仍按整体方案继续验证。

证据：[原生任务结果与独立恢复报告](../evidence/static_fr_ii/pit_front_recovery_20261003.json)。
