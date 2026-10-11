# 策略开发与运行入口

所有命令从本 Git 项目根目录执行。使用能运行现有 requirements.txt 的 Python 环境；本轮验证环境为 F:\anaconda3\python.exe（Python 3.8.5）。

## 抬轮策略

新入口默认选最新推荐清单，输出自动按策略、工况、编号隔离：

```powershell
python scripts/run_strategy.py --strategy wheel_lift --case seed_repeat --dry-run
python scripts/run_strategy.py --strategy wheel_lift --case seed_repeat --run-id lift_trial_001
python scripts/open_run_visualizer.py runs/wheel_lift/seed_repeat/lift_trial_001
python scripts/export_native_video.py runs/wheel_lift/seed_repeat/lift_trial_001 --no-launch
```

需要源模型证明的环境/坑形复用工况仍需显式 `--reference-model`。`--dry-run` 仅显示命令与来源身份，不认领目录。重复运行编号会报错，不能覆盖。求解依赖 TruckSim 2019 Browser/TSLM 及有效许可证。

旧入口继续可用，保持原默认值。相同参数必须产生相同模型与算法轨迹；调参修改只进入 `configs/wheel_lift/` 与该策略目录。旧诊断脚本仍可通过兼容模块替换控制器，但这些替换属于诊断，不作为新的共享控制接口。

## 浅坑场景

两种模板不预填正式坑深、纵向长度和横向宽度。以下是架构验证示例尺寸，不是已认证工况或推荐控制参数：

```powershell
# 单侧轮迹：FR -> RR
python scripts/prepare_strategy_case.py --case right_track --depth-m .05 --length-m .8 --width-m .9 --run-id shallow_scene_001
# 宽坑：FL/FR -> RL/RR
python scripts/prepare_strategy_case.py --case wide --depth-m .05 --length-m .8 --width-m 1.8 --run-id shallow_wide_001
# 只校验配置，不写文件
python scripts/prepare_strategy_case.py --case wide --depth-m .05 --length-m .8 --width-m 1.8 --dry-run
```

可显式传 `--approach-distance-m`、`--friction`、`--stop-s` 或自定义 `--template`。初始化速度为零，校验轮胎包络在坑前固地、单侧排除左轮、宽坑覆盖两侧。生成 model/、地形/显示网格和运行清单。

当前调用 `run_strategy.py --strategy shallow_press ...` 会明确拒绝原生控制仿真。准备命令本身不生成控制结果或原生历史。本次两个示例额外生成了明确标注的 2 s 静态基础设施诊断历史，不能作为下压通过策略的控制结果。

## 实现浅坑控制律

1. 只在 `strategies/shallow_press/` 实现状态机、悬架/转矩控制、配置和专属验收；补齐公共 reset/step/status/summary 协议。
2. 使用同一 `tuned_ii_20261005` 车辆版本、九路能力和 SI 观测合同。浅坑调深度、宽度不能偷偷变质量、载荷、止挡或低速轮胎参数。
3. 配置通过实例传入，不改模块全局对象。增益和参考数据按车辆与工况核验，不能自动借用抬轮的三支撑增益。
4. 实现静止初始化、蠕行进入、下压/接触跟踪、爬出、停车/恢复及异常处理的控制行为；然后将 implemented 改为 true，并在新入口增加其独立 worker。
5. 根据实际可观测量分别验收底盘/悬架触底、姿态稳定、低速滑移、路径、爬出与平顺性。缺少底盘接触观测时记录未评估，不能用 Jnc 行程代替底盘净空。阈值在这一开发阶段明确。
6. 先验证单侧，再验证宽坑；后轴尚未通过不能判全任务完成。记录失败与异常恢复，任务成功和恢复成功分开。

## 回归与公共接口修改

```powershell
$env:PYTHONPATH='src'
python -m pytest -q
python -m pytest tests/architecture tests/shallow_press -q
git diff --check
```

修改共享合同、车型生成、物理/显示地形或视频工具时运行相关回归和全量测试。对行为改动补能够暴露实例泄漏、错误轮迹、输出覆盖或资源混用的测试。不能为了得到 PASS 放宽旧策略验收或修改历史证据。

可恢复的重构前版本：`backup/pre-strategy-refactor-20261008`（647d168）。本机 bundle 位于 `.git/backups/pre-strategy-refactor-20261008.bundle`；它包含 Git 历史，不含被忽略的 runs 仿真产物。

2026-10-08 坑深研究与被动蠕行诊断见 [SHALLOW_DEPTH_BASELINE_20261008.md](SHALLOW_DEPTH_BASELINE_20261008.md)。0.10 m仅是暂定研究分区，未实现控制切换器或浅坑专家；实验独立入口为 `scripts/run_shallow_depth_baseline.py`，原生结果为UNASSESSED。

2026-10-11 新增约10 km/h、4–20 cm诊断及匹配3 km/h对照见 [SPEED_DEPTH_BASELINE_20261011.md](SPEED_DEPTH_BASELINE_20261011.md)。入口增加可选12 m起步、直行转向反馈与1 ms日志，旧默认不变。原宽坑16/20 cm的保护停止历史保留；按用户要求以 `--observe-after-attitude-limit` 关闭诊断姿态停止后完整重跑，最终汇总采用完整数据。工况名称及日后重放来源见 `evidence/speed_depth_baseline_20261011/case_index.md`。
