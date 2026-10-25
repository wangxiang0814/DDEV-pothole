# 策略开发与运行入口

2026-10-24更新：MPC已退出主开发路线，专属代码与证据移至[可校验归档](../archive/mpc_20261013/README.md)。以下带2026-10-13日期的段落为历史记录。当前预瞄前馈与反馈开发见[实现设计](superpowers/specs/2026-10-24-shallow-preview-feedback-design.md)。

当前专家二已有独立的预瞄前馈与反馈控制器、原生 worker、研究筛查和播放接口。`implemented=True`表示代码可运行；`SIMULATION_PASS`只表示明确参数与模型下通过研究筛查，完整实车能力与底盘安全验收仍未完成。

2026-10-25深坑优先研究见[本轮报告](SHALLOW_DEEP_COORDINATION_20261025.md)。72项整数速度/深度矩阵与重复、步长、时延、中间速度分开统计；失败对照不混入活动参数。新增`probe_shallow_extension.py`用于有界静态行程诊断，`analyze_shallow_contact.py`用于已有完整原生结果的接触事件分析；二者都不授予完整安全PASS。MF-Swift授权不可用，当前不默认尝试该分支。精确冻结源码压缩快照及索引随本轮证据保存，复现旧计划须在独立目录恢复对应原始字节，不能改旧SHA绕过校验。

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

准备命令仅生成场景，状态为`PREPARED/UNASSESSED`，不生成控制通过结论。当前控制仿真使用独立入口：

```powershell
python scripts/run_strategy.py --strategy shallow_press --case right_track --depth-m .04 --run-id press_trial_001 --config configs/shallow_press/preview_feedback_20261024.json --tire-model mf52_research --delay-s .04 --duration-s 40
python scripts/open_run_visualizer.py runs/shallow_press/right_track/press_trial_001
```

用户2026-10-24批准独立MF52作为专家二研究基线，默认`mf52_research`，基线身份为`expert2_mf52_research_20261024`，不能当作与原车型物理等价。`--tire-model internal`显式使用原内部PAC52；当前没有原内部模型的合格通过结论。`mf52_compare`保留原未采纳对照语义；采纳前的历史记录不改写。各类运行的车型身份、模型修改、代码、配置和生成资源哈希均随运行保存。初始静止、前后轮驶出、停车保持和完整时长是独立判据；历史视频可直接播放，无需导出文件。

五相机研究采集可加`--demonstration-config configs/shallow_press/five_camera.json`。实际九路控制量、策略和对照模型身份随episode保存；未完成完整验收的episode不能标记为已接受训练样本。

固定参数批量验证入口为`scripts/run_shallow_preview_matrix.py --plan <frozen_plan.json> --workers 2`。计划冻结配置和代码哈希；参数漂移、重复目录或不匹配的续跑均拒绝。每个求解器在独立子进程中运行，线程仅负责调度。`--resume`只复用同一计划且结果哈希完整的已完成运行；批量进程成功不代表各工况筛查通过，需读取`matrix_summary.json`中的逐项结论。

## 浅坑控制开发约束

1. 只在 `strategies/shallow_press/` 实现状态机、悬架/转矩控制、配置和专属验收；补齐公共 reset/step/status/summary 协议。
2. 使用同一 `tuned_ii_20261005` 车辆版本、九路能力和 SI 观测合同。浅坑调深度、宽度不能偷偷变质量、载荷、止挡或低速轮胎参数。
3. 配置通过实例传入，不改模块全局对象。增益和参考数据按车辆与工况核验，不能自动借用抬轮的三支撑增益。
4. 已实现静止初始化、蠕行进入、下压/接触跟踪、爬出、停车保持及反馈失效时的受限卸载。状态属于实例；车身外环20 ms、电机轮速环2 ms，均与原生积分步长分离。
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

2026-10-11 底盘诊断扩展见 [CHASSIS_CONTACT_DIAGNOSTICS_20261011.md](CHASSIS_CONTACT_DIAGNOSTICS_20261011.md)：用户确认缺少真实尺寸，采用显式假定包络对18工况作90组历史净空分析；独立接触重跑直接施加原生车身参考点力。扩展模型不等于原车辆身份，不得借用旧辨识增益或标记实车触底验收通过；点模型、刚度及几何尚需标定。

2026-10-12 分阶段问题与数值敏感性研究见 [PASSIVE_PROBLEM_STUDY_20261012.md](PASSIVE_PROBLEM_STUDY_20261012.md)。接触诊断新增 `--step-s`（0.0005/0.00025/0.000125），仅修改本次隔离模型并保持1 ms日志；`scripts/analyze_passive_problem_study.py`校验冻结矩阵后生成原基准、接触与零反力对照的阶段指标、事件和工况索引。下压专家仍未实现。

2026-10-12 经典下压控制的文献研究和详细设计见 [SHALLOW_PRESS_CLASSICAL_DESIGN_20261012.md](SHALLOW_PRESS_CLASSICAL_DESIGN_20261012.md)。用户确认自适应降速（先 1–3 km/h）、首阶段已知坑形、能力参数与专家一对齐。文献来源与能力快照位于 `evidence/shallow_press_design_20261012/`，是设计证据，不是运行配置或控制验证结果；`shallow_press` 仍为 NOT_IMPLEMENTED。

2026-10-13 MPC前置模型验证见 [SHALLOW_MPC_VALIDATION_20261013.md](../archive/mpc_20261013/README.md)。完成30条独立原生矩阵运行、67状态原生线性化及多步预测/步长对照；局部平地模型有用，但当前统一预测器18条过坑验证均未通过轮载/接触门控。新增辨识/预测研究模块不启用专家入口；MPC闭环尚未释放，`shallow_press`仍为NOT_IMPLEMENTED。正式证据采用 `evidence/shallow_mpc_validation_20261013/verified_analysis/`。

2026-10-24 多速度调优见[SHALLOW_MULTISPEED_TUNING_20261024.md](SHALLOW_MULTISPEED_TUNING_20261024.md)。新候选需显式传入`--config configs/shallow_press/preview_multispeed_v2_20261024.json`。两类坑保持同一配置，并按请求速度调度预瞄整形；实际通过范围读取该版本矩阵，不能拼接旧配置结果。用户批准的更快转矩扫描单独标识，不能覆盖专家一1000 Nm/s能力基线或统计为普通通过。
