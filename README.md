# TruckSim–Python 四角独立悬架研究工程

## 当前控制对象与进展

### 最新开发状态（2026-10-03）

**本轮：当前约 90 s 配置完成四个带噪代表工况复核，全部 PASS。**新增远起点与低附着不同种子两轮原生测试，合并既有标称和近起点证据：动作周期 89.90–90.80 s，共用原始路径下 FR/RR 蠕行及停车最大横移 2.17–2.27 cm；RR 最低坑上净空 28.84–32.99 mm，RR 最大偏航 0.35–0.48°。固定质量/载荷，四轮恢复、支撑三角形和每轮 ≥5 s 保持均通过。278 项 Python 测试重新通过。见[四工况精简证据](evidence/static_fr_ii/robustness_matrix_20261003.json)。这只是指定工况复核，不是任意扰动鲁棒性证明。

| 本轮新增工况（compact / 横向表 scale=0） | 结果 | 动作周期 | 共用路径最大横移 | RR 最低坑上净空 |
|---|---|---:|---:|---:|
| μ=0.7，起点后移 0.3 m，seed=20261003 | PASS | 90.78 s | 2.23 cm | 31.26 mm |
| μ=0.6，标称起点，seed=20261004 | PASS | 90.62 s | 2.27 cm | 28.84 mm |

后续[完整六阶段方案](docs/superpowers/plans/2026-10-03-expert-control-next-stages.md)：①当前版本代表工况复核；②缩短卸载与调姿；③重新辨识当前车型并接入支撑悬架协同约束反馈；④确定工况/速度范围；⑤异常恢复原生验证；⑥可复现专家策略与数据输出。第一阶段指定四工况已完成，后续阶段尚未完成；优先 FR 卸载耗时和支撑稳定余量，不强制均载，也不优先追求更高蠕行速度。当前仍是经典参考轨迹与局部反馈，在线 QP 尚未完整接入。

**时间优化已开始：**同一带噪标称工况，仅将 FR 最大参考推进速度 2.5→3.0，原生完整循环 PASS；FR 卸载 17.02→14.30 s，总周期 90.80→88.48 s，横移 2.18 cm。现有误差减速/暂停、保持和恢复配置保留。这是内存配置覆盖试验，未修改推荐 compact 配置；待薄弱工况复核后再决定采用。见[对照证据](evidence/static_fr_ii/front_timing_trial_20261003.json)与[复现方法](docs/FR_TIMING_TRIAL_20261003.md)。

**最新：在明确调整悬架横向运动学的运行副本上，偏航改善且周期缩短。**推荐显式使用 `compact`、`--lateral-ride-scale 0`、RR `robust` 与 `--front-steering-feedback`。这是仿真参数调整：仅将四角悬架升降引起的轮心横向移动表缩放为 0，质量／载荷、原始车型文件和其他运动学表不变；CLI 默认仍为旧值 0.5，不能将本轮结果解释为旧模型纯控制器修复或泛化成功。

| 最新噪声工况（横向表 scale=0） | 全循环 | 动作周期 | 全循环最大横移 | RR 最大偏航 | RR 最低净空 |
|---|---|---:|---:|---:|---:|
| μ=0.7，标称起点，seed=20261001 | PASS | 90.80 s | 2.17 cm | 0.37° | 32.18 mm |
| μ=0.6，起点提前 0.3 m，seed=20261002 | PASS | 89.90 s | 2.18 cm | 0.35° | 32.99 mm |

原约 102 s 周期缩短约 11–12%。主要是 FR 卸载约 24.5→16.5 s、RR 四轮恢复约 11.7→7.8 s；FR 恢复仍保留原时长，两次三轮保持各 ≥5 s，蠕行速度不变。RR 预载至停车的最大横移与偏航也已核查。新增坑前静态异常的安全落轮恢复入口，保留 FAIL 原因。278 项 Python 测试通过。见[偏航与时间分析](docs/TIMING_AND_YAW_20261001.md)、[包含失败尝试的证据](evidence/static_fr_ii/efficiency_yaw_matrix_20261001.json)。

```powershell
python scripts/run_right_side_full_cycle.py --output runs/compact_recheck --efficiency-profile compact --lateral-ride-scale 0 --rear-control-profile robust --front-steering-feedback --front-target-speed-kph 3.4 --front-min-pit-speed-kph 3 --front-brake-lead-m 0.5 --rear-target-speed-kph 3.35 --rear-min-pit-speed-kph 3 --measurement-noise
```

**上一轮噪声验证（scale=0.5）：标称 PASS、组合 FAIL。**标称最大横移 2.75 cm；μ=0.6／起点提前 0.3 m／另一噪声种子组合横移 8.24 cm。轮角保持、静止转向及航向转矩反馈诊断均未有效解决，未保留新增失败控制分支。见[上一轮说明](docs/NOISE_ROBUSTNESS_20261001.md)及[证据](evidence/static_fr_ii/noise_matrix_20261001.json)。

先前配置为 `balanced` + RR `robust` + `--front-steering-feedback`，横向表 scale=0.5。FR/RR 使用同一原始路径基准，保持固定质量和载荷位置。实际控制采用经典参考轨迹、轮载反馈、速度 PI、转矩/转向方向反馈及停车行程反馈；**尚未完整接入在线 QP，不宣称任意工况泛化成功**。

| 无测量噪声的已复核工况 | 全循环结果 | RR 最低坑上净空 | RR 最弱支撑轮载 | 相对共用路径最大偏差 |
|---|---|---:|---:|---:|
| μ=0.7、标称起点、坑内约 3 km/h | PASS | 28.46 mm | 826 N | 4.32 cm |
| μ=0.7、起点后移 0.3 m、坑内约 3 km/h | PASS | 26.90 mm | 823 N | 4.51 cm |
| μ=0.6、起点提前 0.3 m、坑内约 3 km/h | PASS | 29.04 mm | 854 N | 4.27 cm |

原 3 km/h 基线 RR 净空约 15.44 mm、最弱支撑约 737 N。新版本标称完整动作约 101.8 s，原基线约 109.3 s；完成后观察 2 s 即结束仿真，避免无效运行至 180 s。FR/RR 各保持 5 s，落轮与恢复时间未压缩。

上一轮主要修复：FR 蠕行/停车接入现有转向通道反馈，FR→RR 保留同一路径基准，FR 异常后不继续启动 RR。停车行程反馈和抬轮姿态辅助继续保留。三个无噪声代表工况全部完成两轮过坑、停车和四轮恢复，RR 横滚峰值约 9.00–9.35°，仍接近 10° 边界；μ=0.4、3.5–4 km/h 的旧 FAIL 尚未用新版本复核，暂不发布为可靠范围。

**旧路径指标需正确理解：**旧 RR 阶段在 FR 恢复后重设路径零点，3.35/3.87 cm 是 RR 阶段新增偏差。按原始共用路径复核，这两轮约为 6.75/6.80 cm，旧 PASS 不证明整个流程 ≤5 cm。最新控制和验收均保留同一基准；标称累计偏差已降至 4.32 cm，原远起点失败工况降至 4.51 cm。历史结果原文保留，新增统一基准审计。

停车轮速阻尼、自适应预载、RL 参与预载、静止转向和分阶段预载已作针对性诊断，均未解决对应横移或引入其他失败，不作为默认配置。此前测试为 274／277 项，本轮为 278 项；代码审阅指出的停车转矩交接和真值路径基准问题已修复。

后续优先验证新仿真参数配置的其他起点／噪声种子，再完善约束分配与初始小偏差反馈。保留偏航原问题的模型范围说明；提速和进一步压缩应以稳定余量为依据。详见[历史无噪声反馈报告](docs/SHARED_PATH_FEEDBACK_20261001.md)、[共用路径证据](evidence/static_fr_ii/shared_path_matrix_20261001.json)及[上一轮报告](docs/RIGHT_SIDE_ROBUSTNESS_20261001.md)。下文为历史阶段成果；其中双轮 PASS 使用当时分阶段路径验收，不能直接代替最新共用路径验收。

```powershell
python scripts/run_right_side_full_cycle.py --output runs/shared_path_recheck --efficiency-profile balanced --rear-control-profile robust --front-steering-feedback --front-target-speed-kph 3.4 --front-min-pit-speed-kph 3 --front-brake-lead-m 0.5 --rear-target-speed-kph 3.35 --rear-min-pit-speed-kph 3
```

唯一当前车型是 TruckSim 2019 `corner_module_ddev`（Compact Utility Truck，`I_I` 前后独立悬架，约 1.36 t）。四轮独立转矩和四角主动悬架力通过原有八通道接口输入；没有更换 TruckSim plant。求解器 `tstep=0.0005 s`，`simfile.sim` 的 `EXT_MODEL_STEP=0.01 s` 是另一项设置，不是求解积分步长。当前试验的 200 kg 载荷后移 350 mm、左移 500 mm，仅在随附的隔离平地模型中使用。

**已完成原生静态验证：**平地、零电机转矩，FR 卸载并离地，FL/RL/RR 三轮正载，CoM 与轮载 ZMP 位于支撑三角形内，FR 净空超过 10 mm 持续 5.66 s，随后平滑落轮并恢复四轮。保持段 FL/FR/RL/RR 轮载约为 `6396/0/1229/5710 N`，ZMP 最小重心坐标裕度约 `0.092`。可审阅的[精简证据](evidence/static_fr_ii/lift_summary.json)与[轨迹图](evidence/static_fr_ii/lift_evidence.png)随仓库保存。

**已完成单工况三轮蠕行：**在隔离的 I_I 模型和现有右轮迹坑槽上，预载轨迹按实测轮载误差加快／暂停，FR 离地后保持 5 s，以 2.04–2.13 km/h 跨坑，停车后落轮，原生 TruckSim 结果为 `PASS`。抬轮开始时间从约 48 s 缩短至 34.34 s；运动／停车的三支撑轮最小轮载约 881 N，最小轮载稳定点重心坐标裕度 0.0665，最大横向偏差约 2.84 cm。控制器 50 Hz 更新；求解积分步长仍为 0.5 ms。见[精简结果](evidence/static_fr_ii/closed_loop_2k_summary.json)。

**已完成 3 km/h 档单工况验证：**固定载荷和同一坑槽下，目标速度 3.3 km/h，坑内实测 3.13–3.15 km/h；FR 净空最低 37.3 mm，三支撑轮最低轮载 730 N，运动中最小轮载压力中心三角形裕度 0.05495，横向偏差最大 3.55 cm。FR 越过坑槽后车辆停稳，RR 在坑前约 0.21 m，随后落轮并恢复四轮，原生结果为 `PASS`。减速从 FR 接近坑槽远缘时开始，落轮仍要求 FR 完全越过且 RR 留在坑前。见[3 km/h 精简结果](evidence/static_fr_ii/closed_loop_3k_summary.json)。4–7 km/h 尚未经验证；当前坑前停车距离是主要限制。

**FR→RR 完整循环已有单工况 PASS：**`scripts/run_right_side_full_cycle.py` 在一个原生 TruckSim 运行中串联 FR 与 RR 的卸载、抬轮、三轮稳定保持、约 2 km/h 右轮迹过坑、停车、落轮和四轮恢复。RR 使用 FL/FR/RL 支撑三角形。最近的完整运行中，RR 保持 5 s，三支撑轮最低 942 N，最小 ZMP 三角形坐标 0.0707，坑上最低净空 39.9 mm，最大横向偏差 2.19 cm，最终全部验收项为 `PASS`。见[完整循环精简结果](evidence/static_fr_ii/full_right_side_2k_summary.json)。

该 `PASS` 使用**独立的仿真车型副本**：原始 I_I 悬架运动学表只覆盖 ±70 mm，而本实验的扩展回弹达到约 150 mm；运行副本将表格超出原范围的部分设为边界值，并把随悬架压缩产生的轮心横向移动系数设为原值的 50%。原始车型文件和已有八路接口保持原样，完整循环副本额外接入第九路方向盘角输入用于横向闭环。未调整运动学的原模型上，RR 静止卸载会造成约 13 cm 横移，尚不满足直线要求。当前结论只针对固定质量、载荷、坑槽和这一仿真车型设置；4–7 km/h 及其他参数下的泛化仍待验证。

**新增边界和速度验证：**同一独立车型副本在宽 1.1 m、深 0.25 m 的坑槽上完成 FR→RR 全循环 `PASS`；原基线坑槽为宽 0.9 m、深 0.20 m。该边界工况的车辆指标基本不变，符合抬起轮未接触坑底的假设，见[坑槽边界摘要](evidence/static_fr_ii/full_right_side_pit_boundary_summary.json)。两轮都用 3 km/h 档时，FR 坑内 3.04–3.25 km/h、RR 3.05–3.20 km/h，全循环 `PASS`，见[双轮 3 km/h 摘要](evidence/static_fr_ii/full_right_side_3k_summary.json)。此速度下 RR 最小 ZMP 重心坐标 0.0553、坑上最低净空 15.4 mm，已接近 0.05／10 mm 门槛；4–7 km/h 尚未验证。

**较低附着工况：**物理路面摩擦系数从 0.7 降至 0.6 后，FR→RR 双轮 3 km/h 完整循环单次 `PASS`，见[μ=0.6 结果](evidence/static_fr_ii/full_right_side_3k_mu06_summary.json)。RR 最低坑上净空仅 10.14 mm，最大横向偏差 4.77 cm，几乎触及验收线。RR 在轮载已卸至近零后先进入姿态调整阶段，最终过坑净空门槛仍是 10 mm；不能把这次边界通过解释为充分的扰动鲁棒性。

**当前限制：**预载仍主要依据固定车型的成功指令，反馈修正幅度有限；尚未在坑槽和扰动的参数矩阵中验证泛化。按当前研究范围，载荷质量与位置保持固定。移动试验仅在隔离副本中调整了悬架回弹限位和轮胎低速参数，均非硬件额定值。三轮轮载无需均载。后续按[审核通过的开发方案](docs/FR_LIFT_CRAWL_DEVELOPMENT_PLAN_20260929.md)推进闭环预载、协同分配及鲁棒性。旧的 `expert_controller.py` 是早期移动越坑研究代码，不代表当前 FR 控制回路。

## 仓库结构

| 路径 | 内容 |
|---|---|
| `models/corner_module_ddev/` | 当前 I_I TruckSim 车型、接口合同与原始工况 |
| `src/ddevsim/` | 求解器接口、车型/轮胎几何、原有移动越坑控制与分析模块 |
| `src/ddevsim/static_wheel_lift/` | 支撑几何、辨识、分配器、状态机、约束配置与轨迹工具 |
| `scripts/` | 模型生成、原生仿真、辨识、FR 抬轮诊断和绘图入口 |
| `tests/` | Python 单元与接口合同测试 |
| `evidence/static_fr_ii/` | 可移植的隔离车型、压缩预载指令轨迹、结果摘要与图 |
| `docs/`、`IO_MAPPING.md`、`REFERENCE_NOTES.md` | 车型、接口、阶段实验与文献说明 |

`runs/` 只存本机生成的完整仿真输出，不纳入 Git。论文与参考源码仍在本地 `Research/`，不作为本仓库的运行依赖。旧 S_S 刚性桥车型已从当前工程移除。

## 运行与复核

需要 Windows、TruckSim 2019 求解器及其许可证；`simfile.sim` 中的 `PROGDIR`、`DATADIR` 和 `DLLFILE` 按本机安装位置核对。Python 依赖见 `requirements.txt`。在仓库根目录执行：

```powershell
python -m pip install -r requirements.txt
$env:PYTHONPATH = 'src'
python -m pytest -q
python scripts/run_static_fr_lift_probe.py --m1 evidence/static_fr_ii --preload evidence/static_fr_ii/preload_result.json --output runs/fr_static_ii_recheck --rl-support-n -4500 --fr-lift-n -100
python scripts/plot_static_fr_lift.py runs/fr_static_ii_recheck/result.json
python scripts/run_static_fr_closed_loop.py --crawl --output runs/fr_closed_loop_crawl_recheck
python scripts/run_static_fr_closed_loop.py --crawl --target-speed-kph 3.3 --min-pit-speed-kph 3.0 --output runs/fr_closed_loop_crawl_3k_recheck
python scripts/plot_closed_loop_run.py runs/fr_closed_loop_crawl_recheck
python scripts/run_right_side_full_cycle.py --output runs/right_side_full_cycle_recheck
python scripts/run_right_side_full_cycle.py --output runs/right_side_boundary_recheck --pit-width-m 1.1 --pit-depth-m 0.25
python scripts/run_right_side_full_cycle.py --output runs/right_side_3k_recheck --front-target-speed-kph 3.4 --front-min-pit-speed-kph 3.0 --front-brake-lead-m 0.5 --rear-target-speed-kph 3.35 --rear-min-pit-speed-kph 3.0
python scripts/run_right_side_full_cycle.py --output runs/right_side_3k_mu06_recheck --front-target-speed-kph 3.4 --front-min-pit-speed-kph 3.0 --front-brake-lead-m 0.5 --rear-target-speed-kph 3.35 --rear-min-pit-speed-kph 3.0 --road-friction 0.6
python scripts/open_run_visualizer.py right_side_both_3k_front34_rear335
```

最后一条命令按 `runs/` 中的轮次名称打开已有的 TruckSim 原生动画；也可传入运行目录或具体 `.vs` 文件。它只在 VS Visualizer 中播放，不重跑仿真或导出视频。包含中文字符的工程路径会自动复制到临时英文路径，车辆和道路资源仍从本机 TruckSim 2019 目录读取。

压缩预载轨迹只保存仿真时间及八个输入通道；复核命令在相同隔离车型上重放它。此命令用于**重复已验证的静态试验**，不等于从任意初始工况自动规划抬轮。新车型或新载荷必须先重新辨识和验证可行性。四角力/速率限目前是仿真软件设定，未标称为硬件额定值；详见 [I/O 映射](IO_MAPPING.md)和[控制对象说明](docs/control_object.md)。

要从原始 I_I 接触几何工况复核 M1/M2，可运行 `python scripts/run_static_fr_m1_m2.py --output runs/fr_static_ii_m1_recheck`；其输入模型保存在 `evidence/static_fr_ii/m1_source/`。M3 的局部增益记录在 `evidence/static_fr_ii/gain_matrix.json`，不能替代新工况下的重新辨识。
