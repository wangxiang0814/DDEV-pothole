# 两种主动悬架专家策略架构

2026-11-12：普通单轮和批量采集统一经 `scripts/run_strategy.py` 启动。批量入口 `scripts/run_expert_batch.py` 支持两种专家混合计划，每轮仍由独立进程执行，汇总写入 `runs/_batches/<batch_id>/`。用法和计划格式见[项目首页](../README.md)。

两套专家的新场景共用车辆原点、10–15 m 坑沿距离及固定干燥路面。抬轮在 `strategies/wheel_lift/approach.py` 管理驶近和停车，停车后从当前仿真时间开始静稳及卸载参考。现有支持增益先按原标定车型和工况校验，再生成固定原点的运行场景。两份模型的哈希及实际附着系数分别记录。

保留代表性原生回放、引用中的源轮次、车型和验收证据。退役调参的原始运行已清理，状态和来源索引位于本机 `runs/_cleanup_20261112/result_index.jsonl.gz`。此前各研究记录中的运行保留说明描述当时状态。

2026-10-27：专家二新增显式`event_crawl`策略分支，按逐轮入坑/爬沿风险规划可变低速；旧恒速研究控制器继续保留作回归。`event_phases.py`、`event_speed.py`、`event_controller.py`、`event_assessment.py`分别负责阶段、速度规划、协调控制和独立验收，不依赖专家一。当前仍处于参数研究，不能把可运行代码视为10cm深坑能力已完成。

本工程维护两套独立专家：`wheel_lift`（抬轮过深坑）和 `shallow_press`（预瞄前馈与反馈下压过浅坑）。两者保留同一调参 I_I 基础车身；浅坑已有研究控制器、专属 worker 和验收。用户2026-10-24批准`expert2_mf52_research_20261024`为专家二轮胎研究基线，原内部PAC52保留对照，专家一车型不变。新旧模型必须记录不同物理身份；采纳研究基线不等于完整安全验收。

## 模块职责

| 位置 | 职责 |
|---|---|
| `src/ddevsim/strategies/wheel_lift/` | 前轮、完整循环控制，旧 CLI，模型准备兼容层及策略验收 |
| `src/ddevsim/static_wheel_lift/` | 保留原抬轮专属辨识、分配、反馈、状态与参数实现 |
| `src/ddevsim/strategies/shallow_press/` | 空间预瞄、悬架前馈/反馈、轮载分配、逐轮防滑、独立车型对照与研究筛查 |
| `src/ddevsim/runtime/` | 版本化 SI 合同、策略选择、输出认领、运行清单和进程管理 |
| `src/ddevsim/vehicle_profiles/` | 两套策略共用的调参 I_I 车辆版本与中性参数生成规则 |
| `src/ddevsim/scenarios/` | 坑槽覆盖/初始化校验、物理地形与显示资源同步生成 |
| `src/ddevsim/native_video.py` | 播放和导出共用的运行定位、原生历史读取、显示暂存 |
| `src/ddevsim/vehicle_appearance.py`、`display_chassis.py` | 欧式平头货厢车外观和随原生轮心运动的机械展示部件 |
| `src/ddevsim/chassis_contact.py` | 中性底盘参数化净空、物理路面及可选诊断接触模型；专家不默认启用 |
| `scripts/` | 用户命令及历史诊断入口；三个主要旧运行入口为兼容封装 |
| `configs/wheel_lift/`, `configs/shallow_press/` | 各自参数清单，禁止相互覆盖 |
| `scenarios/shallow_press/` | 右侧轮迹/左右轮迹模板，具体坑槽尺寸必须显式提供 |

依赖方向：入口 → 运行管理/所选策略 → 公共合同与基础设施。两个策略互不导入，共享层不依赖专属状态机/增益。`tests/architecture/test_strategy_dependencies.py` 检查这一边界。

## 车辆与场景身份

`tuned_ii_20261005` 以 `evidence/static_fr_ii/model` 的隔离车型为源，明确使用原低速轮胎修订（原生0.2 km/h，历史变量名误标为m/s）、扩展回弹止挡、运动学端点保持、横向轮心移动系数 0 和第九路方向盘输入。载荷位置及质量来自同一源文件；不为浅坑另调车辆。

车辆 recipe 哈希包括源文件 SHA256 和生成规则。实际车辆文本身份排除 Procedures 路面/初始化部分、外层 SSTART/TSTOP 和显示相机字段；每次运行还记录完整模型/场景 SHA256。**车辆身份不替代旧辨识的完整模型哈希校验，也不授予增益跨场景复用资格。**

轮距 1.260 m、轴距 1.925 m、轮胎半径 0.263 m与当前车型一致。覆盖校验的轮胎半宽 0.10 m是显式的软件几何包络，不是经测量的实体轮胎规格，不改变 TruckSim 参数。后续接入实测接触包络时调整校验规则与版本。

单侧坑覆盖右侧轮迹，FR 先过、RR 后过。宽坑覆盖左右轮迹，前轴 FL/FR 先过，后轴 RL/RR 后过。`length_m` 始终是纵向长度，`width_m` 始终是横向宽度。

## 控制合同

公共合同版本为 1，固定 FL/FR/RL/RR 顺序，所有物理量使用 SI。

```python
strategy.reset(StrategyContext(...))
command = strategy.step(Observation(time_s, observed_at_s, valid, values_si))
state = strategy.status()
report = strategy.summary()
```

`ControlCommand` 返回四轮 N·m 转矩、四角 N 悬架附加力和可选方向盘角 rad。它不是前轮转角。原八路输入模型与九路完整循环分别声明能力，不能错配。公共观测沿用原生变量名，但使用合同中已验证的 SI 单位；例如 Vx 已为 m/s，Jnc 已为 m，Yaw 已为 rad。

`WheelLiftStrategy` 在边界换算到旧控制器的原生单位，旧算法内部仍按 km/h、mm、deg 运算。不同实例分别保存配置、滤波器、状态机与时钟，原 CLI 使用局部配置变量。无效数据由原完整循环的 stale-feedback 分支安全保持；有效但缺少所需通道的数据明确拒绝。

`ShallowPressStrategy.step()` 实现20 ms SI外环，输出四轮转矩、四角力和方向盘角设定。专家专属 `CrawlActuation` 实现显式2 ms轮速差反馈、失载/重接地协调与逐积分步连续幅值/变化率约束；`NativePreviewBridge` 负责时钟、观测时延和原生单位边界。积分细化不改变控制采样周期，车身与电机反馈时延分别声明。公共接口没有改变，专家一不经过这些模块。

浅坑研究结果 `SIMULATION_PASS` 与完整安全 `PASS` 分开：真实底盘几何、执行器实体行程/带宽/功率仍未知，因此 `acceptance_pass=false`。场景准备不运行控制，清单为 PREPARED / UNASSESSED。独立轮胎版本的父车型、实际变更与采纳范围同时进入运行清单、结果和示范元数据；旧对照保留`adopted=false`，用户批准后的研究版本为专家二研究采纳。最终模型/端口哈希在生成修改后刷新，冻结输入另存 `input_provenance.json`，避免结果与清单哈希循环依赖。

MPC专属旧代码、脚本、测试与研究证据已退出活动目录，保留于[校验归档](../archive/mpc_20261013/README.md)和备份tag。当前复现和代表性回放所需的原生历史保留在本机。

## 运行和显示隔离

```text
runs/<strategy>/<case>/<run_id>/
  run_claim.json            独占目录身份
  run_manifest.json         配置、来源、代码身份、生成哈希与实际状态
  control_config.json       新抬轮入口冻结的有效选项
  run.log                   独立 worker 输出
  model/
    run_all.par, simfile.sim, scenario.json
    visual_assets/          本次物理地形对应的 OBJ/MTL
    output/                 原生 .vs/.vsb/*_all.par
  result.json               策略任务验收，准备阶段不存在
```

新运行目录通过原子 mkdir 认领，重复编号拒绝覆盖。`--claimed-output` 是新启动器向旧 worker 传递的内部选项，只接受尚未生成产物的认领目录；旧 CLI 的 `--output` 默认仍要求目录不存在。

每次原生求解都使用独立子进程，因为 TruckSim DLL 与当前工作目录含进程全局状态。进程退出零、求解到终止时刻或异常后恢复都不能单独构成任务 PASS；必须有非空且全部通过的策略验收。进程启动失败、原生异常和缺失结果保留在运行清单中。

原生播放/视频输入接受完整运行目录或历史文件。显示暂存复制该运行的 OBJ/MTL，把显示副本中的坑槽网格引用改到独立暂存资源；原 `.vs/.vsb/*_all.par` 保持字节不变。ASCII 视频暂存路径含源目录哈希，避免同名工况碰撞。旧历史缺少本地网格时保留其原有全局资源行为。

原生 AVI 最后一步仍通过 VS Visualizer GUI 导出。现有工程回放视频不是原生场景；准备新浅坑场景后需要真实历史才能播放该场景的控制过程。

## 兼容性与开发协作

旧三个命令 `run_static_fr_closed_loop.py`、`run_right_side_full_cycle.py`、`run_verified_right_side.py` 保留。旧默认 verified 清单与最新 recommended 清单的差异也保留，新启动器默认使用最新推荐清单。原历史证据、模型、报告、配置清单不迁移。

算法目录隔离不意味着任意同时编辑同一文件都不会发生 Git 冲突。并行开发时分别使用 feature 分支/工作区，在各策略目录调参；公共接口修改须协调、保持版本兼容并回归双方合同。

基础 profile 不支持仅改元数据的轮距、轴距、轮胎半径或积分步长覆盖。专家二研究 worker 可在本轮隔离副本显式改变积分步长或轮胎模型，并保留父车型和变更身份；这些对照不属于专家一已验证车型。历史 `tyre_low_speed_slip_m_s` 名称错误：数值实际直接写入原生 km/h；保留原数值与身份，新的SI比较显式换算。

## 专家二多速度控制与能力诊断

`body_control.py`按实际CG和轮位将升沉、侧倾、俯仰目标力/矩分配四角；`body_leveling_s`先静止平滑调平，再进入蠕行。悬架预瞄与参考带宽按请求速度在reset时选择，转矩爬沿预瞄独立，过坑过程中不随瞬时车速跳变。`actuation.py`实现接地滞回、名义驱动恢复坡道和独立防滑制动；滤波轮速差微分仅在显式启用时使用，失效重置历史。

默认能力仍与专家一对齐。仅显式`--diagnostic-torque-rate-nm-s`或同名配置字段启动用户批准的更快转矩扫描；`capability`记录原1000 Nm/s和实际值，随不可变输入、结果、清单和episode携带。原基线criteria不改写，诊断另列diagnostic_criteria；状态CAPABILITY_DIAGNOSTIC_ONLY、simulation_pass=false、acceptance_pass=false，不能混入普通筛查通过点。

2026-10-25接触协调核查：活动轮速环使用实测横摆角速度计算局部滚动速度，接地切换重置微分记忆。`contact_kinematics.py`只提供轮心/径向切平面近似诊断，未作为活动反馈量；失败的快速groundhook、被动力补偿与转矩率控制退出活动配置并独立归档。`identification.py`和`probe_shallow_extension.py`只做连续限速的静态模型可达核查，不输出实体执行器额定值或策略PASS。

外置轮胎非3114工作模式仅为显式、未采纳模型诊断，必须清除研究采纳身份，并固定MODEL_DIAGNOSTIC_ONLY及两个pass=false；现有3114研究基线不变。MF-Swift授权初始化失败与已完成的控制失败分别记录。派生矩阵汇总重新绑定冻结计划、原始结果状态、失败项与完整工况域，不能直接相信可编辑表中的PASS。见[深坑验证与限制](SHALLOW_DEEP_COORDINATION_20261025.md)。

## 专家二逐轮分阶段通行

`policy=event_crawl`由专属worker选择`EventCrawlStrategy`，公共九路SI命令保持不变。所需过坑轮由明确场景身份固定为FR/RR或四轮，不能因观测轮迹偏离而减少任务。已知几何、轮载和接触压缩共同识别阶段，轮载变化可以提前触发ENTRY，但不能据此宣称真实穿过坑沿。

速度规划对前后轴各入/出坑沿建立制动距离包络，并限制参考加速度及其变化率。接近速度与坑沿参考分开；后轴全部驶出后锁定最终停车，不因小幅回退重新起步。相位悬架使用预瞄伸展前馈、姿态反馈、重接地减力和轮载修正；快速轮载反馈与失载轮速同步均显式配置、受原能力限制。没有加入学习算法。

验收任务`event_crawl_v1`保留历史物理门槛和原恒速结论，替换恒定均速及固定速度停顿要求。最新`raw_events_reference_v3`核查指定车轮的原始前向过沿顺序及横向覆盖、事件速度包络、有效参考、相对可变参考的超速、有限推进、停车和诊断文件哈希。允许低于计划速度继续推进；不把低均速或零参考当作规划成功。`event_checks`只计算新任务证据；完整worker仍执行所有原物理检查。只读复核可在物理验收源码哈希完全一致时复用已校验的旧物理结果，避免重复底盘几何遍历；旧结果不改写。

`road_callback.py`仅用于显式`--road-callback`独立研究。原路面透传与圆轮支撑平面通过官方外胎接口在实际查询坐标处返回高度/梯度，实例内保存回调及DLL句柄至求解结束；记录查询CSV和摘要哈希。任何该类运行都清除模型采纳身份，固定为MODEL_DIAGNOSTIC_ONLY及两个pass=false；与更快转矩扫描组合从配置合并后即拒绝。几何平面一致不代表轮胎实体动力学已标定，不替换MF52/3114采纳基线。

2026-10-27用户检查后要求收束并暂停调优。保留`event_crawl_development_20261027.json`及冻结工况、已验证历史；未接入的爬沿协调草案和失败试验均退出活动接口并归档。当前有效范围和未达标深坑见[阶段记录](SHALLOW_EVENT_CRAWL_20261027.md)。
