# Independent Expert Strategies Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** 在当前工程内隔离抬轮与浅坑下压策略开发，保留抬轮运行兼容性，并让两类浅坑场景使用共用 TruckSim 显示链路。

**Architecture:** 共享原生求解、车辆版本、场景和显示基础设施；策略分别拥有配置、状态、资产与验收。将抬轮运行脚本中的业务逻辑迁入策略包，通过不可变配置和适配器接入公共接口。

**Tech Stack:** Windows PowerShell、Python、NumPy/SciPy、pytest、TruckSim 2019 DLL / VS Visualizer；不增加产品依赖。

**Spec:** `docs/superpowers/specs/2026-10-08-independent-expert-strategies-design.md`（用户于 2026-10-08 确认）。

## Global Constraints

- 两种策略使用完全相同的控制对象；固定车辆版本，场景变化不得隐式调车。
- 单侧坑 FR、RR 依次通过；宽坑 FL/FR 先通过，RL/RR 后通过。
- 轮序固定 FL/FR/RL/RR；原八路接口和完整循环九路接口分别显式声明。
- 所有可变控制状态归属于实例；不得通过修改模块全局 CFG 调参。
- 新策略不得互相导入或导入 scripts；共享层不得导入策略专属实现。
- 源资产与历史证据保留；新运行独占模型和输出目录，拒绝覆盖。
- 原生仿真使用独立进程；同一进程不得并发调用原生求解器。
- 不重新调整抬轮参数，不放宽验收；旧 CLI 默认清单保持原语义。
- 浅坑控制律尚未实现时明确拒绝控制仿真，不输出任务 PASS。
- 物理地形与显示资源使用同一场景合同；驾驶员默认隐藏、ASCII 暂存功能保留。

## Review Focus

- 中文及含空格路径：脚本、子进程与播放暂存正确定位资源（任务 4、6）。
- 配置/观测中的 NaN、无穷及缺失通道：在执行原生控制前拒绝或明确标记无效（任务 2）。
- 路径逃逸、同名输出及错误输出指向源模型：拒绝认领与覆盖（任务 4）。
- 宽坑标签与实际轮迹覆盖不一致，或开始时车轮已经入坑：准备时拒绝（任务 5）。
- 异常终止/许可证不可用：保留失败记录，求解完成不冒充任务成功（任务 4、7）。

## File Map

保留已有底层共享模块与 `static_wheel_lift/`，不为目录外观全面移动。以下新增包均含 `__init__.py`。

| 文件 | 职责 |
|---|---|
| `runtime/contracts.py` | 冻结上下文、观测、命令、状态和策略协议 |
| `runtime/registry.py` | 延迟加载指定策略，拒绝未知/未实现策略 |
| `runtime/run_workspace.py` | 输出认领、清单、资源身份与进程运行 |
| `strategies/wheel_lift/front.py` | 从前轮脚本迁出的实例控制器 |
| `strategies/wheel_lift/acceptance.py` | 从两个脚本迁出的验收函数 |
| `strategies/wheel_lift/model_builder.py` | 抬轮历史模型准备兼容行为 |
| `strategies/wheel_lift/cycle.py` | 完整循环控制器 |
| `strategies/wheel_lift/front_cli.py`, `cycle_cli.py`, `verified_cli.py` | 迁出的原 CLI 解析与执行 |
| `strategies/wheel_lift/adapter.py` | 公共 SI 合同与原抬轮控制接口桥接 |
| `strategies/shallow_press/controller.py`, `config.py` | 未实现控制器的明确边界与独立开发配置 |
| `vehicle_profiles/tuned_ii.py` | 当前调参 I_I 版本的显式声明与生成 |
| `scenarios/pit.py`, `builder.py` | 两类浅坑合同、覆盖核查与共同场景构建 |

以上路径相对 `src/ddevsim/`。新增入口 `scripts/run_strategy.py`、`scripts/prepare_strategy_case.py`；原三个运行脚本变为兼容封装。新增 `configs/wheel_lift/recommended.json`、`configs/shallow_press/development.json`，`scenarios/shallow_press/right_track.json`、`wide.json`。所有新测试位于 `tests/architecture/`、`tests/shallow_press/`。

## Task 1: 建立重构前基线与行为比较证据

**Files:** Create `tests/architecture/test_lift_reference_equivalence.py`; use original three CLI scripts and portable evidence without modification.

**Interfaces:** Produces a deterministic observation fixture and `capture_controller_trace(factory, observations) -> list[dict]` local test helper. Baseline artifact is test data generated from approved HEAD `5edb0dd`, not a guessed PASS result.

- [x] Step 1: 读取项目及父目录的 AGENTS.md（若存在），读取 using-git-worktrees 技能；在实施时建立或复用隔离工作区，保留本项目相对资源布局。
- [x] Step 2: 在项目根目录执行 `$env:PYTHONPATH='src'; python -m pytest -q`，记录实际数量、失败和环境；存在失败先定位，不以文档旧数量为基线。
- [x] Step 3: 用独立 Python 进程保存默认与 efficient 两套前轮配置的确定性观测/输出/状态轨迹；包含初始化、反馈修正、阶段交接和无效观测，复用已有测试的有效几何样本。
- [x] Step 4: 保存三个原 CLI 的 `--help`、推荐启动器 `--dry-run` argv，以及相同配置生成的模型文本与场景；存放测试临时目录或本地 runs，不污染证据。
- [x] Step 5: 将行为比较用的小型确定性 fixture 加入测试；与原实现一致通过，原生完整输出不加入 Git。提交 `test: capture lift refactor compatibility baseline`。

## Task 2: 定义共享合同与策略依赖边界

**Files:** Create `runtime/contracts.py`, `runtime/registry.py`, `strategies/wheel_lift/adapter.py`, `strategies/shallow_press/controller.py`, `config.py`; tests `tests/architecture/test_strategy_contract.py`, `test_strategy_dependencies.py`.

**Interfaces:** Frozen `StrategyContext(strategy_id, vehicle_id, scenario_id, control_period_s, config, assets)`; `Observation(time_s, observed_at_s, valid, values_si)`; `ControlCommand(wheel_torque_nm, suspension_force_n, steering_wheel_angle_rad=None)`; `StrategyStatus(phase, completed, abort_reason)`; `Strategy.reset(context)`, `step(observation) -> ControlCommand`, `status() -> StrategyStatus`, `summary() -> dict`. `load_strategy(strategy_id: str) -> type` lazy loads a supported strategy.

- [x] Step 1: 写失败测试：`test_command_requires_four_finite_values` 验证每组恰好四个值并拒绝 NaN/inf；`test_context_rejects_nonpositive_period`；`test_missing_observations_are_not_zero_filled`；`test_unknown_strategy_rejected`；`test_shallow_control_explicitly_unimplemented`。
- [x] Step 2: 执行 `python -m pytest tests/architecture/test_strategy_contract.py -q`，确认失败来自未实现接口。
- [x] Step 3: 实现冻结合同，映射复制并只读，声明 SI 单位；浅坑 `step` 抛 `NotImplementedError`。注册表使用显式模块路径，不通过遍历目录自动加载算法。
- [x] Step 4: 写 AST 依赖测试并验证：wheel_lift 与 shallow_press 不互相导入，runtime/scenarios/vehicle_profiles 不依赖专属策略；registry 只允许显式字符串延迟加载。禁止产品代码导入 scripts。
- [x] Step 5: 执行上述两个测试文件，确认通过；提交 `feat: define isolated expert strategy contracts`。

## Task 3: 迁移抬轮逻辑并消除全局配置修改

**Files:** Create wheel_lift `front.py`, `cycle.py`, `acceptance.py`, `model_builder.py`, `front_cli.py`, `cycle_cli.py`, `verified_cli.py`; modify `adapter.py`, three original run scripts. Tests `tests/architecture/test_lift_instance_isolation.py`, `test_legacy_entrypoints.py`, Task 1 equivalence test.

**Interfaces:** `LiftCrawlController(*, crawl: bool, scenario: dict, contact_gains=None, config=CLOSED_LOOP_RUN, evidence_dir: Path | None=None)`; `prepare_model(output: Path, *, crawl: bool, road_friction=None, config=CLOSED_LOOP_RUN) -> tuple[Path, dict]`; `evaluate_control_rows(rows: list[dict], *, crawl: bool, scenario: dict, config=CLOSED_LOOP_RUN, **existing_options) -> dict`; complete cycle takes explicit `front_config`, existing rear config and asset paths. CLI `main(argv: list[str] | None=None) -> None`.

- [x] Step 1: 写 `test_two_front_instances_do_not_share_config_or_state`：先创建 efficient 实例，再默认实例，第一实例 step 后第二实例 config/滤波器/计时器不变；模型准备与验收接收同一显式 config。
- [x] Step 2: 执行该测试，确认暴露旧全局配置耦合或新接口缺失。
- [x] Step 3: 原样迁移控制和验收计算，类内 CFG 使用 `self.config`，函数内使用参数 `config`。完整循环 CLI 使用局部 `front_config`，传给模型、控制器、噪声处理、分配和验收。保留低层 static_wheel_lift 算法。
- [x] Step 4: 三个旧脚本仅做路径初始化、显式兼容导出和 `main()`。兼容默认 CFG 可只读导出；不允许 main 或 full cycle 改写它。核查其他脚本/测试的所有旧导入并保留签名兼容。
- [x] Step 5: 实现公共适配器，只在边界换算单位；原控制器仍接收原 EXPORTS 顺序和原单位。命令数组保持原通道顺序，转向仅在九路能力下输出。
- [x] Step 6: 执行 `python -m pytest tests/architecture tests/test_static_fr_runner_telemetry.py tests/test_verified_cycle_launcher.py tests/test_recommended_parameter_cases.py tests/test_rear_cycle.py -q`；比较 Task 1 轨迹和模型文本，原数据满足容差/文本完全相同。提交 `refactor: isolate lift controllers and preserve legacy CLI`。

## Task 4: 冻结同一车辆版本并隔离运行文件与进程

**Files:** Create `vehicle_profiles/tuned_ii.py`, `runtime/run_workspace.py`, `scripts/run_strategy.py`; tests `tests/architecture/test_vehicle_profile_identity.py`, `test_run_workspace.py`; create `configs/wheel_lift/recommended.json`.

**Interfaces:** `TunedIIProfile` declares physical recipe, capability and timing; `build_vehicle(target: Path, profile: TunedIIProfile) -> dict`; `claim_run(root: Path, strategy_id: str, case_id: str, run_id: str) -> Path`; `write_run_manifest(run_dir: Path, data: Mapping) -> Path`; `run_worker(argv: list[str], *, run_dir: Path, cwd: Path) -> int`.

- [x] Step 1: 写失败测试：`test_run_claim_is_exclusive`、`test_run_identifiers_reject_traversal`、`test_source_asset_cannot_be_output`、`test_strategy_outputs_are_distinct`、`test_unicode_worker_argv_is_preserved`、`test_worker_failure_preserves_manifest`。
- [x] Step 2: 执行 `python -m pytest tests/architecture/test_run_workspace.py -q`，确认失败。
- [x] Step 3: 原子创建运行目录，限制标识为单个安全路径段；实际输出绝对路径必须位于 runs 且不覆盖源资产。manifest 记录代码版本/dirty、实际输入 SHA256、能力、时序、随机种子与结束状态，先写临时文件再 replace。
- [x] Step 4: 冻结已推荐 efficient/lateral-ride-scale=0 所用车辆生成规则；参考八路源副本，显式应用原低速轮胎、回弹/运动学、载荷和九路能力修订。从原完整循环生成的模型中提取比较车辆参数，不改变原完整文件增益哈希校验。
- [x] Step 5: 写 `test_lift_and_shallow_use_same_vehicle_recipe`、`test_pit_change_preserves_vehicle_parameters`、`test_profile_hash_tracks_vehicle_changes`、`test_new_recommended_manifest_matches_legacy_options`；生成的车辆应与同配置旧入口一致。
- [x] Step 6: 新入口只启动子进程运行指定算法，argv 使用列表和显式 cwd；记录失败，不将 worker 返回零等同 PASS。旧 --output 语义不改；新清单使用项目根相对资产路径。
- [x] Step 7: 执行本任务测试与旧 launcher 测试，确认通过；提交 `feat: isolate native runs and freeze shared vehicle profile`。

## Task 5: 建立单侧与宽浅坑场景准备通路

**Files:** Create `scenarios/pit.py`, `scenarios/builder.py`, `scripts/prepare_strategy_case.py`, shallow configs/scenarios; tests `tests/shallow_press/test_scenario_geometry.py`, `test_prepare_case.py`.

**Interfaces:** `PitCase(strategy_id, case_id, vehicle_profile_id, coverage, geometry, initial_state, display)`; `validate_case(case: PitCase, profile: TunedIIProfile) -> None`; `prepare_case(case: PitCase, run_dir: Path, profile: TunedIIProfile) -> dict`. Coverage enum `right_track` / `both_tracks`, expected wheel sequence `(FR, RR)` / `((FL, FR), (RL, RR))`.

- [x] Step 1: 写失败测试：`test_right_track_excludes_left_footprints`、`test_wide_covers_both_tracks`、`test_bad_coverage_label_rejected`、`test_all_wheels_start_before_entry`、`test_geometry_rejects_nan_and_overlapping_edges`。
- [x] Step 2: 执行 `python -m pytest tests/shallow_press/test_scenario_geometry.py -q`，确认失败。
- [x] Step 3: 实现覆盖校验，使用同一 profile 的轮距/胎宽/轴距与起点；要求坑有效平底范围覆盖指定轮胎包络，单侧不得触及左轮包络，2*过渡小于坑长/宽，深度/尺寸有限且正。
- [x] Step 4: 准备入口显式接收 --depth-m、--length-m、--width-m 等参数。提交场景模板只声明覆盖/显示/起点规则；未给尺寸时要求命令参数，避免填充冒充用户推荐值。
- [x] Step 5: 场景准备复制 task 4 同车辆版本，只替换路面和允许的初始化/显示字段；复用 `_procedure_block` 与 mesh 生成而不继承深坑专属宽度门槛；移出可共享坑形替换函数时保持旧调用兼容。
- [x] Step 6: 写 `test_preparation_does_not_mutate_sources`、`test_physical_and_visual_depth_agree`、`test_prepared_case_starts_at_rest`、`test_shallow_run_rejects_before_solver_launch`；两类场景参数改变后车辆 recipe 身份相同。
- [x] Step 7: 执行 `python -m pytest tests/shallow_press tests/test_pothole_case.py tests/test_pit_scene_gain_transfer.py -q`，确认通过；提交 `feat: prepare independent shallow pit scenarios`。

## Task 6: 共用历史播放与视频资源定位

**Files:** Modify `scripts/open_run_visualizer.py`, `scripts/export_native_video.py` only if nested output resolution requires changes; tests `tests/architecture/test_strategy_visualization.py`.

**Interfaces:** Retain `resolve_source(value: str) -> Path` and existing video API; both accept completed new run directory, locating `model/output`, preserving native history and display-copy distinction.

- [x] Step 1: 用现有 tests/test_visualizer_driver_visibility.py 的 history fixture 写失败测试：`test_nested_strategy_runs_resolve_history`、`test_equal_case_names_have_distinct_ascii_stage`、`test_nested_run_keeps_driver_hidden_and_source_untouched`、`test_mesh_resources_belong_to_selected_run`。
- [x] Step 2: 执行 `python -m pytest tests/architecture/test_strategy_visualization.py -q`；已有行为覆盖则记录通过，不做无必要改动。
- [x] Step 3: 最小调整历史定位与 staging，保持 --show-driver 和原入口行为；每次显示只引用指定 run 的地图/mesh/历史，禁止固定路径覆盖另一个 run。
- [x] Step 4: 执行 `python -m pytest tests/architecture/test_strategy_visualization.py tests/test_visualizer_driver_visibility.py tests/test_batch_and_video.py tests/test_replay_video.py -q`，确认通过；提交必要改动 `test: verify shared playback across isolated strategy runs`。

## Task 7: 全量回归、原生复核与开发文档

**Files:** Modify `README.md`, `IO_MAPPING.md`, approved spec status; create `docs/ARCHITECTURE.md`, `docs/STRATEGY_DEVELOPMENT.md`, `docs/STRATEGY_REFACTOR_VALIDATION_20261008.md`, project `AGENTS.md` scoped to strategy isolation.

**Interfaces:** New documented entrypoints `run_strategy.py --strategy ... --case ...` and `prepare_strategy_case.py ...`; original three CLI commands retain arguments/defaults. Acceptance report clearly separates Python verification, native baseline comparison, shallow preparation/display and unimplemented control.

- [x] Step 1: 执行 `$env:PYTHONPATH='src'; python -m pytest -q` 和 `git diff --check`，确认实际输出；修复回归后只重跑受影响检查及最终全量。
- [x] Step 2: 选择推荐 seed_repeat 固定工况，先检查需要的模型/增益源与许可证；在原基线独立 checkout 和重构版本分别运行相同 argv/种子到独立 runs。比较全部 criteria、关键轨迹/状态、模型文本和时序；只要有差异就解释与修复，不仅核对 PASS。
- [x] Step 3: 使用显式开发参数生成单侧/宽浅坑，记录这些为示例非推荐工况。许可证允许时用明确标注的基础设施诊断 worker 生成静止历史，只验证资源/显示；它不属于 shallow_press 控制运行，不产生任务成功结论。
- [x] Step 4: 执行原生历史 playback --dry-run，再打开两类场景验证轮迹、坑深/宽和资源；保留当前 AVI GUI 最后导出流程。无法执行原生检查时在报告中精确写明原因和未验证项。
- [x] Step 5: 更新开发说明：分别编辑两个策略目录，不互借配置/增益/验收；公共接口变更回归双方；不得覆盖源资产；实例配置与进程隔离；许可证并发限制；浅坑尺寸和阈值留在后续策略开发确定。
- [x] Step 6: 更新 README 顶部当前架构/命令，保留历史内容和证据路径；将 spec 状态改为已实施仅在任务实际完成后。执行 git diff --check 并核对文档命令的 --help/dry-run。
- [x] Step 7: 使用 verification-before-completion 与 requesting-code-review 技能，检查最终变更；提交 `docs: document isolated strategy development and validation`，交付文件、验证结果及真实限制。

## Self-Review / Execution Handoff

任务 2/3 覆盖策略接口与实例隔离；任务 4 覆盖车辆、运行身份和进程；任务 5 覆盖用户更正的两种浅坑；任务 6 覆盖显示/视频；任务 7 覆盖兼容回归和开发规则。全局约束不因某测试尚未编写而豁免。

用户已确认并授权在本会话当前工程直接执行。七项任务已实施；485 项 Python 测试通过，固定推荐点真实原生等价核对通过，两类浅坑静态诊断/原生历史/播放与视频暂存通过。Visualize 两窗口已启动，工具未做图像逐帧人工检查，AVI 尚未导出。最终独立审查三项问题均已回归修复。执行中的具体测试命名调整和范围解释见 ../../STRATEGY_REFACTOR_VALIDATION_20261008.md；本清单表示任务实施完成，不表示未来浅坑控制律已实现。
