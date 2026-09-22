# HD Utility DDEV 八执行器改造实施计划

> **执行方式：**在当前任务中按测试驱动方式逐项执行；每完成一项都运行对应测试，最终使用真实 TruckSim 2019 Solver API 验证。不得以静态文件检查代替求解器验证。

**目标：**在保持所选 `HD Utility Vehicle w/ Crate` 原始质量、惯量、几何、悬架和轮胎参数不变的前提下，将同一辆 TruckSim 车辆改造成四轮独立轮端转矩与四角主动悬架附加力均可直接控制的 8 输入 DDEV，并产出可复现模型、逐通道验证数据和接口说明。

**核心结构：**TruckSim 保留车身、刚性桥、被动悬架、轮胎和道路动力学；禁用原机械动力链，以 `IMP_MYUSM_*` 接收四轮轮端转矩；以 `IMP_FS_*` 将四角主动作用力并联叠加到原被动悬架。Python 构建模型并调用 Solver API 做配置探测和接口试验；Simulink 与 Python 共用同一 8 维端口顺序。

**技术栈：**Python 3、pytest、TruckSim 2019 Solver API/DLL、JSON、CSV、Markdown；本阶段不依赖 Simulink S-Function 完成真实性验证。

---

## Task 1：建立 HD DDEV 参数转换器的失败测试

**文件：**

- 新建：`tests/test_hd_ddev_case.py`
- 新建：`src/ddevsim/hd_ddev_case.py`

**步骤：**

1. 编写测试，给定最小 TruckSim 合并参数文本，要求转换器：
   - 将全部有效 `OPT_PT 3` 改为 `OPT_PT 0`；
   - 不改变 `M_SU`、`M_PL`、`M_US`、`L_AXLE`、`L_TRACK` 等物理参数行；
   - 在最终 `END` 前按固定顺序加入 8 个 `IMPORT` 和 16 个 `EXPORT`；
   - 重复构建得到字节一致结果；
   - 缺失或重复关键模式时明确失败。
2. 先运行：`python -m pytest tests/test_hd_ddev_case.py -q`，确认因功能未实现而失败。
3. 实现最小转换逻辑与参数解析。
4. 再运行同一测试，确认通过。

## Task 2：固化跨 Python/Simulink/TruckSim 的接口合同

**文件：**

- 修改：`src/ddevsim/channels.py`
- 修改：`interfaces/trucksim2019_channels.json`
- 新建：`interfaces/hd_utility_ddev_8ch.json`
- 修改：`tests/test_channels.py`

**步骤：**

1. 先写/更新测试，断言控制顺序唯一为：
   `T_FL,T_FR,T_RL,T_RR,F_FL,F_FR,F_RL,F_RR`。
2. 断言 TruckSim 导入量依次为：
   `IMP_MYUSM_L1,IMP_MYUSM_R1,IMP_MYUSM_L2,IMP_MYUSM_R2,IMP_FS_L1,IMP_FS_R1,IMP_FS_L2,IMP_FS_R2`。
3. 断言单位前四维为 N·m、后四维为 N，零输入退化为无驱动转矩与被动悬架。
4. 运行旧测试和新增测试，确认不存在旧 `IMP_MY_OUT_D*`、`IMP_FD_*` 误映射。

## Task 3：生成不覆盖原厂数据库的可复现模型工件

**文件：**

- 新建：`scripts/build_hd_utility_ddev.py`
- 生成：`models/hd_utility_ddev/run_all.par`
- 生成：`models/hd_utility_ddev/simfile.sim`
- 生成：`models/hd_utility_ddev/interface_contract.json`
- 生成：`models/hd_utility_ddev/source_manifest.json`
- 修改：`tests/test_hd_ddev_case.py`

**步骤：**

1. 测试 `simfile.sim` 必须声明 `PORTS_IMP 8`、`PORTS_EXP 16`、正确车辆结构 `S_S`、有效 DLL 路径与输出目录。
2. 实现构建脚本，默认源文件为 River Crossing 的缓存 `run_all.par`，允许命令行显式覆盖。
3. 在清单中记录源/目标绝对路径、SHA-256、构建时间、TruckSim DLL、解析出的原车参数、8 输入和 16 输出。
4. 构建前后逐项比较保护参数；任何质量、惯量、几何、弹簧、阻尼或轮胎数据变化都使构建失败。
5. 运行构建并检查生成文件，不写入 TruckSim 原厂 `_Data`。

## Task 4：用真实 TruckSim Solver API 做配置探测

**文件：**

- 修改：`src/ddevsim/solver_api.py`（仅在现有探测结果不足时）
- 新建：`scripts/probe_hd_utility_ddev.py`
- 生成：`models/hd_utility_ddev/probe_report.json`

**步骤：**

1. 先测试探测结果序列化与错误传播。
2. 对生成的 `simfile.sim` 调用 `vs_read_configuration`。
3. 验收必须同时满足：状态 READY、`n_import=8`、`n_export=16`、步长有效、错误日志为空。
4. 若出现变量名不被识别，停止并根据 TruckSim 官方量名修正，不静默替换。

## Task 5：建立逐通道正负脉冲验证器

**文件：**

- 新建：`src/ddevsim/interface_validation.py`
- 新建：`tests/test_interface_validation.py`
- 新建：`scripts/validate_hd_ddev_interfaces.py`
- 生成：`runs/hd_utility_ddev_interface_validation/*.csv`
- 生成：`runs/hd_utility_ddev_interface_validation/manifest.json`

**步骤：**

1. 先写测试，验证每个案例只有一个输入端口非零，正负幅值和时间窗正确，所有案例共享同一时间轴。
2. 实现 17 个试验：1 个零输入基线、8 个端口各 1 个正脉冲与 1 个负脉冲。
3. 转矩脉冲默认 ±500 N·m；主动悬架脉冲默认 ±1,000 N；持续 0.10 s。
4. 每个案例用相同 `run_all.par` 和同一个 8 输入/16 输出合同运行，保存输入、输出、求解状态和日志。
5. 不把非目标轮“完全无响应”设为独立性判据，因为刚性桥、车身与地面会产生物理耦合。

## Task 6：量化“可独立控制”而不是目测判断

**文件：**

- 修改：`src/ddevsim/interface_validation.py`
- 修改：`tests/test_interface_validation.py`
- 生成：`runs/hd_utility_ddev_interface_validation/verification_report.md`
- 生成：`runs/hd_utility_ddev_interface_validation/verification_summary.json`

**步骤：**

1. 对每个试验减去零输入基线。
2. 转矩通道使用对应 `AVy_*` 作为主响应，并记录四轮响应矩阵。
3. 主动悬架通道使用对应 `Fz_*`、`CmpS_*`、`Vz_Wc_*` 的组合响应，并记录四角响应矩阵。
4. 每个端口必须满足：命令只写入目标列、目标响应超过数值噪声、正负脉冲响应方向相反或符号相关性显著。
5. 报告同轴/对角耦合比和可能的符号约定；不以耦合存在判失败，只以串线、无响应、方向不一致或求解失败判失败。

## Task 7：输出车辆参数与控制使用说明

**文件：**

- 修改：`vehicle_selection_report.md`
- 新建：`reports/hd_utility_ddev_parameters_and_control.md`
- 修改：`README.md`

**步骤：**

1. 从最终 `run_all.par` 自动解析并报告原车型的实际质量、轴距、轮距、悬架形式、轮胎、仿真步长等，不手工改写数值。
2. 明确 8 输入的含义、单位、顺序、符号需如何通过脉冲报告确认。
3. 给出 Python Solver API 和 Simulink Bus/向量连接方法。
4. 说明 `IMP_FS_*` 是并联主动悬架力，零力时回到原被动悬架；说明刚性桥使机械响应耦合，但不影响端口独立寻址。
5. 明确本阶段验证的是执行器接口，不宣称已经完成电机额定能力、控制律或坑槽通过性能标定。

## Task 8：完整回归与交付检查

**文件：**

- 修改：必要的测试/文档文件

**步骤：**

1. 运行：`python -m pytest -q`。
2. 重新运行构建、Solver 配置探测与全部 17 个脉冲案例。
3. 搜索生成模型与接口合同，确认不再包含作为最终接口的 `IMP_MY_OUT_D*`、`IMP_FD_*`，且保护参数未变化。
4. 检查全部报告中的绝对路径、摘要、时间戳和结果相互一致。
5. 仅在真实 Solver API 结果全部通过后，向用户报告“8 个执行器独立可控”；若某项失败，准确报告失败端口、日志和下一步，不作推断性成功声明。

