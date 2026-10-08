# 双专家策略架构重构验证记录

日期：2026-10-08。本次完成架构隔离、抬轮迁移、浅坑场景与开发框架、共享原生回放通路。浅坑下压控制律尚未实现。

## Python 与代码审查

使用项目原环境 Python 3.8.5、NumPy 1.24.4、SciPy 1.10.1，执行 `PYTHONPATH=src python -m pytest -q`：**485 passed in 25.24s**。`git diff --check` 通过。测试覆盖旧策略黄金轨迹、SI 合同、实例配置隔离、模块依赖、运行目录独占/失败记录、车辆一致性、两类浅坑覆盖和回放资源隔离。

独立只读审查发现三项问题，均先增加失败回归再修复：原生 CRLF 历史未重绑本地网格；后轮实例配置覆盖未同步耦合增益；固定车型允许声明未应用的轮距/轴距/半径/积分步长。现在兼容 LF/CRLF、按实例后轮参数计算增益，并拒绝固定车型不支持的这些覆盖。原有默认控制行为不变。

## 抬轮真实原生对比

基线源来自 Git 备份 `647d168` 的独立 archive；运行相同推荐 `seed_repeat` 工况和固定种子。TruckSim Browser 启动后，两次 DLL 仿真都成功，原专属验收都为 PASS。

- 基线：`runs/wheel_lift/seed_repeat/baseline_native_20261008_browser/`
- 重构：`runs/wheel_lift/seed_repeat/refactor_native_20261008_browser/`

| 核对对象 | 结果 |
|---|---|
| run_all.par、simfile.sim、scenario.json | 逐字节一致 |
| native_5ms.csv 全部原生信号 | 逐字节一致 |
| 前轮控制/观测 2060 行，后轮控制/观测 1850 行 | 除 support_qp_solve_ms 外全部字段精确一致 |
| 状态、验收 criteria、物理指标、控制配置 | 精确一致 |
| result.json 差异 | 仅 native/csv 输出路径及 support_allocation 三个实际耗时统计 |

原生轨迹 SHA256：`a76a859eb4fee23f8e9fd742e253a9a8a38493e2ddc41c2b938d0971161aa446`。模型 SHA256：`1e51e0a92f0c16dcbd2cd8f52658e8f24bc2fcb465ae7067c6ccc2bd3334bacb`。

之前 Browser/许可证尚未可用的尝试已保留失败日志。新启动器将该次运行记录为 FAILED，未伪造 PASS。原生等价验证只覆盖本次固定推荐点；未宣称所有工况重新通过。

## 浅坑场景与原生显示

下面参数是基础设施验证示例，不是推荐控制工况：深度 0.05 m、纵向长度 0.8 m；单侧横向宽度 0.9 m、宽坑 1.8 m。两者使用相同车辆物理身份 `a059431c563efcb2955863f1e5c41e1082c559ce178a039ff4952280ce1ab6cc`。

- `runs/shallow_press/right_track/architecture_example_20261008/`：FR 后 RR。
- `runs/shallow_press/wide/architecture_example_20261008/`：FL/FR 后 RL/RR。

使用单独的零输入基础设施诊断进程，各执行 2 s，均 COMPLETED、401 条 CSV、81 帧原生历史、无 DLL 错误。第一条原生反馈 Xo=99.1 m、Vx=0；坑入口 101.1 m。无闭环保持时，静态沉降期间最大纵向速度约 0.073 km/h。该诊断只验证初始化、求解与历史通路；明确记录 `strategy_status=NOT_IMPLEMENTED`、`task_status=UNASSESSED`，不评估浅坑通过成功。

两类 `open_run_visualizer.py --dry-run` 和 `export_native_video.py --no-launch` 都通过。随后分别启动真实 Visualizer，两进程存在。各自 ASCII 暂存目录含不同路径哈希，显示副本引用各自 OBJ/MTL，驾驶员默认隐藏，原历史保留不变。图像外观未在本次工具中进行逐帧人工检查；AVI 最后导出仍使用 Visualizer 的 File > Export Video 对话框，本次未生成 AVI。

## 实施选择与后续边界

- 按用户要求在当前 feature 分支原目录实施；备份标签和完整 Git bundle 保留重构前版本，未合并其他分支。
- Windows 使用 PowerShell 维护执行记录；保持原 Python 环境以避免数值依赖变化。
- 黄金测试覆盖确定性控制阶段，原生对比负责真实行为等价；黄金测试不代替可行性验收。
- 旧脚本保留可导入兼容模块，新入口使用严格认领目录的内部 `--claimed-output`。
- 胎宽半包络 0.10 m 是软件场景覆盖校验包络，未声明为测量硬件胎宽。显示相机不计入物理车辆身份，完整文件 SHA 仍保留且旧增益校验不放宽。
- 缺少本地网格的旧历史保留原播放行为；新准备场景必须自带网格。计划中的不存在测试文件名改用现有 `test_batch_and_video.py`。
- 算法、参数和产物隔离已建立；并行修改公共层仍需接口协调及各自分支/工作区。后续浅坑开发需确定实际尺寸、速度和验收阈值，实现控制律和独立验收后才开放正式运行。
