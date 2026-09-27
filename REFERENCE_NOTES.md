# M0 — 三篇机器人论文与参考代码笔记（2026-09-25）

> 本文是文献来源记录。当前原地 FR 抬轮的实测进展以 [README](README.md) 和[载荷优化报告](docs/static_fr_ii_load_balance_iteration_20260927.md) 为准。

三篇本地 PDF 已逐页提取并阅读，位于 `../文献/四轮机器人控制/`。下述映射是后续设计线索，不表示当前汽车模型已实现或验证。

| 论文 | 查阅页与关键思路 | 可移植到静态 FR 的部分 | 不移植的部分 |
|---|---|---|---|
| Yi 等，*Balance Strategy for a Quadruped-Wheeled Robot under Three-Leg Support Conditions* (`8_1.pdf`) | pp. 2–4：三腿支撑多边形、静态姿态/CoM 调整；pp. 2–3、6：五次多项式平滑过渡；pp. 4–5：动态情形含惯性补偿 | FR 移除后的 FL–RL–RR 安全三角形、稳定裕度、先移入安全域再卸载、连续位姿/力轨迹 | 腿端水平位移、关节逆运动学、靠行驶加减速生成惯性力。汽车平地原地场景不能移动轮接触点来复制该方法 |
| Bjelonic 等，*Keep Rollin’ – Whole-Body Motion Control and Planning for Wheeled Quadrupedal Robots* (`1809.03557v2.pdf`) | pp. 3–5：contact scheduler、support polygon、ZMP 约束；pp. 5–6：层级 WBC、接触力/约束任务 | 四接触→三接触→四接触模式，安全支撑多边形，稳定约束优先于 FR 轮载目标，受约束接触力优化 | 机器人浮基关节动力学、Jacobian、足端轨迹 IK、关节力矩求解；原地车辆不需滚动约束规划 |
| Bjelonic 等，*Whole-Body MPC and Online Gait Sequence Generation for Wheeled-Legged Robots* (`Whole-Body_MPC_and_Online_Gait_Sequence_Generation_for_Wheeled-Legged_Robots.pdf`) | pp. 3–5：单刚体模型、地面反力及 stance/swing 接触约束；pp. 5–7：lift-off、touchdown 与在线接触时序 | 将 FR STANCE 定义为 Fz>0 且参与载荷分配，SWING 定义为 Fz_ref=0 且跟随平滑高度轨迹；接触切换须有传感门槛和恢复路径 | 第一版不引入完整 WBC/MPC、关节速度优化或在线步态生成 |

静态车辆映射：接触位置取 TruckSim 原生 `Xctc/Yctc`；准静态 `p_ZMP=Σ(Fz_i p_i)/ΣFz_i`，同时用 `XCG_TM/YCG_TM` 核对。先把当前稳定点投影到收缩安全三角形的最近可行点，再评估行程、力、姿态是否可达。支撑轮载分配的硬约束优先于 FR 目标。五次轨迹适用于 FR 轮载过渡和后续高度过渡，但每次状态转移仍须由反馈与 dwell 门槛决定。

## 五个开源项目本机状态

M0 时指定位置 `Research/开源代码` 不存在。用户随后提供两个 GitHub 地址，已在同一本地项目下克隆 `legged_control`（`a7f381c`）和 `ocs2`（`2638675`）；其余 `awesome-wheeled-legged`、`go2w_rl_gym`、`Wheel_Legged_Gym` 仍未取得。机器人 ROS/URDF/Pinocchio/Jacobian、OCS2 全栈和 RL/PPO 均不作为首版依赖。

## 后续源码核查（2026-09-25）

`legged_control/legged_wbc/src/WbcBase.cpp` 把广义加速度、各接触三维反力、关节力矩纳入决策向量；`modeNumber2StanceLeg` 决定哪些腿处于接触。浮基动力学、驱动极限、摩擦锥、无接触足端零反力是高优先级等式/不等式。`HierarchicalWbc.cpp` 先组合这些约束，再将车身加速度/摆动腿、接触力目标分层；`HoQp.cpp` 用前一层零空间及 slack 保持优先级；`WeightedWbc.cpp` 则在硬约束下最小化加权跟踪误差。`legged_controllers/src/LeggedController.cpp` 将观测、接触状态、MPC 参考与 WBC 串成循环。汽车映射仅保留“接触模式 + 稳定硬约束 + 轮载软目标”，不复制浮基关节动力学。

`ocs2` 的 `ocs2_core/reference/ModeSchedule` 管理模式事件和序列，`TargetTrajectories` 管理时间索引参考；`ocs2_oc/synchronized_module/ReferenceManager` 同时保存两者；`ocs2_mpc/MPC_MRT_Interface` 将观测和更新后策略送入运行循环。对本任务，可用显式车辆 FSM 表达 4→3→4 接触模式，无需以 OCS2 为依赖。参见用户提供的上游仓库：https://github.com/leggedrobotics/ocs2 与 https://github.com/qiayuanl/legged_control。

M0 范围仅为文献与工程审计。本笔记不评价静态抬轮可行性；已有原地诊断的失败与模型差异见 `IO_MAPPING.md`。
