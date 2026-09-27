# FR 原地抬轮精简证据包

本目录保存两份 I_I 平地隔离模型：`m1_source/` 用于重新生成四轮静稳 M1/M2 基线；`model/` 是移动现有 200 kg 载荷后的成功抬轮模型。两者都沿用 TruckSim 2019 plant 与八通道接口，`simfile.sim` 的安装路径须按本机核对。

- `result.json`：成功模型的 M1 静稳和支撑几何摘要；其 SHA-256 与 `model/run_all.par` 匹配。
- `gain_matrix.json`：该模型 ±500 N 静态小扰动 M3 的力、行程及姿态局部增益和病态诊断。仅作局部模型，不能直接外推到离地。
- `preload_result.json`、`preload_trace.csv.gz`：已达到 FR 近零轮载、三角形裕度达标的八输入指令时间历程。压缩轨迹只含时间和指令，不含原始 TruckSim 历史。
- `lift_summary.json`、`lift_evidence.png`：原生抬轮、三轮保持、落轮的量化摘要与图。
- `swing_identification_summary.json`：已离地工作点的 ±250 N 局部耦合响应。

从仓库根目录按 README 的命令重放。完整 5 ms CSV、ERD 与大量中间失败尝试属于本地 `runs/`，不上传 GitHub。此包能复核已验证的静态路径，不能证明控制器能在任意初值、地形扰动或低速蠕行下稳定。
