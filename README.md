# TruckSim–Python 四角独立悬架研究工程

## 当前控制对象与进展

唯一当前车型是 TruckSim 2019 `corner_module_ddev`（Compact Utility Truck，`I_I` 前后独立悬架，约 1.36 t）。四轮独立转矩和四角主动悬架力通过原有八通道接口输入；没有更换 TruckSim plant。求解器 `tstep=0.0005 s`，`simfile.sim` 的 `EXT_MODEL_STEP=0.01 s` 是另一项设置，不是求解积分步长。当前试验的 200 kg 载荷后移 350 mm、左移 500 mm，仅在随附的隔离平地模型中使用。

**已完成原生静态验证：**平地、零电机转矩，FR 卸载并离地，FL/RL/RR 三轮正载，CoM 与轮载 ZMP 位于支撑三角形内，FR 净空超过 10 mm 持续 5.66 s，随后平滑落轮并恢复四轮。保持段 FL/FR/RL/RR 轮载约为 `6396/0/1229/5710 N`，ZMP 最小重心坐标裕度约 `0.092`。可审阅的[精简证据](evidence/static_fr_ii/lift_summary.json)与[轨迹图](evidence/static_fr_ii/lift_evidence.png)随仓库保存。

**当前限制：**这是一段经过原生仿真验证的预载轨迹重放、局部悬架力调整和在线异常恢复试验，不是已经闭环验证的低速蠕行控制器。三轮轮载无需均载；下一阶段以支撑裕度、每轮可用摩擦力、横摆与轨迹误差为约束，验证三轮扭矩分配及扰动下的重心维持。旧的 `expert_controller.py` 是移动越坑研究代码，不代表当前 FR 静态试验的控制回路。

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
```

压缩预载轨迹只保存仿真时间及八个输入通道；复核命令在相同隔离车型上重放它。此命令用于**重复已验证的静态试验**，不等于从任意初始工况自动规划抬轮。新车型或新载荷必须先重新辨识和验证可行性。四角力/速率限目前是仿真软件设定，未标称为硬件额定值；详见 [I/O 映射](IO_MAPPING.md)和[控制对象说明](docs/control_object.md)。

要从原始 I_I 接触几何工况复核 M1/M2，可运行 `python scripts/run_static_fr_m1_m2.py --output runs/fr_static_ii_m1_recheck`；其输入模型保存在 `evidence/static_fr_ii/m1_source/`。M3 的局部增益记录在 `evidence/static_fr_ii/gain_matrix.json`，不能替代新工况下的重新辨识。
