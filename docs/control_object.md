# 当前 TruckSim 控制对象：I_I 角模块车

截至 2026-09-27，工程的唯一当前车型为 `models/corner_module_ddev`，源于 TruckSim 2019 的 Compact Utility Truck。`VEHICLE_CODE I_I` 表示前后均为独立悬架；旧 `hd_utility_ddev` 是前后刚性桥 S_S，已不作为本项目当前控制对象。

| 参数 | 当前值与出处 |
|---|---|
| 总质量 | 约 1360 kg：簧载 600 kg、三个 200 kg 载荷、前后桥非簧载各 80 kg；见 `run_all.par` |
| 轴距 / 轮距 | 1925 / 1260 mm；见 `run_all.par` |
| 轮胎自由半径 | 四轮 `R0=263 mm`；见 `run_all.par` |
| 簧载质心高度 | `H_CG_SU=700 mm`；见 `run_all.par` |
| 簧载惯量 | `IXX_SU=384`, `IYY_SU=624.2`, `IZZ_SU=686.9 kg·m²` |
| 当前前后悬架几何止挡 | 压缩约 +160 mm、回弹约 −100 mm；原始 TruckSim 前止挡不适合静态车姿，修订原因见 `source_manifest.json`。这是悬架几何界，不是执行器额定行程。 |
| 四角执行器 | `IMP_FS_L1/R1/L2/R2` 弹簧座附加力；四轮驱动 `IMP_MYUSM_L1/R1/L2/R2`。信号映射见根目录 `IO_MAPPING.md`。 |
| 仿真频率 | `run_all.par` 的 `tstep=0.0005 s`；`simfile.sim` 的 `EXT_MODEL_STEP=0.01 s` 不同。 |

模型参数可追溯至 `models/corner_module_ddev/run_all.par`、`source_manifest.json` 和 `interface_contract.json`。原车型及后续隔离试验均未配真实电机/丝杠规格。旧移动越坑控制器 `expert_controller.py` 中 `force_limit_static_multiple=5.0`、`force_slew_time_s=0.4 s` 是**软件限幅**；静态 FR 诊断另用 `static_wheel_lift/config.py` 中的阶段配置，不能把两套参数混为硬件能力。

静态抬轮的可移植证据车型位于 `evidence/static_fr_ii/model`，相对当前基准 I_I 车型，仅在隔离副本中将现有 200 kg 后部载荷后移 350 mm、左移 500 mm，并设置平坦物理路面和静止制动。成功保持段三轮轮载和力见[结果摘要](../evidence/static_fr_ii/lift_summary.json)。该结果尚不证明低速蠕行稳定或真实硬件可实现。
