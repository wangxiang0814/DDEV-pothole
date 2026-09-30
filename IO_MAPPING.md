# 当前 I_I TruckSim–Python I/O 映射

本工程当前仅使用 `models/corner_module_ddev`：约 1.36 t 的 Compact Utility Truck，`VEHICLE_CODE I_I`，前后独立悬架。`models/corner_module_ddev/simfile.sim` 调用 TruckSim 2019 DLL；Python 入口为 `src/ddevsim/cosim.py:run_stepwise`。求解器积分 `tstep=0.0005 s`；`EXT_MODEL_STEP=0.01 s` 是外部模型配置，不可当作积分步长。CSV 的 5 ms 抽样只是记录间隔。

轮序固定 FL/FR/RL/RR = L1/R1/L2/R2，坐标为全局 +X 前、+Y 左、+Z 上。正 `Jnc` 表示悬架压缩；每角 `IMP_FS` 是弹簧座附加力（ADD 模式），实际力另由 `FsExt` 回读，不能仅凭命令名推断车身升降。当前静态试验中四个电机转矩恒为零。

| 轮位 | 转矩输入 N·m | 主动悬架输入 N | 轮载输出 N | 行程 / 行程速率 |
|---|---|---|---|---|
| FL | `IMP_MYUSM_L1` | `IMP_FS_L1` | `Fz_L1` | `Jnc_L1` mm / `JncR_L1` mm/s |
| FR | `IMP_MYUSM_R1` | `IMP_FS_R1` | `Fz_R1` | `Jnc_R1` mm / `JncR_R1` mm/s |
| RL | `IMP_MYUSM_L2` | `IMP_FS_L2` | `Fz_L2` | `Jnc_L2` mm / `JncR_L2` mm/s |
| RR | `IMP_MYUSM_R2` | `IMP_FS_R2` | `Fz_R2` | `Jnc_R2` mm / `JncR_R2` mm/s |

| 状态量 | 原生变量与说明 |
|---|---|
| 姿态与角速度 | `Roll_E`, `Pitch` 为度；`AVx`, `AVy` 为车身角速度，度/s。带轮位后缀的 `AVy_L1/R1/L2/R2` 为轮速 rpm。 |
| 速度 | `Vx` 为 km/h；当前合同无可信的 `Vy` 导出，接入蠕行控制前需增补并核单位。 |
| 接地点 | 增补只读输出 `Xctc_?i/Yctc_?i`，全局 m；不能用轮心 XY 冒充接地点。 |
| 重心 | 增补只读输出 `XCG_TM/YCG_TM`，全局 m；与 ZMP 交叉验证。 |
| ZMP | 无原生单通道输出；准静态估算 `Σ(Fz_i p_i)/ΣFz_i`，`p_i` 为同一次运行的轮胎接地点。FR 离地后使用 FL/RL/RR 三角形的 barycentric 与最小边距。 |
| FR 净空 | 目前无独立 contact flag。当前**平地隔离模型**使用 `Z_R1 − Zgnd_R1i − R0`，并同时要求 FR 近零轮载；坑槽场景必须重新计算真实轮胎包络。 |

车型参数、接口合同与软件限幅分别见 `models/corner_module_ddev/`、`src/ddevsim/interface_validation.py`、`src/ddevsim/static_wheel_lift/config.py`。当前成功试验的可移植模型和结果见 [evidence/static_fr_ii](evidence/static_fr_ii)。该隔离车型使用 200 kg 载荷后移 350 mm、左移 500 mm；力/速率限均为仿真软件限幅，不是硬件额定值。

FR→RR 完整循环的独立运行副本额外接入第九路 `IMP_STEER_SW`（`REPLACE`，方向盘角，度），用于三轮蠕行时修正横向偏差。原始车型及已有八路控制入口保持原样。TruckSim 自带的 `Run_imp_tab.txt` 和导入通道数据集将该变量定义为方向盘角；本机负角脉冲试验使正偏航减小，证据保存在本机 `runs/right_side_steer_probe_neg60/`。这是软件转向输入，尚无实体转向执行器约束。
