# TruckSim–Python 重载 4×4 DDEV 单轮深坑联合仿真平台

正式仿真对象：TruckSim 2019 `HD Utility Vehicle w/ Crate`。原车质量、惯量、几何、刚性桥、
叶簧、阻尼、轮胎和载荷参数保持不变；原机械动力链由 `OPT_PT 0` 旁路，外部控制器直接驱动
四轮轮端转矩与四角主动悬架附加力。

研究范围：**单轮迹深坑**（右轮迹），复现 Liu et al. 2024 *IEEE/ASME ToM* 的
"三轮支撑 + 抬轮过坑 + 车身姿态稳定"专家策略。其余场景类型留待后续扩展。

---

## 1. 八维执行器接口

固定轮序 `FL, FR, RL, RR`：

```text
u = [
  IMP_MYUSM_L1, IMP_MYUSM_R1, IMP_MYUSM_L2, IMP_MYUSM_R2,  # 轮端转矩 N·m
  IMP_FS_L1,    IMP_FS_R1,    IMP_FS_L2,    IMP_FS_R2       # 弹簧座附加力 N
]
```

两组接口均为 `ADD`，已由同车 17 轮正负脉冲矩阵验证
（`runs/hd_utility_ddev_interface_validation/`）。

## 2. 输出通道单位合同（重要）

TruckSim 2019 **不会**把 `EXPORT` 变量的单位写进生成文件，而且**不是全 SI**。平台早期把
逻辑名写成 `wheel_speed_radps` / `speed_mps`，与实际单位不符。现已用冗余通道实测确定，
见 `src/ddevsim/units.py`：

| 通道 | 实测单位 | 验证方式 |
|---|---|---|
| `AVy_*` | **rpm**（不是 rad/s） | 对 `60·d(Rot_*)/dt` 的比值 = 0.997–1.000 |
| `Vx` | **km/h**（不是 m/s） | 对 `3.6·d(Xo)/dt` 的比值 = 0.9999 |
| `Fz_*` | N | 静载合计 87296 N vs `m·g` = 8900×9.81 = 87309 N |
| `CmpS_*` | mm | 静载 53.6 mm；与 ±151 mm 行程一致 |
| `Roll_E`, `Pitch` | deg | `.vs` 头声明 |
| `Xo`, `X_L1..X_R2` | m | 坑口站 101.1 m 与 `X_R1=101.113` 吻合 |
| `Vz_Wc_*` | **未确定** | 与轮心垂向速度强相关（r≤0.996）但尺度在 3.1–3.9 间不可复现 |

`Vz_Wc_*` 被显式标记为 `UNVERIFIED`，`require_verified()` 会拒绝它进入控制器和数据集。
复测命令：

```powershell
$env:PYTHONPATH='src'; python scripts\verify_channel_units.py
```

## 3. 专家策略（论文复现）

参考：S. Liu, L. Zhang, et al., "Motion Posture Control of Corner Module Architecture
Intelligent Electric Vehicle on Deep-Potholed Roads", *IEEE/ASME Trans. Mechatronics*,
29(6), 4480–4491, 2024。论文轮序 1=左前 2=右前 3=左后 4=右后，与本平台 FL/FR/RL/RR 一致；
论文场景正是右轮迹深坑、轮 2 与轮 4 依次通过。

实现要点（`src/ddevsim/expert_controller.py`）：

- **支撑阶段状态机** Step 0–4（接近 → 抬轮2 → 恢复 → 抬轮4 → 恢复）。论文用固定时间窗
  （0–1.4 / 1.4–3.1 / 3.1–5 / 5–6.9 / 6.9–9 s）；本实现改为由**实测轮心站号**驱动，
  因此改坑位、坑长、车速都不需要重新整定。
- **三轮支撑静力解**：三个接触点、三个方程（垂向力、俯仰矩、侧倾矩）唯一确定。本车实测
  几何代入后，**不做重心偏移时左后轮载荷为 −1194 N（会翻车）** —— 这正是论文要解决的
  问题。按论文的姿态模式（抬升轮 1、4，压缩轮 3）使重心左移 37.5 mm，左后轮变为
  **+888 N**（论文原文即"轮 3 垂向载荷很小，假设为 0"），所需 0.114 m 差动行程在本车
  ±151 mm 行程内。这些数字都有测试锁定。
- **前馈**：不是简单 1:1，而是用**实测的 4×4 作动器增益矩阵**求逆。原因是 `IMP_FS` 作用在
  弹簧座而非接地点：本机实测 **本角增益只有 0.082**，抬起一个车轮需要最强角约 **78 kN**
  指令（当前默认限幅 ±100 kN）。单标量增益会让前馈差一个数量级。
- **反馈**：积分滑模 + 边界层（求解步长 0.5 ms，若用 `sgn` 会颤振），10 ms 采样保持
  （与论文一致）；另加姿态修正项跟踪论文的期望悬架挠度 `z_si,d`。
- **安全层**：行程、力、变化率、有限值检查，`SAFE_STOP` 锁存后退化为被动悬架。

**当前状态：策略结构已忠实实现并可运行，但在这个车型+0.45 m 深坑组合下还不能给出可信的
越障性能结论。** 实测数据表明瓶颈不在控制律，而在模型有效域与作动器权限：

- `runs/batch_sweep_a/` 的 QA 门控把全部工况判为 `usable=false`，原因包括
  `tyre_or_suspension_table_extrapolated`（轮胎表在 39 kN 外插，而峰值到 309 kN）、
  `suspension_travel_exceeded`、`whole_vehicle_airborne`；
- 相比之下 0.10 m 浅坑工况已明显好转（无安全带停机、无整车离地、峰值载荷 5.7 倍额定），
  说明 **0.45 m 坑深本身超出了本模型的有效域**；
- 因此下一步应先做模型标定（轮胎表重载段、限位表）或把"深坑"重新定级，再谈策略优劣。

## 4. 批量仿真与数据集

```powershell
$env:PYTHONPATH='src'
python scripts\probe_actuator_gain.py         # 实测作动器增益矩阵（跑批量前先做）
python scripts\run_batch.py --name sweep_a    # 默认扫描：深度 0.10/0.20/0.30 + 车速 1.4/5.6
python scripts\run_batch.py --name sweep_b --workers 4
```

- 每个 case 独立模型目录（`cases/<case>/model/`），历史 basename 唯一，互不覆盖；
- 数据集 `cases/<case>/dataset.npz`：`obs`(T×20, **SI**) / `act`(T×8) / `time_s`，附通道名；
  未定单位的 `Vz_Wc_*` 默认剔除；
- `dataset_index.csv` 汇总每个 case 的 QA 门控结果与历史路径；
- 并行是**进程级**（求解器 DLL 是进程内单例）。受限沙箱禁止命名管道时自动退回串行并在
  `parallelism` 字段记录原因。

## 5. 原生视频导出

```powershell
python scripts\export_native_video.py runs\hd_utility_ddev_expert_pothole\native
```

脚本校验并暂存历史、写 ANSI 编码的 `animator.par`、启动 VS Visualizer。**最后一步
`File > Export Video...` 必须手动**：TruckSim 2019 的 AVI 写出走 Windows VfW
（`AVISaveOptions` 压缩器对话框），命令行**没有**任何视频开关（完整 CLI 已从程序自身的
帮助对话框读出并逐条核对）。详细步骤、编码注意事项与故障排查见
`docs/native_video_export.md`。

> 注意：受限/沙箱会话里 VS Visualizer 因无法写 `%LOCALAPPDATA%\VS Visualizer\2019\`
> 而**根本起不来**，请在自己的交互式桌面会话中运行。

## 6. 常用命令

```powershell
$env:PYTHONPATH='src'
python -m pytest -q                                  # 78 tests
python scripts\build_hd_utility_ddev.py              # 生成 DDEV 基座
python scripts\probe_hd_utility_ddev.py              # 求解器配置探测
python scripts\validate_hd_ddev_interfaces.py        # 17 轮接口脉冲验证
python scripts\verify_channel_units.py               # 输出通道单位实测
python scripts\build_single_wheel_pothole_case.py    # 生成单轮深坑工况
python scripts\probe_actuator_gain.py                # 作动器增益矩阵
python scripts\run_expert_pothole.py                 # 论文策略闭环单跑
python scripts\run_batch.py --name sweep_a           # 批量仿真 + 数据集
python scripts\export_native_video.py <history>      # 原生视频导出
```

## 7. 目录

- `models/hd_utility_ddev/`：DDEV 基座与单轮深坑工况。
- `interfaces/trucksim2019_channels.json`：接口合同（含单位）。
- `src/ddevsim/units.py`：**单位合同与换算**（权威来源）。
- `src/ddevsim/vehicle_params.py`：从生成模型解析控制常量。
- `src/ddevsim/expert_controller.py`：论文三轮支撑专家策略。
- `src/ddevsim/batch.py`：批量仿真、QA 门控与数据集导出。
- `src/ddevsim/native_video.py`：原生视频打包与启动。
- `runs/hd_utility_ddev_interface_validation/`：17 轮接口验证。
- `runs/_actuator_gain/`：作动器增益矩阵实测。
- `runs/batch_<name>/`：批量结果、QA 索引与数据集。
- `docs/native_video_export.md`：原生视频导出说明。
- `docs/paper/deep_pothole_paper.txt`：参考论文抽取文本。
- `scratch_video_probe/`：VS Visualizer 命令行与 AVI 接口调查证据。
