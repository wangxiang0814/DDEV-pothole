# 控制对象说明（角模块 DDEV）

本文件说明本平台当前使用的 TruckSim 控制对象、它的架构、实测参数，以及为使其成为
**有效**模型所做的三项显式修正。可直接用于向导师说明"控制对象是什么"。

---

## 1. 控制对象身份

| 项目 | 值 |
|---|---|
| TruckSim 数据集 | `Vehicle: Loaded Combination` → **`Compact Utility Truck (I_I)`** |
| 类别 | TruckSim `2A - Utility Vehicles`（两轴轻型商用工具车） |
| **车辆结构代码** | **`i_i`** —— 第一位 = 前桥，第二位 = 后桥，`i` = **Independent（独立悬架）**，`s` = Solid（刚性桥） |
| 源工况 | `F:\TruckSim2019\TruckSim2019.0_Data\Results\Run_b20aee53-c150-44b1-a804-20e43bfde604\run_all.par` |
| 生成目录 | `models/corner_module_ddev/` |

**对照**：原先使用的 `HD Utility Vehicle w/ Crate` 是 `s_s`（前后**刚性桥**），四角垂向
响应经车桥机械耦合，不能作为角模块控制对象。本平台的 `Compact Utility Truck` 是
`i_i`，前桥与后桥**均为独立悬架**。

## 2. 架构验证（不靠车型名称，直接查模型）

生成器会把下面的结论自动写入 `models/corner_module_ddev/source_manifest.json`：

```json
"suspension_architecture": {
  "vehicle_code": "I_I",
  "front_axle": "independent",
  "rear_axle": "independent",
  "corners_mechanically_independent": true,
  "independent_kinematics_datasets": [
    "Compact Utility Truck - Drive Axle",
    "Compact Utility Truck - Steer Axle"
  ],
  "compliance_datasets": [
    "Compact Utility Truck - Drive Axle",
    "Compact Utility Truck - Steer Axle"
  ]
}
```

即：前后桥都用 `Suspension: Independent System Kinematics`，且每桥有各自的
`Suspension: Independent Compliance, Springs, and Dampers`（**每角独立弹簧与阻尼**）。
因此一个角上的作用力只作用在该角，四个角的高度可以各自设定——这是角模块架构的定义。

## 3. 八执行器接口

原机械动力链由 `OPT_PT 0` 旁路，四个轮端电机与四个主动悬架作动器直接由外部控制器驱动：

```text
u = [ IMP_MYUSM_L1, IMP_MYUSM_R1, IMP_MYUSM_L2, IMP_MYUSM_R2,   # 轮端转矩 N·m
      IMP_FS_L1,    IMP_FS_R1,    IMP_FS_L2,    IMP_FS_R2 ]      # 弹簧座附加力 N
```

轮序固定为 `FL, FR, RL, RR`。两组接口均为 `ADD`，零指令时退化为被动悬架。

**验证结果（在该控制对象上重新跑过 17 轮同车脉冲矩阵）**

```
verdict: PASS
torque: FL True  FR True  RL True  RR True
active: FL True  FR True  RL True  RR True
input integrity: 17/17 全部通过（每个案例只有目标输入列非零，无端口串线）
```

## 4. 实测车辆参数

**质量**
| 项目 | 值 |
|---|---|
| 簧载质量 `M_SU` | 600 kg |
| 载荷 `M_PL` | 3 × 200 kg = **600 kg**（TruckSim 按载荷实例逐个声明，须求和） |
| 非簧载质量 `M_US` | 80 kg / 桥（每角 40 kg） |
| **总质量** | **1360 kg** |
| 静态单轮载荷（实测） | 3484 / 3483 / 3163 / 3163 N |
| 静态总重（实测） | 13 292 N（= 1360 × 9.81 = 13 337 N，误差 0.3%） |
| 轴荷分配 | 前 52.4% / 后 47.6% |

**惯量** `IXX 384.0`、`IYY 624.2`、`IZZ 686.9` kg·m²

**几何**
| 项目 | 值 |
|---|---|
| 轴距 | 1925 mm |
| 轮距 | 1260 mm |
| 轮胎半径 `R0` | 263 mm |
| 簧载质心高度 `H_CG_SU` | 700 mm |
| 质心到前轴 / 后轴 | 0.9161 m / 1.0089 m（由实测静载反推，两者之和 = 轴距 1.9250 m） |

**悬架与轮胎**
| 项目 | 值 |
|---|---|
| 形式 | 前后独立悬架，每角独立弹簧与阻尼 |
| 前簧刚度 | 30 N/mm（原始值） |
| 后簧 | 非线性表（`FS_COMP_TABLE`），60 mm 附近约 88 N/mm |
| 轮胎 | 175/70 R13，`FZ_REF` 4100 N（额定高于 3322 N 静载，无需修正） |
| 缓冲块行程 | 前 **121 mm**（原始 61 mm，已修正，见第 5 节）/ 后 101 mm |
| 静态下沉量（实测） | 前 168.1 mm / 后 66.6 mm |

**仿真** 求解步长 0.5 ms（定步长）；控制器采样 10 ms（与论文一致）

## 5. 为使其成为"有效"模型所做的三项显式修正

TruckSim 2019 的这个数据集**开箱即用是有缺陷的**，三项修正都已写入
`source_manifest.json`，可复核。

### 5.1 前缓冲块行程（本轮的关键修复）

数据集的前缓冲块表以 **61 mm** 结束：`50,0 / 60,0 / 61,7000`，但该车**自身的静态坐姿是
80.03 mm**。表格按最后一段（7000 N/mm）外插，于是 t=0 时前角被施加

```
7000 + (80.03 − 61) × 7000  ≈  140 kN
```

的力——**约等于整车重量的 10 倍**。这就是模型静止就剧烈振荡、前轮被抛离地面、任何控制器
看起来都失效的原因。**证据**：该数据集自带的 TruckSim 运行日志
（`LastRun_log.txt`）在 `T = 0` 报出完全相同的警告。

**修正**：把静态坐姿以下的缓冲块行程外移（默认 121 mm），让缓冲块恢复"真正到底才起作用"。
效果实测：

| 指标 | 修正前 | 修正后 |
|---|---|---|
| 致命的越上界外插 | 存在（≈140 kN/角） | **消失** |
| `quiet_at_rest` | False（漂移 0.518） | **True（漂移 0.025）** |
| 静止四轮载荷 | 左右不对称 | **3484/3483/3163/3163，对称** |

### 5.2 载荷求和

`M_PL` 按载荷实例逐个声明（`M_PL(1..3) 各 200 kg`），必须求和。只取第一个会把整车算成
960 kg，使三轮支撑静力解整体偏小 42%。

### 5.3 作动器力限幅按车型定标

原 ±100 kN 是按 8.9 t 卡车设的；对 1.36 t 的车这是整备质量的 7.5 倍，会把车掀飞。
现按 `5 × 最大静态角载荷` 自动定标 → 角模块车 **±17.4 kN**，卡车数值不变。

## 6. 专家策略复现质量（本平台的核心结论）

论文：Liu et al., *IEEE/ASME Trans. Mechatronics* 29(6):4480-4491, 2024。
策略：抬起目标轮 → 三轮支撑 → 保持车身姿态稳定通过坑槽。

| 指标 | HD 卡车（刚性桥） | **角模块车（I_I，修正后）** |
|---|---|---|
| 到达坑口**之前**目标轮卸荷程度 | 71% | **100%** |
| 达到该卸荷所需作动力 | −16.9 kN | **−9.4 kN** |
| 目标轮完全卸荷位置 | 101.097 m | **100.895 m（坑口 101.10 前 20 cm）** |
| 过坑期间目标轮载荷 | 0 N | **0 N** |
| 安全带停机 | 未触发 | **未触发** |

**结论**：在角模块控制对象上，目标轮在**到达坑口前 20 cm 就已被完全卸荷**，而且只需要卡车
**55%** 的作动力。这正是"角模块独立悬架"相对"刚性桥"的意义所在——专家策略在该车型上
复现得**更好**。

## 7. 复现命令

```powershell
$env:PYTHONPATH='src'
python scripts\build_corner_module_ddev.py            # 构建角模块控制对象
python scripts\validate_hd_ddev_interfaces.py --simfile models\corner_module_ddev\simfile.sim --target runs\corner_module_interface_validation
python scripts\probe_actuator_gain.py --model corner_module --out-dir runs\_actuator_gain_corner_module
python scripts\build_corner_module_pothole_case.py    # 生成单轮深坑工况（按新车型重定标）
python scripts\run_expert_pothole.py --model corner_module
```

### 可选的有效域杠杆

```powershell
--payload-scale 0.3333                 # 只留一个 200 kg 载荷
--steer-spring-rate-n-per-mm 70        # 加硬前簧（实测会把运行变成提前终止，默认不改）
--steer-jounce-stop-mm 0               # 恢复原始 61 mm 缓冲块行程（会复现振荡）
```
