# 动力学数据集通道契约与评测指标

本文档说明每次仿真输出的通道、单位来源、如何校验，以及可计算的稳定性/通过性指标。
目的是让"用这批数据训练动力学代理模型与扩散模型、并评估稳定性与通过性"这件事有据可依。

## 1. 输出格式

| 项 | 值 |
|---|---|
| 行频 | 5 ms（200 Hz，`log_decimation=10` × 0.5 ms 求解步） |
| 控制器 | 10 ms 采样保持（100 Hz），与论文一致 |
| 列 | `time_s` + 8 个 `imp_*`（指令）+ 112 个 `exp_*`（反馈）= 121 列 |
| 契约来源 | `src/ddevsim/hd_ddev_case.py:DDEV_EXPORTS`（每轮通道）+ `pothole_case.py:SCENARIO_EXPORTS`（车身/场景） |

`interface_validation.EXPORT_NAMES` **派生**自 `DDEV_EXPORTS`，不再重复定义。导出是按下标绑定的，
列表一旦漂移，第一个差异之后的所有通道都会静默错位——这正是把它改成单一来源的原因。

## 2. 通道分组

| 组 | 通道 | 用途 |
|---|---|---|
| 执行器指令 | `imp_IMP_MYUSM_L1..R2`(N·m)、`imp_IMP_FS_L1..R2`(N) | 扩散模型的动作 |
| 执行器实际 | `FsExt_*`(N)、`My_US_*`(N·m) | 动作反馈（见 §4 注意事项） |
| 车身位姿 | `Xo/Yo/Zo`(m)、`Roll_E/Pitch/Yaw`(deg)、`Roll_Rd`(deg)、`Sta_Road`(m) | 稳定性指标、场景相位 |
| 车身速度 | `Vx`(km/h) | 通过性、损失 |
| 车身加速度 | `Ax/Ay/Az/Az_SM`(g) | 代理模型目标；`Az_SM` 是簧上质量垂向加速度 |
| 车身角速度 | `AVx/AVy/AVz`(deg/s) | 阻尼相关状态（注意 `AVy` ≠ `AVy_L1`） |
| 轮运动 | `X_*/Y_*/Z_*`(m)、`AVy_*`(rpm)、`AAy_*`(rad/s²) | 轮心几何、转速与角加速度 |
| 轮胎 | `Fz_*/Fx_*/Fy_*`(N)、`Kappa_*i`(–)、`Alpha_*i`(deg)、`CmpT_*i`(mm)、`RRE_*i`(mm)、`MuX_*i`(–) | 载荷、力、滑移、接地判据 |
| 悬架 | `Jnc_*`(mm)、`JncR_*`(mm/s)、`CmpS_*`(mm)、`Fs_*/Fd_*/FsExt_*`(N)、`CmpJSt*/CmpRSt*`(mm) | 行程、速度、力、缓冲块 |
| 地形 | `Zgnd_*i`(m) | 轮下地面高度（坑槽本身） |

**注意区分**：`Jnc_*` 是**车轮总行程**，`CmpS_*` 是**弹簧压缩量**，二者由运动比联系（本车前角静载时
168 mm 弹簧压缩对应 80 mm 车轮行程）。jounce/rebound 限值针对的是**行程**，所以任何与限值比较的
逻辑都必须用 `Jnc_*`。

## 3. 单位来源

通道表并非猜测，而是取自 TruckSim 自带的目录文件：

- `F:\TruckSim2019\TruckSim2019.0_Data\Results\Run_*\Run_out_tab.txt` — 输出变量目录
  （`Keyword, Units, Component Type, Full Label`）
- `Run_imp_tab.txt` — 导入通道目录（8 个 `imp_*` 的权威来源）
- `F:\TruckSim2019\TruckSim2019.0_Data\IO_Channels\O_Channels\Export_*.par` — 33 个 TruckSim
  自带的导出通道集

据此确定的关键单位：`Vz_Wc_*` = **km/h**（此前标为 `UNVERIFIED` 的正是这个 ~3.6 因子）、
`Ax/Ay/Az/Az_SM` = **g**（不是 m/s²）、`AVx/AVy/AVz` = deg/s、`AAy_*` = rad/s²、`CmpT_*i`/`RRE_*i` = mm。

单胎后缀用 `i`（两轴均为 `L_DUAL 0`、`itire 1`，每轮仅一个轮胎）。命名陷阱：`AVy`（车身俯仰角速度）
与 `AVy_L1`（车轮转速）是不同物理量；`CmpJStL1` 没有下划线，而 `FJSt_L1` 有。

## 4. 校验与已知限制

```powershell
$env:PYTHONPATH='src'; python scripts\report_run_metrics.py
```

该工具做的是**独立对照**，而不是量程检查——量程检查抓不到单位错误或通道错位：

| 对照 | 期望 |
|---|---|
| `Vz_Wc_L1` 对 `d(Z_L1)/dt` | 中位比 ≈ 1（确认 km/h） |
| `Zgnd_R1i` 最低点 对 配置坑深 | 一致 |
| `FsExt_R1` 对 `imp_IMP_FS_R1` | 一致（见下） |
| `Jnc_R1` 对 `CmpS_R1` | 高度相关 |
| `MuX_R1i` 对 配置摩擦 | 一致 |
| 平路上四轮 `Fz` 之和 对 车重 | 一致 |

已知限制（诚实记录，未解决）：

- **`FsExt_*` 与 `imp_IMP_FS_*` 逐点相等**，即它只回显指令，不能当作"实际作动力"用来检测执行器
  饱和或滞后。（`FsExt_*` 的注释写作 "realised" 是不准确的。）
- **`Zgnd_*i` 在极端悬架状态下不可靠**：曾测得最低 −0.461 m 而配置坑深仅 0.200 m。作为常规地形
  信号可用（多数运行中校验通过），但悬架行程使车轮远离路面时需要谨慎。
- **每轮驱动力矩目前只有一个自由度**：控制器把同一个小力矩复制到四轮
  （`expert_controller` 返回 `(self._applied_torque,)*4`）。若扩散模型要输出"每角"控制序列，
  当前没有能产生它的执行器——这是架构缺口，不是记录缺口。

## 5. 可计算的指标

`scripts/report_run_metrics.py` 在坑槽通过窗口内计算：

| 指标 | 定义 | 当前实测 |
|---|---|---|
| 侧倾/俯仰 RMS 与摆幅 | `Roll_E`/`Pitch` 的 RMS、极值差 | 4.30 / 2.32° RMS |
| 横摆摆幅与终值 | `Yaw` 极值差、末值 | 19.3° / −16.6° |
| 横向漂移 | `Yo` 极值差、末值 | 0.49 m / −0.48 m |
| 侧翻指数 LTR | 扣掉静态偏置的左右载荷差 / 总载荷 | 峰值 0.99 |
| 最小轮荷 | 四轮 `Fz` 最小值 | 0 N（抬轮） |
| 总轮荷峰值/车重 | 冲击强度 | 3.85 倍 |
| 簧上垂向加速度 | `Az_SM` 峰值 / RMS | 9.78 g / 0.43 g |
| 接地丢失时长 | `Fz < 500 N` 的累计时长（每轮） | FL 0.54 / FR 0.71 / RL 1.40 / RR 1.04 s |
| 四轮全腾空 | 全部 `Fz < 500 N` 的时长 | 0 s |
| 行程使用 | `Jnc_*` 极值 | 137.8 mm（**超出 121 mm 缓冲块**） |
| 执行器峰值 | `imp_FS` 极值 | 5862 N（限值 7262 N，未饱和） |

**评测必须同时看通过性。** 一次实测中把蠕行力矩降到 6 N·m 得到"最漂亮"的位姿指标
（横摆摆幅 2.5°、`Az_SM` 0.66 g），但车辆其实**卡在坑里没有通过**（只到 station 101.85）——
静止的车没有横摆。只看姿态会把失速误判为稳定，所以
`scripts/report_run_metrics.py` 的窗口与"是否通过坑槽"必须一起看。
