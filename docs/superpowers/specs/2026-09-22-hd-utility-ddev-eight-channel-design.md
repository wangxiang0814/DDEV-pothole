# HD Utility Vehicle 八执行器DDEV改造设计

## 1. 目标与范围

本设计将TruckSim 2019的`HD Utility Vehicle w/ Crate`改造成同一车辆上的四轮独立电驱、四角主动悬架研究模型。保留所选TruckSim合并参数文件中的原始质量、惯量、两轴4×4越野几何、前后刚性桥、叶簧、阻尼器、轮胎和货箱载荷，不进行质量重参数化。

最终模型名称：

`HD Utility DDEV 4x4 Active Suspension`

该模型用于接口和控制研究；报告中的质量与惯量必须从最终`run_all.par`解析并记录，不能依据车型名称或旧报告手工填写。

## 2. TruckSim母型与质量一致性

母型数据集：

- Loaded Combination：`HD Utility Vehicle w/ Crate`
- Lead Unit：`HD Utility Vehicle`
- 原始越野示例：`River Crossing`
- 车辆结构代码：`S_S`
- 轴距：3,900 mm
- 前后轮距：1,975 mm
- 质量、惯量、载荷和非簧载质量：保持选定`run_all.par`原值，由构建器解析并写入源清单

构建器不得修改`M_SU`、`M_PL`、`M_US`、转动惯量、轴距、轮距、弹簧、阻尼器或轮胎参数。源文件中的实际参数值和SHA-256摘要必须写入清单，确保后续结果可追溯。

## 3. 动力链改造

原机械动力链不得继续向车轮提供驱动力。构建后的TruckSim参数设置：

```text
OPT_PT 0
```

文件内出现的所有有效`OPT_PT 3`均替换为`OPT_PT 0`。原发动机、变速器、分动器和差速器数据可以保留在合并参数文件中供动画或解析使用，但其动力学路径被禁用。

四个轮边电机使用TruckSim官方轮端/非簧载质量反力矩接口：

```text
IMPORT IMP_MYUSM_L1 Add 0.0! 0   # T_FL [N·m]
IMPORT IMP_MYUSM_R1 Add 0.0! 0   # T_FR [N·m]
IMPORT IMP_MYUSM_L2 Add 0.0! 0   # T_RL [N·m]
IMPORT IMP_MYUSM_R2 Add 0.0! 0   # T_RR [N·m]
```

固定轮序为`FL, FR, RL, RR`。不得使用`IMP_MY_OUT_D*`作为最终DDEV接口。

## 4. 主动悬架改造

原叶簧与阻尼器全部保留，主动执行器作为四个弹簧座上的并联附加力：

```text
F_total_i = F_passive_spring_i + F_passive_damper_i + F_active_i
```

接口定义：

```text
IMPORT IMP_FS_L1 Add 0.0! 0   # F_FL [N]
IMPORT IMP_FS_R1 Add 0.0! 0   # F_FR [N]
IMPORT IMP_FS_L2 Add 0.0! 0   # F_RL [N]
IMPORT IMP_FS_R2 Add 0.0! 0   # F_RR [N]
```

使用`ADD`而不是`REPLACE`，以保证零命令时车辆仍由被动悬架支撑。不得用`IMP_FD_*`替代主动执行器；`IMP_FD_*`是阻尼力接口，只适合半主动阻尼控制。

由于母型为前后刚性桥，同轴左右轮存在物理耦合。四角“独立控制”指四个输入可独立给定，不表示四个轮端垂向运动彼此解耦。

## 5. 八维输入合同

Solver API、Simulink和Python统一使用：

```text
u = [
  T_FL, T_FR, T_RL, T_RR,
  F_FL, F_FR, F_RL, F_RR
]
```

单位：

- `T_*`：N·m
- `F_*`：N

初始安全边界用于接口试验，不代表最终控制器额定值：

- 单轮转矩脉冲：±500 N·m
- 单角主动悬架力脉冲：±1,000 N
- 脉冲持续时间：0.10 s
- 任一未测试端口保持0

## 6. 输出合同

接口试验至少导出：

```text
AVy_L1, AVy_R1, AVy_L2, AVy_R2
Fz_L1, Fz_R1, Fz_L2, Fz_R2
CmpS_L1, CmpS_R1, CmpS_L2, CmpS_R2
Vz_Wc_L1, Vz_Wc_R1, Vz_Wc_L2, Vz_Wc_R2
```

其中轮速验证轮端转矩，垂向轮载、弹簧压缩量和轮心垂向速度共同验证主动悬架。若某个短变量名在该车型不可用，构建应失败并报告该变量，而不是更换为未经核对的近似变量。

## 7. 工件与目录

模型生成到项目内可复现目录，不覆盖TruckSim原厂数据库：

```text
models/hd_utility_ddev/
  run_all.par
  simfile.sim
  interface_contract.json
  source_manifest.json
  output/
```

脉冲测试结果：

```text
runs/hd_utility_ddev_interface_validation/
  baseline.csv
  torque_FL_pos.csv ... torque_RR_neg.csv
  active_FL_pos.csv ... active_RR_neg.csv
  manifest.json
  verification_report.md
```

## 8. 构建器行为

Python构建器接收缓存的River Crossing合并参数文件，执行以下确定性变换：

1. 将所有有效`OPT_PT 3`替换为`OPT_PT 0`；
2. 保持源质量、惯量、悬架、轮胎和载荷参数不变；
3. 将仿真时间缩短为接口脉冲测试所需时间；
4. 在最终`END`前按固定顺序加入8个IMPORT和16个EXPORT；
5. 写出`PORTS_IMP 8`、`PORTS_EXP 16`的`simfile.sim`；
6. 保存源文件SHA-256、变换后文件SHA-256、解析得到的车辆参数和接口合同。

任何目标模式匹配数量不符合预期时必须拒绝生成，防止静默生成错误车型。

## 9. 验证方法与判据

### 9.1 Solver配置验证

- `vs_read_configuration`返回READY；
- `n_import=8`；
- `n_export=16`；
- 求解步长大于0；
- 日志中无未识别变量、重复IMPORT或动力链配置错误。

### 9.2 零输入基线

八输入全为0运行相同时间，记录所有16个输出。所有脉冲试验均与该基线逐时刻相减，避免把道路或车辆自然运动误判为执行器响应。

### 9.3 四轮转矩测试

对每个轮端分别施加±500 N·m脉冲：

- 目标轮角速度相对基线有可测变化；
- 正、负脉冲的目标轮响应方向相反；
- 输入CSV中只有目标转矩端口非零；
- 四个轮端映射均通过。

刚性桥和轮胎-地面耦合会使非目标轮响应，因此不以“其他轮完全不动”为判据。独立性由输入可独立施加、目标响应占主导或符号一致、且四个通道无串线共同证明。

### 9.4 四角主动悬架测试

对每个弹簧座分别施加±1,000 N脉冲：

- 目标角`Fz/CmpS/Vz_Wc`至少一个通道相对基线有可测变化；
- 正、负脉冲产生相反方向的目标响应；
- 输入CSV中只有目标主动悬架端口非零；
- 同轴耦合被记录但不得出现端口串线；
- 四个角均通过。

### 9.5 通过边界

“符合主动悬架DDEV要求”的本阶段含义仅为：同一车辆的8个输入被TruckSim接受，四轮转矩和四角悬架力均能独立命令并产生物理响应。它不等价于已经完成电机额定参数标定、主动悬架控制律或深坑成功越障。

## 10. 后续控制方式

### Simulink

TruckSim S-Function输入向量严格按八维合同连接。电机控制器输出前4维，主动悬架和安全层输出后4维。每个通道在进入TruckSim前执行单位、符号、幅值、变化率和有限值检查。

### Python

Python负责参数生成、策略目标、批量仿真和数据整理。实时闭环优先在Simulink中执行；Python直接Solver API模式用于批量测试和无Simulink数据采样时，也必须复用同一八维合同。

### 安全默认值

- 转矩非法或数据失效：四轮转矩归零并请求制动；
- 主动力非法或通信中断：四角主动悬架力归零，退化为被动悬架；
- 行程、轮载、侧倾或支撑裕度越界：锁存SAFE_STOP。
