# 五相机专家示范数据采集（2026-10-21）

## 范围与现状

独立模块 `src/ddevsim/demonstration/` 复用现有 TruckSim 2019、原生车辆/路面显示与 Python 控制入口，不修改专家控制律、车辆、求解步长或验收线。只采集示范数据，不包含 VLA 训练代码。

已验收专家为 `wheel_lift`；`shallow_press` 已有独立研究控制器和记录入口，尚无完整安全PASS。浅坑使用 `configs/shallow_press/five_camera.json` 的专属任务指令；其 `SIMULATION_PASS`、独立轮胎对照及未知真实几何不能变成可训练成功示范。采集元数据保留实际策略、车型比较和输入来源身份。

采用**两阶段、按仿真时间对齐**的原生方案：

1. 求解时记录每个 0.5 ms 积分区间的积分前观测，以及真正提交给 `vs_integrate_io` 的动作。
2. 求解结束后，VS Visualizer 回放同一轮 `.vs/.vsb` 和运行专属路面，通过五个 Camera/RenderView/SharedBuffer 读取 RGB。以原生 `mFrameSimTime` 对齐状态/动作，五路时间必须完全一致。

这是示范数据生产，不是图像参与实时控制。原生动画历史间隔为 **25 ms**，渲染姿态由原生显示器插值；记录中明确保存这一视觉时间分辨率。状态按相邻 0.5 ms 观测线性插值，动作按实际提交区间零阶保持。图像保持实际时间戳，不伪装成整齐的固定时间网格，也不补造重复帧。

## 运行

从项目根目录执行，依赖沿用 `requirements.txt`，新增 Pillow。

```powershell
# 完整专家仿真 + 自动采集；run-id 必须是新的
python scripts/run_strategy.py --strategy wheel_lift --case seed_repeat --run-id vla_001 --demonstration-config configs/demonstration/five_camera.json

# 仅对已经记录 transitions 的轮次重新渲染，不重跑控制器
python scripts/capture_demonstration.py runs/wheel_lift/seed_repeat/vla_001 --capture-name episode_retry

# 调整相机后先做短预览；标记 PREVIEW，永不标为 trainable
python scripts/capture_demonstration.py runs/wheel_lift/seed_repeat/vla_001 --capture-name camera_preview --preview-seconds 1

# 检查镜头是否贴近真正车身表面，生成安装位置图
python scripts/check_camera_mounts.py runs/wheel_lift/seed_repeat/vla_001 --output runs/camera_mount_check

# 独立重读全部 PNG，检查时间戳、状态动作关联和数据质量
python scripts/audit_demonstration.py runs/wheel_lift/seed_repeat/vla_001/demonstration/episode
```

关闭采集：不传 `--demonstration-config`，或把配置 `enabled` 设为 `false`。本阶段依赖本机 VS Visualizer 64 位、VS Shared Buffer DLL、原生安装资源和可用 TruckSim 许可证；没有引入 Unreal 或 MATLAB 依赖。

渲染会打开单独的 VS Visualizer 实例。采集期间不要手动暂停该实例；结束后脚本关闭自己创建的实例。不会修改用户日常查看动画的默认设置。当前低倍速渲染用于保证集成显卡五路采样吞吐，增加的是数据生成的现实耗时，不改变控制策略的仿真周期。

## 相机安装与配置

`configs/demonstration/five_camera.json` 集中配置相机姿态、分辨率、视场角、裁剪距离、采样频率和采集质量线。默认 640×480、10 FPS 采样目标。实际采样频率和最大间隔写入报告。

五个镜头都绑定 **Xo/Yo/Zo + Yaw/Pitch/Roll_E** 的车身坐标系，独立于轮胎跳动/转向。坐标为 x 向前、y 向左、z 向上，单位 m；原点是 TruckSim 车身参考点，不是自行假定的 CoM。

- front：驾驶室前上方车身表面，朝前俯视。
- wheel_FL/FR：对应前轮轮拱边缘车身表面。
- wheel_RL/RR：对应后轮上方货厢/轮拱边缘表面。
- 镜头光学中心与安装表面相距 **20 mm**，表示小型相机外壳厚度；没有远离车身的虚拟悬浮支架。
- `mounts.py` 对实际原生 OBJ 三角形求交，核验原生显示偏移、表面锚点、镜头外伸距离及资源哈希。仅检查外包围盒不够。相机移到空中或车身内会拒绝采集。
- 近裁剪面为 5 mm，避免默认裁剪把近处车身裁掉造成“透视车身”。保留真实遮挡；轮侧相机不能看穿轮胎底部接触斑，记录的是可见轮胎和接触附近路面。
- 不添加会遮挡镜头的装饰相机模型；安装位置图为基于真实网格的几何检查图，原生 RGB 是另一份实际显示证据。

每帧保存原生相机全局位置/轴角、内参 K 和 `T_world_from_optical`。光学坐标 x 向右、y 向下、z 向前，RGB 左上角原点，理想针孔零畸变。安装姿态使用 yaw-pitch-roll；每帧交叉核验配置的车身变换与原生返回相机变换。

## 文件与字段

```text
demonstration/
  transitions.bin          全积分频率 float64 原始状态/实际动作
  transitions.json         列名、原生/SI 单位、完整标志、最终观测
  episode/
    episode.json           相机参数、场景、指令、来源哈希、任务/采集双重验收
    frames.jsonl           每行一组五相机、状态、动作、真实仿真时间、所属阶段
    images/front/*.png
    images/wheel_FL/*.png  其余 FR/RL/RR 同样分目录
    previews/*.png         各阶段首次画面的五路拼图
    audit.json             独立文件/同步审计（运行 audit 后生成）
```

动作按 FL/FR/RL/RR：四轮 `IMP_MYUSM_*`（N·m）、四角 `IMP_FS_*`（N），再加现有 `IMP_STEER_SW`（SI 记录 rad，原生 deg）。最后一路是**方向盘角，不是前轮转角**。原生符号保持不变；悬架力是弹簧座 ADD 附加力，正值增加弹簧力，不能把符号直接解释为轮胎上升/下降。转矩使用既有专家实际施加的符号；不额外翻转任何一轮。接口依据见 `IO_MAPPING.md`。

26 个可由传感器获得或估计的状态通道：Vx；Roll_E/Pitch/Yaw；AVx/y/z；Ax/y/z；四轮 AVy；四角 Jnc、JncR、FsExt。SI 分别为 m/s、rad、rad/s、m/s²、rad/s、m、m/s、N。Jnc 正值为压缩，保留原生通道轴定义；具体原生单位及换算因子逐字段随数据保存。

目前这些记录来自 plant 的传感器类真值通道，**并非专家内部加入噪声/延迟后的观测**。Vy 未接出，实际转向角传感器未接出；不虚构字段。CoM、接触几何、轮载和专家 FSM 不混入 policy state；阶段仅作为审查标签保留。场景参数/成功状态属于 episode 元数据。

## 验收与失败处理

专家任务 PASS 与图像采集 PASS 分开。只有完整非预览 episode 同时满足二者才 `trainable=true`：五路同时间、覆盖起止、无超过配置门槛的间隔、车身挂载一致、源历史未改变。中断、缺帧、无效状态记录均不能冒充训练成功样本。

记录器 I/O 异常会停用记录并报告错误，仍继续原有控制和求解器清理。图像在仿真结束后采集，图像写盘慢不会改变车辆运动。失败目录保留用于诊断；重新采集使用新的 `capture-name`，拒绝覆盖旧目录。

测试证据与五路样例位于 `evidence/vla_capture_20261021/`。大规模原始图片留在忽略的 `runs/`，不上传 Git；样例和精简审计可以版本管理。后续可转换为 LeRobot，本阶段没有声称已经是 LeRobot 数据集。
