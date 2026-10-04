# 在线分配：低附着与近起点复核

## 工况与辨识

固定 I_I 车型、质量、200 kg 载荷及其位置，compact3、横向运动学表 scale=0、RR robust、FR 转向反馈。路面 μ=0.6，起点前移 0.3 m，坑槽仍长 0.8/宽 0.9/深 0.2 m。使用已通过的 `front_rate3_lateral0_mu06_close` 为对照和保持指令来源。未改变增益权重、支撑/姿态/净空门槛、控制周期或 5 s 保持时间。

三种接触模式各基线加八次 ±250 N 探针，共 27 次原生辨识全部 SETTLED/PASS。有效轮载秩分别 3/2/2；条件数约 2.0e5、9.4e17、无穷，继续阻尼/正则优化，不能普通求逆。高度增益直接从各探针的稳态窗口计算。

发布 `support_gains_mu06_close_20261004.json`，源模型 SHA256 为 `1e51e0a92f0c16dcbd2cd8f52658e8f24bc2fcb465ae7067c6ccc2bd3334bacb`。包含完整辨识记录和三个来源文件哈希。新增打包脚本拒绝不同模型、失败模式、缺失/非有限矩阵，不覆盖历史增益。这是工况匹配标定后验证，不是固定一组增益对所有工况泛化。

## 原生结果与取舍

| 版本/种子 | 全循环 | RR 最弱支撑 N | RR 最小 ZMP λ | RR 最小坑上净空 mm | RR 横滚峰值 ° | 共用路径最大横移 cm |
|---|---|---:|---:|---:|---:|---:|
| 旧反馈基线 / 20261002 | PASS | 798 | 0.05984 | 32.17 | 9.24 | 2.17 |
| 新在线分配 / 20261002 | PASS | 832 | 0.06241 | 53.92 | 9.45 | 2.17 |
| 新在线分配 / 20261005 | PASS | 822 | 0.06170 | 48.41 | 9.15 | 3.81 |

每轮保持≥5 s、三支撑正载、CoM/ZMP 裕度、停车后落轮及四轮恢复均通过，无连续分配失败。第一组周期仍 87.68 s，卸载参考和动作时间未调整。第一组支撑/净空改善但姿态峰值增加，第二组横移较大：不能宣称所有指标都改善，也不能用两组样本证明统计鲁棒性。噪声模型仍只作用于轮载、横向位置、姿态和速度；接触几何、CoM、净空、角速度和行程仍理想化。

## 计算精简

通用上层轮载 QP 在抬起轮上下界固定时，实际上只剩三个未知轮载和三个平衡等式。新增中心化几何线性求解，检查总载荷/力矩残差与每轮上下界；不满足则 INFEASIBLE，退化三角形回到原通用路径。没有删减约束，不是将下层三悬架 QP 换为开环。

同进程交替 100 次测量：上层 FR p95 8.43→1.24 ms，RR 8.90→1.25 ms，轮载最大差异≤2e-12 N。记录两版源码哈希，见 `fixed_balance_benchmark_20261004.json`。这仅量化上层计算，不等于整个控制器或硬实时延迟保证。精简前低附着完整循环的上下层合计 p95 13.39–14.15 ms、最大 21.07–22.28 ms，少量超过20 ms，仿真仍沿用统一时间戳/固定步长。

精简后同种子完整循环独立复核 PASS：周期87.68 s、最大横移2.17 cm、RR最弱支撑832.22 N、最小ZMP坐标0.06242、横滚峰值9.44868°，与精简前基本一致。未并发运行其他本任务测试时，上下层合计平均3.23 ms、p95 6.77 ms、最大13.23 ms。先前与全套测试并发的同一复核也PASS、车辆指标一致，但墙钟p95 14.82 ms、最大209.77 ms；保留这项长尾，不能说多任务负载下保证50 Hz实时性。上述两个原生复核使用同一直接求解源码；本机其他应用负载未受控。

本轮304项Python测试通过。新测试验证FR/RR非零固定抬轮载荷及平移后的几何平衡，禁止唯一解情形调用迭代求解；原有不可行、一般QP和下层约束测试继续通过。精简前两组噪声结果与精简后同种子独立/并发复核均收入同一精简证据，避免混淆版本。

## 复现入口

```powershell
$env:PYTHONPATH='src'
python scripts/run_right_side_full_cycle.py --output runs/mu06_qp_recheck --efficiency-profile compact --lateral-ride-scale 0 --rear-control-profile robust --front-steering-feedback --front-target-speed-kph 3.4 --front-min-pit-speed-kph 3 --front-brake-lead-m 0.5 --rear-target-speed-kph 3.35 --rear-min-pit-speed-kph 3 --measurement-noise --noise-seed 20261002 --road-friction 0.6 --vehicle-start-offset-m 0.3 --support-allocation active --support-allocation-gains evidence/static_fr_ii/support_gains_mu06_close_20261004.json
```

重做辨识：对上述成功运行的 `model` 分别调用 `scripts/identify_current_model.py --contact-mode FOUR_CONTACT/FR/RR --source-model ... --reference-run ... --output ...`，再用 `scripts/bundle_support_identification.py --four-contact .../gain_matrix.json --fr .../gain_matrix.json --rr .../gain_matrix.json --output ...` 发布。不得改哈希绕过校验。

计算基准：`python scripts/benchmark_fixed_contact_allocation.py --baseline-ref cb4077b --output runs/fixed_balance_timing.json`。完整原生对照由 `scripts/summarize_right_side_matrix.py` 导出，精简证据保留各轮真实验收、配置、周期及源模型哈希。

## 后续范围

继续保持在线模式可选、默认基线；近起点低附着的匹配模型可显式启用。仍未验证任意附着、外扰、延迟、4–7 km/h 或全阶段统一分配。下阶段优先改进卸载/抬升的实时协同反馈，并复核初始化路径参考对观测噪声的敏感性；不依靠放宽姿态、支撑或直线验收换取 PASS。
