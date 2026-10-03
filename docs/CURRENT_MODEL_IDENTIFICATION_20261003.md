# 当前车型三接触模式辨识与反馈对照

## 实测结果

固定 I_I 质量/载荷，横向表 scale=0，使用标称成功循环的同一 `run_all.par`。每个模式各一次基线、四角 ±250 N 独立配对实验；每次从原生初始状态开始，三轮模式回放完整已记录输入到保持段 3 s 后，冻结输入再扰动。日志命令按前一采样保持，绝不继续进入蠕行。基线后 8 s 开始 2 s quintic 力变化，用第 16–18 s 的相邻窗口均值判断辨识稳定。

| 模式 | 原生局部辨识 | 有效轮载秩 | 条件数 |
|---|---|---:|---:|
| FOUR_CONTACT | PASS | 3 | 1.14×10⁵ |
| FR 抬起 | PASS | 2 | 9.46×10¹⁶ |
| RR 抬起 | PASS | 2 | 无穷大 |

保存 G_F、G_att、G_travel、G_CoM、G_ZMP 及具体冻结输入和各探针的均值/散布检查。奇异条件数在严格 JSON 中用 null 和 `condition_is_infinite=true` 表示；不将病态的小奇异值视作控制能力。三模式 source model 哈希相同。

辨识使用专用平均稳定条件，不把初始化 0.1°/s 门槛强加给后续调姿。三轮尾窗还检查抬起轮近零载、正净空、三支撑正载、CoM/压力中心三角形裕度、姿态和行程。**这些是局部辨识尾窗检查，不是完整回放全过程安全验证，更不是完整控制任务成功证据。**四接地测量也记录姿态和行程。原生提前终止或窗口不足时保存 FAIL 原因，不发布可用增益。

## 复现入口

```powershell
$env:PYTHONPATH='src'
python scripts/identify_current_model.py --source-model runs/front_rate3_lateral0_noise_nominal/model --output runs/reid_stance
python scripts/identify_current_model.py --source-model runs/front_rate3_lateral0_noise_nominal/model --reference-run runs/front_rate3_lateral0_noise_nominal --contact-mode FR --output runs/reid_fr
python scripts/identify_current_model.py --source-model runs/front_rate3_lateral0_noise_nominal/model --reference-run runs/front_rate3_lateral0_noise_nominal --contact-mode RR --output runs/reid_rr
```

成功参考运行及完整 CSV 为本机 runs 产物。若没有该运行目录，先按 README 推荐命令生成同一标称场景/种子；再用实际运行目录替换上述路径。输出目录须为新目录。每份 gain_matrix 的 source hash 取原成功循环模型；probe model hash 因改变 TSTOP 而不同，二者均保存。

## 接入现有反馈的结果

完整循环入口新增 `--contact-gain-bundle evidence/static_fr_ii/current_contact_gains_20261003.json`。在运行模型最终生成后按完整模型 SHA256 严格校验；不同附着、起点或几何导致哈希变化时须重新辨识，不能盲用这一 bundle。

当前接入范围：四接地矩阵用于 FR 轮载/支撑局部反馈与 RR 预载耦合标量；FR 三轮矩阵用于 FR 支撑反馈。RR 三轮矩阵仅保存，准备后续协同分配。CoM/姿态增益尚未完整接入实时约束优化。

同一带噪标称全循环结果 PASS：88.56 s、共用路径最大横移 3.28 cm、RR 净空最低 27.16 mm。对照仍用历史局部增益的 compact3 为 88.48 s、2.18 cm、30.45 mm。RR 横滚峰值 9.43→9.19°，但路径与净空余量变差，因此**不把“新辨识”视作整体性能改善，不默认启用 bundle**。

下一步以这些同车型、分接触模式的响应完善协同约束反馈，同时控制轮载、支撑裕度、姿态、输入与行程；不能仅替换一个增益矩阵就宣称鲁棒性改善。上层轮载 QP 已兼容 RR 并保留精确总载荷/力矩等式，但尚未在本轮原生循环中作为实时上层控制接入。

证据：[三模式测量](../evidence/static_fr_ii/current_contact_gains_20261003.json)、[闭环对照](../evidence/static_fr_ii/current_gains_feedback_trial_20261003.json)。
