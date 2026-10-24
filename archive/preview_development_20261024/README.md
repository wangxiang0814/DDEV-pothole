# 预瞄反馈开发试验归档

主开发配置为`configs/shallow_press/preview_feedback_20261024.json`。本目录仅保留被淘汰或被替代的开发文件，避免在运行配置目录中留下多份候选参数。

- `tuning_configs.zip`：14份调参配置，逐文件及归档SHA256见`tuning_manifest.json`。包含未通过的符号坡度补偿、减速度超前预测、过强轮速反馈等试验；不能把它们当作发布参数。
- `observer_prototype.zip`：未获得足够改进的轮胎力/扰动观测原型及测试，来源见`observer_manifest.json`。
- 本地原件留在被忽略的`local_originals/`；所有成功、失败、诊断和模型对照的原生`runs/`均未删除或改写。

恢复时先校验清单哈希，再将所需条目解压到新的研究目录；不要覆盖当前控制器、原车型或历史运行。每次运行自己的`control_config.json`与`source/`快照是复现该次试验的直接依据。
