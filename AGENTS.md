# 专家策略独立开发约定

- 本目录是 Git 项目根目录；先读 docs/ARCHITECTURE.md 与 docs/STRATEGY_DEVELOPMENT.md。
- 抬轮控制属于 src/ddevsim/strategies/wheel_lift/ 与原 static_wheel_lift/；浅坑下压属于 strategies/shallow_press/。两个策略不得互相导入、借用专属状态机或修改对方配置。
- 策略不得导入 scripts。runtime/scenarios/vehicle_profiles 等共享层不得依赖策略专属控制、参数或辨识实现。registry 只做显式延迟加载。
- 所有可变控制状态归属于实例；参数显式传入，禁止运行时修改模块全局 CFG。
- 两者默认使用同一 tuned_ii_20261005 车辆生成规则。修改坑形不能改变车辆或绕过旧增益完整哈希校验。
- 浅坑单侧定义为 FR 后 RR；宽坑为 FL/FR 后 RL/RR。分别描述纵向 length_m 和横向 width_m。
- 每次生成独立 runs/<strategy>/<case>/<run_id>/，重复拒绝覆盖。模型源、历史证据与已验证清单不得作为生成目标。
- 原生 DLL 每次仿真独立进程，不在同进程线程并行调用。共享模块编辑需协调；并行开发使用各自 feature 分支/工作区。
- 旧 CLI 和验证阈值保持兼容。公共接口变更须显式版本化并回归两侧，不能用重构顺便调抬轮参数。
- 场景物理地图与显示网格来自同一配置，播放只使用对应运行资源。保留默认隐藏驾驶员及 ASCII 暂存。
- 尚未实现的浅坑控制明确报 NOT_IMPLEMENTED。任务 PASS 必须基于完整专属验收，不能仅凭进程退出/求解结束/异常恢复。
- 检查使用 PYTHONPATH=src 的 python -m pytest -q 及 git diff --check；原生许可证不可用时明确记录未验证范围。
