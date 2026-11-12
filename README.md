# DDEV TruckSim 专家策略平台

本项目使用 TruckSim 2019 和 Python。两个专家各自维护控制律、配置与验收，共享运行目录、场景资产、车辆接口和原生回放。

| 策略 | 入口与配置 | 适用范围 |
|---|---|---|
| 抬轮 `wheel_lift` | `src/ddevsim/strategies/wheel_lift/`、`configs/wheel_lift/recommended.json` | 单侧深坑，先右前轮再右后轮。推荐清单记录已通过的离散工况 |
| 下压 `shallow_press` | `src/ddevsim/strategies/shallow_press/`、`configs/shallow_press/` | 单侧或宽坑。`SIMULATION_PASS` 表示通过所配置的研究筛查 |

## 从场景到日志

以下命令从项目根目录执行。场景准备生成物理路面和对应的显示网格。实际求解为每轮启动独立进程。

~~~powershell
python scripts/prepare_strategy_case.py --case right_track --depth-m .04 --length-m .8 --width-m .9 --run-id scene_001
python scripts/run_strategy.py --strategy wheel_lift --case seed_repeat --run-id lift_001
python scripts/run_strategy.py --strategy shallow_press --case right_track --depth-m .04 --speed-kph 2 --duration-s 40 --delay-s .04 --config configs/shallow_press/preview_multispeed_v2_20261024.json --run-id press_001
~~~

两种专家都可用 `--dry-run` 检查参数。每轮写入 `runs/<strategy>/<case>/<run_id>/`。`run_manifest.json` 和 `result.json` 保存执行与验收结果，`run.log` 保存 worker 输出，CSV 保存原生信号及控制诊断，`model/output/` 保存原生回放历史。需要 VLA 示范时可传 `--demonstration-config`。

## 批量运行

将计划保存为 JSON。`args` 接收对应单轮入口的选项。

~~~json
{
  "version": 1,
  "runs": [
    {"strategy": "wheel_lift", "case": "seed_repeat", "run_id": "batch_lift_001"},
    {
      "strategy": "shallow_press",
      "case": "right_track",
      "run_id": "batch_press_001",
      "args": ["--depth-m", "0.04", "--speed-kph", "2", "--duration-s", "40",
               "--delay-s", "0.04", "--config", "configs/shallow_press/preview_multispeed_v2_20261024.json"]
    }
  ]
}
~~~

~~~powershell
python scripts/run_expert_batch.py --plan my_batch.json --batch-id pilot_001 --dry-run
python scripts/run_expert_batch.py --plan my_batch.json --batch-id pilot_001
~~~

批量入口先检查所有工况和输出冲突，再顺序启动单轮入口。`runs/_batches/pilot_001/summary.json` 逐轮记录退出码、任务状态、日志和可播放历史。采集与验收沿用单轮入口的定义。再次运行请指定新批次和运行编号。

## 检查原生回放

播放和导出接受完整路径、`.vs`/`.vsb` 文件或唯一的运行编号。同名轮次需要给出完整路径。

~~~powershell
python scripts/open_run_visualizer.py vla_on_20261021
python scripts/open_run_visualizer.py coverage_v2yaw_right_track_d04_v20_20261025_001
python scripts/export_native_video.py vla_on_20261021 --no-launch
~~~

欧式平头、封闭货厢和随轮心运动的机械展示部件由 `vehicle_appearance.py`、`display_chassis.py` 写入显示暂存资源。车辆动力学参数由 `vehicle_profiles/` 和对应车型源管理。AVI 导出在 VS Visualizer 的 **File → Export Video…** 中完成。

## 目录与历史

- `src/ddevsim/runtime/` 管理运行合同、批量计划和结果。
- `src/ddevsim/scenarios/` 与 `scenarios/` 管理场景生成和模板。
- `src/ddevsim/demonstration/` 管理相机示范与状态动作记录。
- `evidence/` 保留当前车型源、辨识增益、验证指标和样例。`archive/` 保存退役研究的可校验源码与证据。
- 本机 `runs/_cleanup_20261112/` 保存清理清单和历史结果索引。旧报告中的部分运行路径已退役，重新复核原始指标需要按冻结条件重新运行。
- 历史入口与研究结论见 [开发与复现](docs/STRATEGY_DEVELOPMENT.md) 和 [架构](docs/ARCHITECTURE.md)。清理前的根目录说明可用 `git show 840761d:README.md` 查看。

~~~powershell
$env:PYTHONPATH = 'src'
python -m pytest -q
git diff --check
~~~
