# 从历史仿真文件导出原生 TruckSim 视频

本文件说明如何从一次已完成的 TruckSim 仿真历史记录，导出**原生**的 TruckSim 3D 动画视频
（而不是用 NumPy 自绘的工程示意图）。

---

## 1. 结论先说：哪些能自动、哪些必须手动

我把 VS Visualizer 的**完整**命令行帮助从它自己的帮助对话框里读了出来（注意：`-h` 弹的是
GUI 消息框，不往 stdout 打印任何东西 —— 这一点很容易误判成“没有帮助信息”）：

```
Usage: VsVisualizer.exe [-help|-h] [-res <width> <height>] [-tb <0|1>] [-fs [0|1]]
       [-c <"config name">] [-pos <x pos> <y pos>] [-i <"instance name">] [-nosound]
       [-altresource [alt resources directory]] [-fscf <"full-screen configuration file name">]
       [-uimode <none|ani|plot|aniplot>] [-vsrap <"vsrap output file name">]
       [-vsrapoverwrite] [-r] [input.par]

    default          Load data from supplied file names.
    -altresource     Load resources from an alternate location.
    -c               Specify alternate configuration (preferences) to use.
    -fs              ... fullscreen (1, default), or windowed (0).
    -fscf            ... multi-monitor configuration file (documented as unsupported).
    -help, -h        Display this help message.
    -i               Specify a string to identify this particular execution instance.
    -nosound         Disable all sound processing.
    -pos             Resolution of render window in pixels.
    -r               Disable loading of data files (e.g. *.erd, *.vs).
    -res             Resolution of render window in pixels.
    -tb              Override toolbar (time control) visibility: 0 = hide, 1 = show.
    -uimode          Specify which UI elements are initially visible.
    -vsrap           Create VSRAP with the specified name from the Parsfile, then exit.
    -vsrapoverwrite  Overwrite VSRAP if it exists. Use with "-vsrap".
```

**没有任何 video / AVI / MP4 / export / capture / frame 开关。**

视频导出只存在于 GUI 菜单里：二进制内有 `Export &Video...`、`Generate an AVI file from this
run.`、`Export AVI File` 等字符串，并导入 `AVIFIL32.dll` 的
`AVIFileCreateStreamW` / `AVIMakeCompressedStream` / **`AVISaveOptions`** / `AVIStreamWrite`。
`AVISaveOptions` 就是 Windows VfW 的“视频压缩”压缩器选择对话框，**按设计就是模态的**，
二进制里不存在静默选压缩器的代码路径。所以最后一步无法脚本化。

**另一个必须知道的坑（已实测确认）**：VS Visualizer **会把命令行参数里的非 ASCII 字符
替换成下划线**。传入
`F:\1tongji\1 分布式电驱\...\animator.par` 时，它自己的日志记录：

```
Error: Unable to open parsfile file:
"F:\1tongji\1 _____\Research\TruckSim__\runs\...\animator.par"
```

中文字符变成了 `_____`，导致 parsfile 打不开、场景根本不会加载。
**所以必须把历史暂存到纯 ASCII 路径**（脚本默认就这么做，落在
`%TEMP%\ddev_native_video\`）。注意路径必须按**绝对路径**判断 ASCII 可表示性 ——
相对路径 `runs\batch_x\native_video` 看起来是 ASCII，但它拼出来的绝对路径不是。

> **本平台已实测通过的完整链路**（TruckSim 2019.0，本机）：
> 校验历史 → 暂存到 ASCII 路径 → 写 `animator.par` → VS Visualizer 成功启动并
> `Loading Dataset: "...single_wheel_deep_pothole.vs"`（无报错）→ 3D 形状资源加载
> （无警告）→ 命令行生成 `.vsrap` 成功（**52 793 字节**）。
> 唯一残留的、必须人工完成的只有最后写 AVI 的那一步。

> 顺带说明：本平台早期那个“工程回放 MP4”是用 `src/ddevsim/replay_video.py` 以 NumPy
> 软件投影自绘的示意图，**不是** TruckSim 3D 场景，不要用于论文插图。原生视频必须走
> VS Visualizer。

---

## 2. 一键脚本

```powershell
cd "F:\1tongji\1 分布式电驱\Research\TruckSim仿真"
$env:PYTHONPATH='src'

# 方式 A：指向某次运行的 native 目录（目录里有 .vs/.vsb/_all.par）
python scripts\export_native_video.py runs\hd_utility_ddev_expert_pothole\native

# 方式 B：直接指向 .vs 文件（批量仿真每个 case 的 model\output 目录里都有）
python scripts\export_native_video.py runs\batch_sweep_a\cases\baseline\model\output\baseline.vs

# 只校验+暂存，不打开界面
python scripts\export_native_video.py <路径> --no-launch

# 指定导出分辨率
python scripts\export_native_video.py <路径> --resolution 1920 1080

# 额外尝试用命令行生成 .vsrap（best-effort，见第 4 节）
python scripts\export_native_video.py <路径> --vsrap
```

脚本会做这些事：

1. **定位并校验**历史三元组 `.vs` / `.vsb` / `_all.par`。校验不只是看文件在不在：它会读
   `.vs` 的 JSON 头拿到通道数和 `XStep`，再按 `.vsb` 的二进制头（`<6i>`，第 5 个 int 是
   每值字节数、第 6 个是通道数）算出帧数，两者对不上就直接报错。
2. **暂存历史三元组**到目标目录（默认是历史所在目录下的 `native_video\`），并写
   `animator.par`。
   **编码很关键**：VS Visualizer 以 ANSI 读 parsfle。脚本用 Windows ANSI 代码页
   （`mbcs`，本机为 cp936）写这个文件，所以中文路径可以正常工作；如果误用
   `-Encoding Ascii` 写，路径会变成 `F:\1tongji\1 ?????\Research\TruckSim??\...`，数据集
   加载就会失败。只有当 ANSI 代码页确实无法表示某个字符时才会退回 UTF-8 —— 那种情况下
   请改用 `--ascii-stage` 把历史暂存到临时目录下的纯 ASCII 路径。
3. **`animator.par` 内容**：
   ```
   PARSFILE
   SET_RUN_SLOT 0
   DATASET   <...>\xxx.vs
   PARSFILE  <...>\xxx_all.par
   END
   ```
4. **启动 VS Visualizer**，带上 `-i <窗口名> -fs 0 -res 1280 720 -tb 1 -nosound` 和
   `animator.par` 的路径。
5. **打印下面的手动步骤**，并把 JSON 报告写到
   `<stage_dir>\native_video_export.json`。

---

## 3. 手动导出步骤（唯一必须人工的部分）

脚本跑完后 VS Visualizer 已经打开并载入了 TruckSim 3D 场景，接着：

1. 确认窗口里显示的是 TruckSim 的车辆/路面 3D 场景（不是曲线图）。顶部标题栏会显示历史
   文件名。
2. 确认回放覆盖整段仿真：报告里给了 `duration_s` 和帧数（例如本平台深坑工况为
   8.975 s / 360 帧 @ 0.025 s）。
3. **先调好相机再导出**。本平台的深坑模型已经把相机写进基础工况
   （azimuth −45°、elevation 14°、distance 16 m），直接用它即可；若要换角度，用
   Animator 窗口的相机控制或用相机数据集。
4. 菜单：**`File > Export Video...`**
5. 在对话框里设置时间区间 `0` 到 `<duration_s>`（报告里已给出具体数值），帧率填 `30`。
6. 填输出文件名，扩展名用 `.avi`。
7. 选压缩器：
   - `Microsoft Video 1` —— 所有 Windows 都有，体积小、画质一般，做检查用足够；
   - `Full Frames (Uncompressed)` —— 无损但文件极大（1280×720×30fps 约 80 MB/s）；
   - `H.264` —— 只有在机器装了对应 VFW 编码器时才出现在列表里。
8. 确定并等待。**导出是按真实时间逐帧渲染的**，所以 9 s 的仿真大约需要 9 s 加编码时间。
9. （可选）转成 MP4 便于放进论文/PPT：
   ```powershell
   ffmpeg -i native.avi -c:v libx264 -pix_fmt yuv420p -crf 18 native.mp4
   ```
   本机当前没有 `ffmpeg`，需要先安装（或用 `ImageMagick`/格式工厂等）。

输出分辨率由启动时的 `--resolution`（即 `-res`）决定。

---

## 4. 关于 `.vsrap`（已实测可用）

`.vsrap` 是 VS Visualizer 的“快速动画包”，把 `.vs`/`.vsb`/`parsfile` 打包成单文件，
双击即可播放，便于归档和给别人。命令行开关 `-vsrap` 官方支持且会 `then exit`。

**本平台已实测成功**：在 ASCII 暂存目录下执行

```
VsVisualizer.exe -vsrap <out.vsrap> -vsrapoverwrite <animator.par>
```

生成了 **52 793 字节**的有效 `.vsrap`（退出码仍是 −1，**不要用退出码判断成败**，看文件
是否存在且非空）。脚本里用 `--vsrap` 启用，失败或超时会记录而不抛错。

> 注意参数顺序：`-vsrap` 把**紧随其后的参数**当作输出文件名，所以 `-vsrapoverwrite`
> 不能插在 `-vsrap` 和文件名之间。

**视频导出本身并不需要 `.vsrap`** —— 直接用 `animator.par` 启动就够了。`.vsrap` 的用处是
归档和分发。

---

## 5. 批量仿真时怎么用

`scripts\run_batch.py` 会给每个 case 建独立模型目录，历史的 basename 就是 case 名，
所以在 `runs\batch_<name>\cases\<case>\model\output\` 下每个 case 都有一份独立历史：

```powershell
# 逐个导出（推荐只导代表性工况，因为最后一步要手动点）
python scripts\export_native_video.py runs\batch_sweep_a\cases\baseline\model\output\baseline.vs
python scripts\export_native_video.py runs\batch_sweep_a\cases\depth_010\model\output\depth_010.vs
```

`dataset_index.csv` 里的 `native_history` 列直接给出了每个 case 的历史目录路径，可以直接
喂给上面的命令。

> 提示：因为最后一步必须人工，**不要对上千个 case 逐个导视频**。建议先用
> `dataset_index.csv` 的 QA 列筛出可用工况，再挑代表性的几个（如 baseline、最优、最差）
> 导视频。

---

## 6. 故障排查

| 现象 | 原因与处理 |
|---|---|
| 日志里 `Unable to open parsfile file: "...\1 _____\..."` | **命令行非 ASCII 字符被替换成下划线**。必须用 ASCII 暂存路径（脚本默认已做）；不要加 `--no-ascii-stage`，除非整条绝对路径本来就是 ASCII。 |
| `incomplete history: ... is missing` | 该目录缺少 `.vs`、`.vsb` 或 `_all.par` 三者之一。注意 `_all.par` 是求解器归档出来的合并参数文件，不是模型的 `run_all.par`。 |
| `contains several histories` | 目录里有多个 `.vs`。直接指向具体 `.vs` 文件即可。 |
| `.vsb payload ... is not a whole number of frames` | 历史文件被截断（仿真中途被杀）。重新跑该工况。 |
| 界面起不来，弹 `can't open user configuration file.` | VS Visualizer 无法写 `%LOCALAPPDATA%\VS Visualizer\<版本>\`。属权限问题，与输入文件无关（零参数启动也复现）。换到有写权限的桌面会话即可。 |
| 提示实体形状资源缺失（`Unable to load asset file ... .obj`） | parsfle 里的形状资源是**相对路径**，VS Visualizer 按工作目录解析。必须以**含 `Animator\3D_Shape_Files` 的目录**为工作目录启动 —— 本机是 `F:\TruckSim2019\TruckSim2019.0_Prog\Resources`，**不是** `_Data`（`_Data\Animator` 里没有 HD Utility 的车辆网格）。脚本已默认这么做。**不要**用 `-altresource` 指向 `Prog\Resources`：那会替换整个资源根，导致 `Shaders\Components\lispm_shadow.vert` 找不到且数据集不再加载。 |
| 少量 `Failed to read image file ... .dds` 警告 | 缺的是法线贴图等外观贴图，不影响几何与动画，可忽略。 |
| 退出码是 −1（4294967295） | TruckSim 2019 的常态，**不能**作为失败判据；看产物文件是否存在且非空。 |
