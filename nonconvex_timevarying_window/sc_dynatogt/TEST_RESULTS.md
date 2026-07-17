# SC-DynaTOGT 验证记录

初始验证日期：2026-07-14；闭环演示更新验证日期：2026-07-17（Asia/Shanghai）。命令均在仓库根目录执行。

## 单元、梯度与回归测试

```bash
pytest -q nonconvex_timevarying_window/sc_dynatogt/tests
```

2026-07-17 不规则闭环、真实场景可视化及 OpenGL 离线渲染更新后的完整测试结果见本节末。覆盖边界稠密化和 Chang 采样、角点保留、Clipper2 偏置、SC 参数求解/映射/持久化、degree-7 MINCO、四旋翼平坦性、完整 `[K,D]` 梯度、动态窗口、任意有序边界列表的多窗口场景、显式三维窗口位姿与运动倍率、闭环端点、论文风格布局、独立默认输出目录与历史布局保留、只显示真实物理边界的统一门框、四旋翼位姿绘制、实验与渲染 CLI，以及 PNG/CSV/GIF/MP4 输出和中文图解生成。

```bash
python -m compileall -q nonconvex_timevarying_window/sc_dynatogt
pytest -q
```

结果：`compileall` 通过；2026-07-17 仓库根 `pytest.ini` 指定的 `gap_step/tests` 回归为 `46 passed in 3.10s`。AirSim 风格渲染更新后，本方法为 `104 passed in 31.18s`；需按上一条命令单独执行，与已有 Atlas 方法的组织方式一致。

## E0–E5 端到端 smoke

```bash
python -m nonconvex_timevarying_window.sc_dynatogt.experiments \
  --suite smoke \
  --outdir nonconvex_timevarying_window/sc_dynatogt/results/smoke
```

六组均返回 `passed=true`：

| 组别 | 主要结果 |
|---|---|
| E0 | 原凸映射与 SC 的总时间相对差 `1.50645e-12`，低于 1% |
| E1 | 6 类边界全部找到合格采样数 |
| E2 | SC 收敛率 `1.0`，真安全多边形合法率 `1.0` |
| E3 | 收敛率/指定顺序合法率均为 `1.0`；窗口梯度 p99 `1.95e-8`，联合梯度 p99 `6.24e-6` |
| E4 | 收敛率/指定顺序合法率均为 `1.0`；窗口梯度 p99 `2.25e-8`，联合梯度 p99 `1.04e-5` |
| E5 | 完整时间梯度与零窗口时间梯度消融都收敛且合法 |

实验数值和代表轨迹保存在 `results/smoke/E*/summary.json`、CSV 和 PNG 中。

## 完整 E1 边界矩阵

```bash
python -m nonconvex_timevarying_window.sc_dynatogt.experiments \
  --suite default --experiment E1 \
  --outdir nonconvex_timevarying_window/sc_dynatogt/results/e1_default
```

结果：6 类边界 x 5 个顶点数，共 30 组全部通过。各类在全部顶点数上的最大边界误差为：

- L/U/五角星：`0 m`；
- limaçon：`0.00018044 m`；
- 波浪边界：`0.00098561 m`；
- 直线–Bézier 混合边界：`0.00196985 m`。

均低于方案的 `0.005 m` 上限。

## E0–E5 default 完整实验

2026-07-14 完整执行了 E2 的 30 个种子、E3/E4/E5 各 155 个种子、三窗口 L/U/五角星轨迹以及每个被验证映射的百万点检查。独立种子分块在 20 个 CPU 核上并行，合并时严格检查 seed 集合恰为 `0..154`；未降低样本数、映射验证数或优化配置。

| 组别 | 正式结果 |
|---|---|
| E0 | SC/原凸 TOGT 总时间相对差 `0.4942% < 1%` |
| E1 | 30/30 边界组合通过；最大误差 `1.9699 mm < 5 mm` |
| E2 | SC `30/30` 收敛且合法；凸包虽 `30/30` 收敛，但仅 `1/30` 在真非凸安全区内 |
| E3 | 收敛 `151/155 = 97.42%`，合法 `155/155`，窗口梯度 p99 最大 `2.68e-8` |
| E4 | 收敛 `153/155 = 98.71%`，合法 `155/155`，窗口梯度 p99 最大 `4.22e-8` |
| E5 | 完整/去时间梯度均收敛 `153/155 = 98.71%`、合法 `155/155` |

E3/E4 的三个窗口映射均为 `1,000,000/1,000,000` 点合法。`results/default/E3|E4|E5/` 中 2026-07-14 生成的 PNG/GIF 保留了当时的黑色虚线/彩色安全区样式；当前 `visualization.py` 重新生成时只画橙黑真实门框和四旋翼，不画安全区。历史产物不因样式升级而覆盖。

## 六形状六窗口动态演示

```bash
python -m nonconvex_timevarying_window.sc_dynatogt.diverse_demo \
  --mode full --quality smoke --layout paper_irregular --motion-scale 3.5 \
  --validation-samples 1000 \
  --outdir nonconvex_timevarying_window/sc_dynatogt/results/diverse_paper_irregular_closed_physical_scene
```

演示按 `L -> U -> 五角星 -> limaçon -> 光滑波浪 -> 直线–Bézier` 顺序穿越 6 个同时平移、旋转、缩放的窗口。窗口位置、RPY 和空间穿越模式参考原论文配套赛道的 `Gate1 -> Gate2 -> Gate3 -> Gate4 -> Gate6 -> Gate7`，整体放大后形成不规则闭环。起点与终点同为 `[-16,4,3.2] m`，中心覆盖 `x=[-9.9,20.24] m`、`y=[-13.2,14.96] m`、`z=[1.8,6.48] m`，七个航段长度为 `14.77--30.74 m`，运动振幅为正式 E3–E5 的 3.5 倍。

本次实跑结果：优化收敛，6/6 穿越点指定顺序合法，六个 SC 映射各 `1000/1000` 点合法，总时间 `14.90481 s`，385 次迭代，零无效试探点。输出的 `trajectory.png`、`trajectory.csv`、`dynamic_windows.gif`、六份预处理产物和 `summary.json` 均非空。这条轨迹的 `sampled_dynamic_limits_satisfied=false`：强运动长距离演示通过了窗口合法性，但未满足全部采样动力学硬上限，不将软惩罚收敛冒充为硬可行。

为避免覆盖历史实验，当前保留五个独立目录：旧开放布局 `results/diverse_demo/`、规则闭环中间版本 `results/diverse_closed_loop_regular_20260717/`、论文风格不规则闭环原版 `results/diverse_paper_irregular_closed/`、真实场景风格 `results/diverse_paper_irregular_closed_physical_scene/`、OpenGL 实体场景 `results/diverse_paper_irregular_closed_airsim_style/`。新版 Matplotlib 场景图不画安全区，只画真实时变边界的统一橙黑门框；静态图在穿越点画四旋翼，GIF 根据速度和加速度更新无人机的航向与倾斜。

这是功能和可视化演示，不改动 E0–E5 的正式统计定义。

## AirSim 风格 OpenGL 离线渲染

`simulation_render.py` 使用可选 `pyrender + trimesh + EGL` 后端，直接读取上述不规则闭环的 `summary.json`，重建同一条 MINCO 轨迹和六个真实时变边界。画面包含三维管状门框、四旋翼网格及航向/倾斜、建筑玻璃立面、道路、树木、方向光阴影、天空日光、距离雾、追踪相机和 HUD。安全区不进入场景。

实际在 RTX A4000 上通过 EGL 离屏渲染，并用 `imageio-ffmpeg` 编码成 H.264：

```text
results/diverse_paper_irregular_closed_airsim_style/airsim_overview.png  960 x 540
results/diverse_paper_irregular_closed_airsim_style/airsim_chase.png     960 x 540
results/diverse_paper_irregular_closed_airsim_style/airsim_chase.mp4    144 frames, 12 fps, 12.0 s
```

MP4 解码回读为 `codec=h264`、`pix_fmt=yuv420p`、`144/144` 帧可读，首帧、中帧和末帧均为有效 `960 x 540 x 3 uint8` 画面。该模块只做已求解轨迹的离线渲染，不声称实现 AirSim 动力学或传感器模型。

缩放状态另行核对如下：

| 窗口 | L | U | star | limaçon | wavy | line–Bézier |
|---|---:|---:|---:|---:|---:|---:|
| 穿越时 `s(t)` | 1.419 | 0.874 | 0.716 | 1.378 | 1.153 | 0.611 |

`simulation_render._window_pose` 在节点变换的 `3 x 3` 部分使用 `s(t)R(t)`，视频循环每帧都重新设置六个节点位姿。振幅为 `0.42`，完整范围为 `[0.58,1.42]`；追踪镜头同时改变视距和透视，门框也在旋转，且六种边界原始尺寸不一，因此画面中缩放不显著不代表变换缺失。固定距离 `GATE CAM` 和实时 `SCALE ×` 还没有实现。

## SC 映射合法性

```bash
python -m nonconvex_timevarying_window.sc_dynatogt.validation \
  --map nonconvex_timevarying_window/sc_dynatogt/results/smoke/E4/preprocessed_gates/00_L/sc_map.npz \
  --samples 1000000 --seed 0 --batch-size 4096 \
  --output nonconvex_timevarying_window/sc_dynatogt/results/smoke/E4/preprocessed_gates/00_L/validation_1m.json
```

结果：`inside=1,000,000`，`outside=0`，`NaN=0`，`Inf=0`，退化雅可比数 `0`，完整 `Psi(B(d))` 的最小 `|det J|=3.29215926e-05`。

直线–Bézier 混合边界另行通过了默认 CLI 预处理、artifact 重载、1000 个 `d~N(0,4I)` 映射点和 SC 网格绘图：`sampled_vertices=256`，`safe_vertices=106`，`inside=1000`，最小 `|det J|=2.70846156e-4`。

## 依赖环境

已验证的直接依赖版本：NumPy 1.26.4、SciPy 1.11.4、PyTorch 2.1.0+cu121、Shapely 2.1.2、pyclipr 0.1.8、Matplotlib 3.8.0、imageio 2.33.1、Pillow 10.2.0、PyYAML 6.0.1 和 pytest 7.4.0。

OpenGL 离屏渲染另行验证了 pyrender 0.1.45、trimesh 4.11.5、PyOpenGL 3.1.0、pyglet 1.5.31、freetype-py 2.5.1 和 imageio-ffmpeg 0.5.1；这些依赖仅由 `requirements-render.txt` 引入，不影响核心求解器。

## 已知限制

- 原 TOGT 使用平滑积分软惩罚，不是硬动力学约束。smoke 中主 SC 动态轨迹的单旋翼推力采样峰值为 E3 `5.11529 N`、E4 `5.00594 N`，高于 `5.0 N` 上限；代码如实输出 `sampled_dynamic_limits_satisfied=false`，不将软惩罚收敛冒充为硬可行性。
- `B(d)` 将有限变量映射到开圆盘。当时间最优解趋向窗口边界时，优化后 `|d|` 可达数百；映射与解析梯度仍有限，但不适合在该浮点饱和点上用固定 `h=1e-6` 做中心差分。联合梯度因此在方案指定的有限初始化 `d=0` 处检查，动态窗口梯度则使用随机 `d` 独立检查。
- default 中 E3/E4 各有 4/2 个种子未返回收敛标志，E5 两种方法各有 2 个；总体收敛率仍高于方案的 90% 门槛，且所有输出穿越点均通过真非凸安全区检查。
- OpenGL 追踪视频缺少固定尺寸参照，缩放可见性有限；当前只能通过状态函数、节点变换和上表数值验证，不能将未实现的 `GATE CAM` 表述为现有功能。
- 其余几何范围与 Clipper2 join 限制见 `README.md` 的“已知限制”。
