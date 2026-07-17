# SC-DynaTOGT：Schwarz–Christoffel 非凸时变窗口

本目录是 `nonconvex_timevarying_window/` 下的独立算法实现。实现严格分开三件事：

1. Chang、Gotsman、Hormann 的方法只用于边界累计弧长均匀重采样和真实角点保留；
2. 安全区域内部点只由圆盘 Schwarz–Christoffel（SC）映射生成；
3. TOGT 使用原正时间变量、degree-7 MINCO、四旋翼微分平坦性约束和 L-BFGS 联合优化穿越点与时间。

这里没有使用 Chang 论文的 harmonic measure、Poisson kernel 或区域间参数化，也没有复用 `atlas_dynatogt/` 的三角 chart。

## 先看图

如果公式和源码较抽象，先看 [FIGURE_GUIDE.md](FIGURE_GUIDE.md)。它用 5 幅中文图依次解释整个算法、文件分工、动态梯度、三类可视化输出以及 E0–E5 结果；其中早期概念图与当前场景渲染的差异已在指南中注明。

如果需要从头理解 E0–E5、空间梯度、窗口时间梯度、MINCO、收敛与合法性等名词，请读 [EXPERIMENTS_AND_TERMS.md](EXPERIMENTS_AND_TERMS.md)。

## 安装

在仓库根目录执行：

```bash
source /home/jack/anaconda3/etc/profile.d/conda.sh
conda activate wyh
python -m pip install -r nonconvex_timevarying_window/sc_dynatogt/requirements.txt
```

`pyclipr` 是 Clipper2 的 Python 绑定；代码不会退回 Shapely buffer。PyTorch 只用于 float64 反向模式求导，SC 映射和优化器仍是 NumPy/SciPy 数据接口。

## 完整调用链

```text
Line / CircularArc / Bezier / BSpline / CSV
  -> 1 mm 弦误差、1 cm 最大弦长的稠密边界
  -> Chang 累计弧长均匀采样，m = 256/512/1024/2048/3200
  -> 角点原样保留、5 mm 全局误差、3 mm 凹陷误差
  -> Clipper2 向内偏置 0.315 m
  -> SC 圆盘预顶点离线求解、polylabel 归一化
  -> q_i = Psi_i(B(d_i))
  -> p_i = c_i(t_i) + E_i(t_i) s_i(t_i) q_i
  -> x = [K,D] -> degree-7 MINCO -> TOGT 代价 -> L-BFGS-B
```

窗口姿态与原复现一致：输入顺序是 `roll,pitch,yaw`，旋转矩阵为 `Rz @ Ry @ Rx`，窗口局部平面使用旋转矩阵前两列。

L-BFGS 数值配置直接对齐复现包的 `standard_lbfgs.yaml`：`memorySize=256`、`past=32`、
`maxLinesearch=64`、`maxIterations=0`、`relCostTolerance=1e-5`、`relGradTolerance=0`。SciPy
的 `ftol` 只比较相邻迭代，因此 `optimizer.py` 用环形缓冲器实现原 C++ 的 32 次历史代价停止准则；
`maxIterations=0` 则以最大 32 位整数预算表示，实际由历史代价准则终止。

## 离线预处理

Python API：

```python
from nonconvex_timevarying_window.sc_dynatogt.preprocessing import (
    PreprocessingConfig,
    l_shape_boundary,
    preprocess_boundary,
)

gate = preprocess_boundary(
    l_shape_boundary(),
    name="L",
    config=PreprocessingConfig(),
)
gate.save("nonconvex_timevarying_window/sc_dynatogt/results/L_gate")
```

保存目录包含 `manifest.json`、`geometry.npz` 和 `sc_map.npz`，均不使用 pickle。加载时会交叉核对安全多边形和 SC 参数。

对应命令行：

```bash
python -m nonconvex_timevarying_window.sc_dynatogt.preprocessing \
  --shape l_shape \
  --outdir nonconvex_timevarying_window/sc_dynatogt/results/L_gate

python -m nonconvex_timevarying_window.sc_dynatogt.preprocessing \
  --input gate_dense.csv \
  --corners gate_corners.csv \
  --outdir nonconvex_timevarying_window/sc_dynatogt/results/custom_gate
```

也可从曲线段或 CSV 进入：

```python
from nonconvex_timevarying_window.sc_dynatogt.preprocessing import (
    preprocess_csv,
    preprocess_segments,
)
```

支持的边界类型：折线、多段圆弧、任意阶 Bézier、B-spline、光滑闭曲线以及直线—曲线混合边界。输入必须是无洞、无自交的单个简单闭边界。

## 优化与实验

快速烟测 E0–E5：

```bash
python -m nonconvex_timevarying_window.sc_dynatogt.experiments \
  --suite smoke
```

论文方案的完整样本数：

```bash
python -m nonconvex_timevarying_window.sc_dynatogt.experiments \
  --suite default \
  --gif
```

完整套件执行：E1 的全部 6×5 个采样组合、E2 的 30 个种子、动态组的 155 次运行，以及每个被验证 SC map 的 `10^6` 个 `d ~ N(0,4I)` 样本。它是长时间实验，不是 CI 命令。

`smoke` 中 E3/E4/E5 只使用 1 个 L 形窗口和 1 次运行，用于验证调用链。`default` 中动态组使用 L、U、五角星 3 个窗口，并按指定顺序各穿越一次。如果只想先生成一份三窗口可视化样例，可以显式覆盖正式统计次数：

```bash
python -m nonconvex_timevarying_window.sc_dynatogt.experiments \
  --suite default --experiment E4 --replicates 1 --mapping-samples 1000 --gif \
  --outdir nonconvex_timevarying_window/sc_dynatogt/results/experiments/diagnostics/manual_e4
```

单独运行一组：

```bash
python -m nonconvex_timevarying_window.sc_dynatogt.experiments --suite smoke --experiment E4 --outdir /tmp/sc_e4
python -m nonconvex_timevarying_window.sc_dynatogt.experiments --suite default --experiment E5 --replicates 155 --outdir /tmp/sc_e5
```

实验组：

- E0：相同凸安全矩形、相同初值下，原 TOGT 凸组合映射与 SC 映射总时间误差不超过 1%；
- E1：L、U、五角星、limaçon、波浪和直线—Bézier 边界采样；
- E2：固定安全中心、凸包映射和 SC 非凸映射；
- E3：仅平移窗口；
- E4：平移、旋转和均匀缩放；
- E5：完整窗口时间梯度与令 `dp_i/dt_i=0` 的消融。

## 多形状、多窗口演示

E3–E5 的正式统计场景固定为 L、U、五角星 3 个窗口。E1 中的光滑和混合边界原先只做边界误差实验。独立的六窗口演示把下列形状接入同一条全动态轨迹：

```text
L -> U -> 五角星 -> limaçon -> 光滑波浪 -> 直线–Bézier 混合边界
```

```bash
python -m nonconvex_timevarying_window.sc_dynatogt.diverse_demo \
  --mode full --quality smoke --layout paper_irregular --motion-scale 3.5 \
  --validation-samples 1000
```

`paper_irregular` 是当前默认的大空间闭环布局。它参考原论文配套赛道
`race_uzh_7g_multiprisma.yaml` 的窗口位置、RPY 和 `Gate1 -> ... -> Gate7` 顺序，将六种非凸窗口依次放到 `Gate1,Gate2,Gate3,Gate4,Gate6,Gate7` 的放大位置；跳过 Gate5 是因为它与 Gate4 的 x/y 相同，仅高度不同。起点和终点均为 `[-16.0,4.0,3.2] m`，中心覆盖 `x=[-9.9,20.24] m`、`y=[-13.2,14.96] m`、`z=[1.8,6.48] m`，七段航程长短不一，为 `14.77--30.74 m`。`motion-scale=3.5` 对应缩放系数范围 `[0.58,1.42]`。

未显式指定 `--outdir` 时，命令自动写入 `results/demos/runs/<时间戳>_paper_irregular_full/`，成功后更新 `results/current_demo.json`。当前结果经过无损整理后位于：

- `results/demos/runs/20260717_paper_irregular_closed/`：当前精选闭环，包含数值数据、图、动画和 OpenGL；
- `results/demos/archive/spacious_open_20260717/`：旧开放三维实验；
- `results/demos/archive/regular_closed_20260717/`：规则闭环中间版本；
- 精选运行的 `legacy/`：原始轨迹图、GIF 和旧 OpenGL 成片，均保留原 SHA-256；
- `--layout compact --motion-scale 1.0`：仍可复现最早的开放共线小幅布局。

每个新演示运行统一包含 `summary.json`、`run_manifest.json`、`data/trajectory.csv`、`preprocessing/`、`figures/` 和 `media/`。`figures/route_overview.png` 只保留一个代表性四旋翼；`figures/crossings_grid.png` 用相同世界尺度并排显示六次穿越；`figures/scale_profile.png` 显示完整缩放曲线和穿越时刻。`media/dynamic_windows.gif` 仍按统一时间轴更新全部真实门框和四旋翼。安全区只保留在预处理诊断图和数值合法性验证中。

三类图的用途不同：`preprocessing.png` 是显示真实边界、内缩安全区和 SC 网格的算法诊断图；`figures/` 与 `media/` 是只画真实门框与四旋翼的 Matplotlib 结果；`opengl/` 则是带实体环境、光照和追踪相机的展示层。

### OpenGL 离线仿真画面

`simulation_render.py` 使用 EGL/OpenGL 读取同一条 MINCO 轨迹和真实动态窗口位姿，生成更接近仿真软件的画面。它包含带厚度的管状门框、四旋翼网格、建筑玻璃立面、道路、树木、太阳阴影、大气雾、追踪相机和飞行 HUD。这是基于已求解轨迹的离线三维渲染，不是 AirSim 动力学或传感器仿真。

```bash
python -m pip install -r nonconvex_timevarying_window/sc_dynatogt/requirements-render.txt
PYOPENGL_PLATFORM=egl python -m nonconvex_timevarying_window.sc_dynatogt.simulation_render \
  --summary nonconvex_timevarying_window/sc_dynatogt/results/demos/runs/20260717_paper_irregular_closed/summary.json
```

省略 `--summary` 时会读取 `results/current_demo.json`；省略 `--outdir` 时会写到该运行的 `opengl/`。`opengl_overview.png` 是全局航线图，`opengl_chase.png` 是追踪镜头定帧，`opengl_chase.mp4` 是 960×540、12 fps 的 H.264 追踪视频。完成后运行 manifest 和结果主页会自动刷新。

OpenGL 中每扇门的每帧节点变换都实际使用 `s(t)R(t)`。六扇门在穿越时刻的缩放依次为 `1.419, 0.874, 0.716, 1.378, 1.153, 0.611`，完整运动范围为 `[0.58,1.42]`。追踪视频仍不适合直接比较缩放，因此新增的固定尺度 `crossings_grid.png` 和 `scale_profile.png` 专门提供对照；固定距离 OpenGL `GATE CAM` 视频与实时 `SCALE ×` 仍未实现。

## 结果整理与主页

`results_manager.py` 管理本地结果目录。迁移命令默认只打印计划，只有加入 `--apply` 才移动文件；迁移清单记录全部原路径、目标路径、字节数和 SHA-256，可重复验证：

```bash
python -m nonconvex_timevarying_window.sc_dynatogt.results_manager migrate
python -m nonconvex_timevarying_window.sc_dynatogt.results_manager migrate --apply
python -m nonconvex_timevarying_window.sc_dynatogt.results_manager verify
python -m nonconvex_timevarying_window.sc_dynatogt.results_manager index
```

统一入口为 `results/index.html`；`results/INDEX.md` 适合纯文本浏览，`results/catalog.json` 供脚本读取。正式 E0–E5 位于 `results/experiments/formal/20260714_default/`，smoke、诊断、历史演示和 chunks 分别保存在自己的类别下。

Python 端不限于上述六种预置。`build_boundary_scenario(definitions, centers=..., angles=..., motion_scale=..., start=..., goal=...)` 可接收任意数量的有序 `DenseBoundary` 列表，并为每个窗口指定三维中心和 RPY 姿态，也可显式设置相同起终点构造闭环；未给出 `centers` 时才使用旧的 x 轴等距排布。折线、光滑曲线、直线–曲线混合边界和 CSV 稠密边界都使用相同的 Chang 采样、Clipper2 偏置、SC 取点和动态窗口链。仍然要求边界无洞、无自交，且安全偏置后保持单连通。

## 数值验证

完整 SC 合法性验证：

```bash
python -m nonconvex_timevarying_window.sc_dynatogt.validation \
  --map nonconvex_timevarying_window/sc_dynatogt/results/L_gate/sc_map.npz \
  --samples 1000000 \
  --output nonconvex_timevarying_window/sc_dynatogt/results/L_gate/validation.json
```

验证对象是完整映射 `q(d)=Psi(B(d))`；雅可比行列式同时包含 SC 部分与
`det(J_B)=(1+||d||^2)^-2`。梯度检查使用中心差分 `h=1e-6`。判据固定为：中位相对误差 `<1e-5`，99% 分位 `<1e-3`。`validation.py` 同时提供 `check_window_gradients` 和 `check_joint_objective_gradient`。

测试：

```bash
pytest -q nonconvex_timevarying_window/sc_dynatogt/tests
python -m compileall -q nonconvex_timevarying_window/sc_dynatogt
```

已执行的命令、通过数和端到端实验记录见 [TEST_RESULTS.md](TEST_RESULTS.md)。

## 输出

每个实验目录包含 JSON/CSV 数值结果。含轨迹的实验还会输出：

- `preprocessing/<index>_<name>/manifest.json|geometry.npz|sc_map.npz|preprocessing.png`：可重载离线产物和算法诊断图；
- `figures/route_overview.png|crossings_grid.png|scale_profile.png`：干净总览、固定尺度穿越图和缩放曲线；
- `data/trajectory.csv`：位置、速度、加速度、jerk、snap、crackle；
- `media/dynamic_windows.gif`：真实窗口与四旋翼动画；
- `opengl/opengl_overview.png|opengl_chase.png|opengl_chase.mp4`：可选 OpenGL 实体场景、追踪镜头和 H.264 视频；不包含安全区。

`algorithm_figures/` 中另外保存 5 幅面向阅读的中文算法、组件、梯度和实验结果图，具体说明见 [FIGURE_GUIDE.md](FIGURE_GUIDE.md)。

## 文件结构

```text
boundary.py       边界类型、稠密化、Chang 均匀采样、角点和误差
offset.py         Clipper2 0.315 m 安全偏置与拓扑检查
sc_mapping.py     圆盘 SC 参数、值、导数、逆映射和持久化
preprocessing.py  完整离线管线与 E1 边界目录
environment.py    平移/旋转/缩放窗口及空间、时间梯度
time_mapping.py   原 TOGT K<->T 映射和穿越时刻反传
minco.py          degree-7 minimum-snap
dynamics.py       四旋翼平坦性、约束积分和反向梯度
optimizer.py      x=[K,D] 联合目标与 L-BFGS-B
baselines.py      E0/E2 专用原凸映射、固定点和凸包基线
results_manager.py 结果迁移、manifest、索引和中文结果主页
validation.py     10^6 映射合法性与数值梯度检查
visualization.py  PNG、CSV、GIF
simulation_render.py EGL/OpenGL 仿真场景 PNG/MP4
explain_figures.py 中文算法/组件/结果图解
experiments.py    E0–E5 命令行协议
tests/            单元、梯度、回归和烟测
```

## 已知限制

- 只支持无洞、无自交、偏置后仍为单连通单分量的窗口；不支持重复穿越。
- 圆盘 SC 存在固有 crowding。极深狭槽或近退化边会被数值诊断明确拒绝，不会返回一个伪映射。
- `pyclipr` 对一条闭路径只提供一种 join type。没有真实凹角时，全局 Round 与凸角的向内 Miter 交点一致；存在真实凹角时使用全局 Miter，稠密光滑点成为高分辨率 miter 近似，产物 metadata 会明确标出该限制。
- 为与 TOGT 复现保持一致，速度、角速度和推力是平滑积分软惩罚，不是额外添加的硬约束。实验 JSON/CSV 因此另行记录 `constraint_extrema` 和 `sampled_dynamic_limits_satisfied`，后者是采样诊断，不是连续时间可行性证书。
- `B(d)` 的像是开单位圆盘。若时间最优解趋向窗口边界，优化后的 `|d|` 可很大；映射和解析梯度仍保持有限，但固定步长中心差分会在浮点饱和区丢失分辨率。因此联合梯度在文档统一初始化 `d=0` 处检查，窗口映射梯度另以随机 `d` 检查。
- `designated_order_legal` 严格检查规定穿越时刻的顺序、轨迹点与当时真实安全多边形；它不声称对所有连续时刻的额外窗口平面交点做障碍物认证。
- OpenGL 追踪镜头缺少固定尺寸参照，不能仅凭主观画面判断窗口是否缩放；应以 `MotionProfile.scale(t)` 和门框节点的 `s(t)R(t)` 变换为准。
- 完整 30/155 次实验和每门 `10^6` 样本验证计算量较大；烟测只验证调用链，不代替论文统计。

## 参考

- [Chang, Gotsman, Hormann, *Real-Time Conformal Maps and Parameterizations*](https://www.inf.usi.ch/hormann/papers/Chang.2026.RCM.pdf)：本实现只采用其“均匀边界重采样并可保留指定顶点”的预处理用途；
- `../../复现/论文/2309.06837v3.pdf`：TOGT 问题、时间变量、约束积分和门内取点消元；
- Driscoll, Trefethen, *Schwarz–Christoffel Mapping*：圆盘 SC 公式与预顶点参数问题。
