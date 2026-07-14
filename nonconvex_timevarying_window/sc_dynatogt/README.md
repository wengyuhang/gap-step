# SC-DynaTOGT：Schwarz–Christoffel 非凸时变窗口

本目录是 `nonconvex_timevarying_window/` 下的独立算法实现。实现严格分开三件事：

1. Chang、Gotsman、Hormann 的方法只用于边界累计弧长均匀重采样和真实角点保留；
2. 安全区域内部点只由圆盘 Schwarz–Christoffel（SC）映射生成；
3. TOGT 使用原正时间变量、degree-7 MINCO、四旋翼微分平坦性约束和 L-BFGS 联合优化穿越点与时间。

这里没有使用 Chang 论文的 harmonic measure、Poisson kernel 或区域间参数化，也没有复用 `atlas_dynatogt/` 的三角 chart。

## 先看图

如果公式和源码较抽象，先看 [FIGURE_GUIDE.md](FIGURE_GUIDE.md)。它用 5 幅中文图依次解释整个算法、文件分工、动态梯度、现有预处理/轨迹图以及 E0–E5 结果。

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
  --suite smoke \
  --outdir nonconvex_timevarying_window/sc_dynatogt/results/smoke
```

论文方案的完整样本数：

```bash
python -m nonconvex_timevarying_window.sc_dynatogt.experiments \
  --suite default \
  --outdir nonconvex_timevarying_window/sc_dynatogt/results/default
```

完整套件执行：E1 的全部 6×5 个采样组合、E2 的 30 个种子、动态组的 155 次运行，以及每个被验证 SC map 的 `10^6` 个 `d ~ N(0,4I)` 样本。它是长时间实验，不是 CI 命令。

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

- `preprocessed_gates/<index>_<name>/manifest.json|geometry.npz|sc_map.npz`：每个窗口的可重载离线产物；
- `preprocessed_gates/<index>_<name>/preprocessing.png`：稠密边界、Chang 采样、角点、安全偏置和 SC 网格；
- `trajectory.png`：三维 MINCO 轨迹和穿越时刻窗口；
- `trajectory.csv`：位置、速度、加速度、jerk、snap、crackle；
- `dynamic_windows.gif`：传入 `--gif` 时生成的动态窗口动画。

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
validation.py     10^6 映射合法性与数值梯度检查
visualization.py  PNG、CSV、GIF
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
- 完整 30/155 次实验和每门 `10^6` 样本验证计算量较大；烟测只验证调用链，不代替论文统计。

## 参考

- [Chang, Gotsman, Hormann, *Real-Time Conformal Maps and Parameterizations*](https://www.inf.usi.ch/hormann/papers/Chang.2026.RCM.pdf)：本实现只采用其“均匀边界重采样并可保留指定顶点”的预处理用途；
- `../../复现/论文/2309.06837v3.pdf`：TOGT 问题、时间变量、约束积分和门内取点消元；
- Driscoll, Trefethen, *Schwarz–Christoffel Mapping*：圆盘 SC 公式与预顶点参数问题。
