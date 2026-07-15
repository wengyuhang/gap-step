# SC-DynaTOGT 验证记录

验证日期：2026-07-14（Asia/Shanghai）。所有命令均在仓库根目录、`wyh` Conda 环境中执行。

## 单元、梯度与回归测试

```bash
pytest -q nonconvex_timevarying_window/sc_dynatogt/tests
```

结果：`90 passed in 30.75s`。覆盖边界稠密化和 Chang 采样、角点保留、Clipper2 偏置、SC 参数求解/映射/持久化、degree-7 MINCO、四旋翼平坦性、完整 `[K,D]` 梯度、动态窗口、任意有序边界列表的多窗口场景、显式三维窗口位姿与运动倍率、原始物理边界与安全区同位姿可视化、实验 CLI、PNG/CSV/GIF 输出和中文图解生成。

```bash
python -m compileall -q nonconvex_timevarying_window/sc_dynatogt
pytest -q
```

结果：`compileall` 通过；仓库根 `pytest.ini` 指定的 `gap_step/tests` 回归为 `46 passed in 3.31s`。新方法的 85 项测试需按上一条命令单独执行，与已有 Atlas 方法的组织方式一致。

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

E3/E4 的三个窗口映射均为 `1,000,000/1,000,000` 点合法。正式三窗口 PNG/GIF 保存在 `results/default/E3|E4|E5/`；黑色虚线为原始物理边界，彩色区为内缩安全区。

## 六形状六窗口动态演示

```bash
python -m nonconvex_timevarying_window.sc_dynatogt.diverse_demo \
  --mode full --quality smoke --layout spacious --motion-scale 2.5 \
  --validation-samples 1000 \
  --outdir nonconvex_timevarying_window/sc_dynatogt/results/diverse_demo
```

演示按 `L -> U -> 五角星 -> limaçon -> 光滑波浪 -> 直线–Bézier` 顺序穿越 6 个同时平移、旋转、缩放的窗口。新默认使用三维稀疏布局：中心覆盖 `x=[-10.0,10.2] m`、`y=[-3.5,3.6] m`、`z=[1.5,5.8] m`，运动振幅为正式 E3–E5 的 2.5 倍。

本次实跑结果：优化收敛，6/6 穿越点指定顺序合法，六个 SC 映射各 `1000/1000` 点合法，总时间 `4.70937 s`，1060 次迭代。输出的 `trajectory.png`、`trajectory.csv`、`dynamic_windows.gif`、六份预处理产物和 `summary.json` 均非空。这条轨迹的 `sampled_dynamic_limits_satisfied=false`：强运动长距离演示通过了窗口合法性，但未满足全部采样动力学硬上限，不将软惩罚收敛冒充为硬可行。

这是功能和可视化演示，不改动 E0–E5 的正式统计定义。

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

`python -m pip check` 还报告现有 Conda 环境中与本方法无关的 `conda-repo-cli`/`gensim`/`anaconda-cloud-auth` 可选包冲突；上述直接依赖均已安装并完成实际运行。

## 已知限制

- 原 TOGT 使用平滑积分软惩罚，不是硬动力学约束。smoke 中主 SC 动态轨迹的单旋翼推力采样峰值为 E3 `5.11529 N`、E4 `5.00594 N`，高于 `5.0 N` 上限；代码如实输出 `sampled_dynamic_limits_satisfied=false`，不将软惩罚收敛冒充为硬可行性。
- `B(d)` 将有限变量映射到开圆盘。当时间最优解趋向窗口边界时，优化后 `|d|` 可达数百；映射与解析梯度仍有限，但不适合在该浮点饱和点上用固定 `h=1e-6` 做中心差分。联合梯度因此在方案指定的有限初始化 `d=0` 处检查，动态窗口梯度则使用随机 `d` 独立检查。
- default 中 E3/E4 各有 4/2 个种子未返回收敛标志，E5 两种方法各有 2 个；总体收敛率仍高于方案的 90% 门槛，且所有输出穿越点均通过真非凸安全区检查。
- 其余几何范围与 Clipper2 join 限制见 `README.md` 的“已知限制”。
