# NonConvex DynaTOGT：非凸动态时变窗口穿越实验

本项目是一个独立子项目，用于研究无人机穿越动态、非凸、无洞二维窗口区域的问题。它不修改 `togt_timevarying_window/` 的凸多边形版本，也不接入 `gap_step/` PPO 主线。

## 任务

窗口局部可通行区域记为：

```text
Omega_i(t) subset R^2
```

区域可以是非凸折线边界，也可以是光滑边界采样后的无洞简单多边形。无人机需要按顺序选择穿越时间 `t_i` 和窗口内部穿越点：

```text
p_i in Omega_i(t_i)
```

本项目使用三角剖分 atlas，把每个非凸区域拆成若干三角 chart。每个 chart 使用 softmax 重心坐标从 `R^2` 可微映射到三角形内部；所有 chart 的并集覆盖整个非凸窗口。

## 命令

快速实验：

```bash
python -m nonconvex_timevarying_window.experiments --suite smoke --outdir nonconvex_timevarying_window/results
```

完整实验：

```bash
python -m nonconvex_timevarying_window.experiments --suite default --outdir nonconvex_timevarying_window/results
```

## 方法

实验只运行一个方法：`AtlasDynaTOGT`。

它使用非凸三角剖分 atlas 表示窗口内部点，考虑窗口的动态平移、旋转、缩放，并用 L-BFGS-B 优化穿越时间和穿越点。

## 程序结构

```text
geometry.py      非凸几何、ear clipping、TriangleChart / ChartAtlas
environment.py   动态窗口、场景和穿越顺序
optimizer.py     轨迹和 atlas 优化器
visualize.py     CSV / PNG / GIF 导出
experiments.py   批量实验和 summary.csv
```

主要调用链是：

```text
geometry -> environment -> optimizer -> experiments
                         -> visualize
```

## 文献取舍

- Mean Value Coordinates：说明任意平面多边形参数化的背景，但全局内部性保证实现较重，本项目不直接采用。
- Polygon Triangulation / ear clipping：无洞简单多边形可以三角剖分，适合无新增依赖的实验版本。
- FRep / SDF：适合表达更复杂隐式形状；本项目当前主算法使用 atlas 保证 chart 内部性。

## 输出

```text
nonconvex_timevarying_window/results/<suite>/
  summary.csv
  trajectories/*.csv
  figures/*.png
  gifs/*.gif
```

CSV 的 `crossing` 行包含：

- `chart_id`：穿越点来自哪个三角 chart；
- `local_u/local_v`：窗口局部坐标；
- `plane_error`：点到窗口平面的距离；
- `boundary_margin`：点到非凸边界的有符号裕度；
- `chart_contains`：点是否位于对应 chart；
- `contains`：点是否位于真实动态非凸窗口。

## 测试

```bash
python -m py_compile nonconvex_timevarying_window/*.py
pytest -q nonconvex_timevarying_window/tests
```
