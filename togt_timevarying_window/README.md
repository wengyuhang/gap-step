# TOGT-TimeVarying-Window

这是一个独立的 3D 新项目原型，不依赖旧的 `gap_step` 迷宫环境。环境采用论文 TOGT 的 race track / ordered gates 抽象，并在其上扩展动态门：门/窗口的三维位置、姿态和形状尺度可以随时间变化。

## 对应论文思想

论文 TOGT 的核心是：不要把 gate 简化成中心 waypoint，而是在 gate 的几何可行域里选择穿越点，并联合时间分配优化轨迹。

本原型改成动态版本：

```text
原 TOGT:      p(t_i) in G_i
动态版本:     p(t_i) in G_i(t_i)
```

其中 `G_i(t)` 是第 `i` 个门在时间 `t` 的三维平面多边形/近似圆形可行域。

## 文件

```text
togt_timevarying_window/
  geometry.py      几何变换、多边形采样、包含测试
  environment.py   论文式动态 gate/racetrack 环境
  planner.py       按门序的 TOGT 风格时间扩展规划器
  demo.py          可运行示例
  export_demo.py   导出 3D 轨迹 CSV、PNG 和 GIF
```

## 运行

```bash
python -m togt_timevarying_window.demo
python -m togt_timevarying_window.demo --static
python -m togt_timevarying_window.export_demo
python -m togt_timevarying_window.export_demo --static
```

输出包含赛道名、是否动态、lap time、路径长度和每个 gate 的穿越时间。`export_demo` 还会生成 `outputs/*_trajectory.csv`、`outputs/*_trajectory.png` 和 `outputs/*_trajectory.gif`。

## 当前算法

当前实现是轻量可复现实验原型：

- 对每个动态 gate 在候选到达时间采样内部穿越点；
- 在 `(gate 序号, 到达时间, 候选点)` 上做 A* / dynamic programming；
- 约束最大速度和 gate order；
- 代价包含总时间和转向平滑项；
- 支持 rectangle、triangle、pentagon、hexagon、ball 近似。

它不是原 C++ TOGT 的 MINCO/L-BFGS 完整重写。下一步可以把当前离散解作为 warm start，再加入连续优化变量：每个 gate 的穿越点、穿越时间、以及动态 gate 形变参数下的可行域约束。

## 验证

```bash
pytest -q togt_timevarying_window/tests
python -m py_compile togt_timevarying_window/*.py
```

当前本地结果：`3 passed`，动态 demo 输出约 31.4s 的 12-gate 复杂 3D 轨迹，静态对照约 28.6s；`export_demo` 已生成动态/静态 CSV、PNG 与 GIF。
