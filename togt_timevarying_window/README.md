# DynaTOGT：动态时变窗口穿越实验

本子项目是基于 TOGT 论文思想做的一个独立研究原型。它不依赖 `gap_step/` 主线 PPO，也不修改 `gap_step/` 代码。

原 TOGT 论文的核心观点是：无人机竞速不应该只把每个 gate 当作一个中心点 waypoint，而应该利用 gate 的几何空间，在 gate 内选择更合适的穿越点。原论文的静态约束可以写成：

```text
p(t_i) in G_i
```

这里 `G_i` 是第 `i` 个静态 gate 的几何区域。

本项目把任务改成动态时变窗口：

```text
ordered_dynamic:  p(t_i) in G_i(t_i)
shuffled_dynamic: p(t_k) in G_sigma(k)(t_k)
```

也就是说，窗口不再固定，而是会随时间移动、旋转、缩放或形变。无人机必须在正确的时间穿过当时的窗口区域。

## 这个项目解决什么问题

目标是展示一个新的任务场景：

- 窗口形状不同：矩形、圆形、三角形、五边形、六边形、斜四边形；
- 窗口会运动：位置随时间变化；
- 窗口会旋转：姿态随时间变化；
- 窗口会缩放/变形：可通行区域大小随时间变化；
- 一个窗口可以被指定穿越多次；
- 穿越顺序可以任意指定，不要求每个窗口只出现一次。

默认顺序是：

```text
G1 -> G6 -> G3 -> G2 -> G5 -> G4
```

重复穿越示例：

```text
G1 -> G6 -> G1 -> G3 -> G2 -> G5 -> G4 -> G2
```

## 和原 TOGT 论文的关系

| 对比项 | 原 TOGT 论文 | 本项目 DynaTOGT |
| --- | --- | --- |
| gate/window | 静态 gate `G_i` | 动态窗口 `G_i(t)` |
| 几何约束 | `p(t_i) in G_i` | `p(t_i) in G_i(t_i)` |
| 穿越点 | 在 gate 几何区域内优化 | 在当前时刻的动态窗口内优化 |
| 时间变量 | 优化每段飞行时间 | 优化每段飞行时间，窗口状态也随时间改变 |
| 轨迹表示 | MINCO 多项式轨迹 | 轻量 Hermite 多项式轨迹 |
| 优化方法 | 变量变换 + L-BFGS | 动态窗口局部变量映射 + L-BFGS-B |
| 动力学约束 | 更接近完整四旋翼约束 | 用速度、加速度、jerk 作为近似可飞性指标 |
| 任务顺序 | 通常给定 gate 序列 | 支持给定序列、重复窗口，也保留 shuffled 对照 |

简言之：原论文解决“如何更好地穿过静态 gate”；本项目解决“如何穿过会动、会转、会变形的动态窗口”。

## 算法概述

新算法叫 **DynaTOGT**。它不是完整复刻原论文的 MINCO/L-BFGS 系统，而是一个用于验证新任务场景的研究原型。

核心步骤：

1. 用户给定一个穿越序列，例如 `G1,G6,G1,G3,G2,G5,G4,G2`。
2. 对序列中每一次穿越，算法选择：
   - 穿越时间 `t_i`；
   - 窗口内部的局部穿越点。
3. 局部穿越点会被映射到当前动态窗口 `G_i(t_i)` 内，因此几何上天然满足窗口约束。
4. 用离散搜索得到初始解。
5. 用 `scipy.optimize.minimize(method="L-BFGS-B")` 连续优化时间和穿越点。
6. 用 Hermite 多项式生成连续无人机轨迹。
7. 导出 CSV/PNG/GIF，验证每次穿越是否成功。

目标函数大致包含：

```text
总时间
+ 路径长度
+ 最大加速度惩罚
+ jerk 平滑惩罚
+ 动态可行性惩罚
+ 窗口边界安全裕度惩罚
```

## 命令

单次演示：

```bash
python -m togt_timevarying_window.demo --scenario canonical --mode ordered_dynamic
```

导出 PNG/GIF/CSV：

```bash
python -m togt_timevarying_window.export_demo --scenario canonical --mode ordered_dynamic
```

重复穿越示例：

```bash
python -m togt_timevarying_window.export_demo \
  --scenario canonical \
  --mode ordered_dynamic \
  --order G1,G6,G1,G3,G2,G5,G4,G2 \
  --outdir togt_timevarying_window/results/repeated_demo
```

运行实验：

```bash
python -m togt_timevarying_window.experiments --suite smoke --outdir togt_timevarying_window/results
python -m togt_timevarying_window.experiments --suite default --outdir togt_timevarying_window/results
```

## 输出文件

```text
togt_timevarying_window/results/<suite>/summary.csv
togt_timevarying_window/results/<suite>/trajectories/*.csv
togt_timevarying_window/results/<suite>/figures/*.png
togt_timevarying_window/results/<suite>/gifs/*.gif
```

CSV 中有两类行：

- `crossing`：每次穿越窗口的时间、位置和验证信息；
- `sample`：无人机轨迹的密集采样点，包括位置、速度、加速度和 yaw。

每个 `crossing` 行还包含：

```text
local_u, local_v, plane_error, gate_margin, contains
```

含义：

- `contains=True`：该点确实在当前动态窗口内；
- `plane_error` 接近 0：点在窗口所在平面上；
- `gate_margin > 0`：点没有撞边框，数值越大离边界越远。

PNG/GIF 中的中文提示：

- `穿越成功`：当前帧是某个窗口的精确穿越时刻；
- `裕度`：穿越点到窗口边界的安全裕度；
- 绿色点：精确穿越点；
- 虚线窗口：过去/未来姿态；
- 实线窗口：当前姿态。

## 测试

```bash
pytest -q togt_timevarying_window/tests
python -m py_compile togt_timevarying_window/*.py
```
