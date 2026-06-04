# DynaTOGT 实验说明

本文档说明本子项目如何验证 DynaTOGT 的有效性，以及每个实验结果应该怎么看。

## 1. 实验目的

本项目不是只做一个动画，而是要证明一个算法观点：

> 当窗口会移动、旋转、缩放或变形时，不能再把问题当作静态 gate 或中心点 waypoint；算法需要显式考虑 `G_i(t)`。

实验要回答几个问题：

1. DynaTOGT 能不能按指定顺序穿过动态窗口？
2. 如果窗口是动态的，静态 TOGT 还可靠吗？
3. 只穿窗口中心点是否足够好？
4. 只做离散搜索，不做连续优化，效果是否会变差？
5. 算法能否适应不同类型的动态变化？

## 2. 实验模式

当前支持三种模式。

### static

窗口固定不动，等价于较接近原论文的静态 gate 任务。

```text
p(t_i) in G_i
```

### ordered_dynamic

核心实验模式。窗口随时间变化，穿越顺序由用户给定。

```text
p(t_i) in G_i(t_i)
```

这里的 `--order` 是任务序列，可以重复窗口，例如：

```text
G1 -> G6 -> G1 -> G3 -> G2 -> G5 -> G4 -> G2
```

### shuffled_dynamic

扩展对照模式。算法自动搜索一个一次性穿越所有窗口的顺序。这个模式主要用于比较“顺序固定”和“顺序可变”的区别。

## 3. 实验场景

### smoke

快速测试用，只跑 canonical 场景和少量基线。用于确认代码、导出、测试流程都正常。

命令：

```bash
python -m togt_timevarying_window.experiments --suite smoke --outdir togt_timevarying_window/results
```

### default

主实验套件，包含多种动态情况。

场景包括：

```text
canonical_6
translation_only
rotation_only
scale_only
slow_dynamic
fast_dynamic
random_0 ... random_9
```

含义：

- `canonical_6`：默认六窗口场景，形状混合，运动/旋转/缩放同时存在；
- `translation_only`：只移动，不旋转不缩放；
- `rotation_only`：只旋转，不移动不缩放；
- `scale_only`：只缩放/形态变化；
- `slow_dynamic`：窗口变化幅度较小；
- `fast_dynamic`：窗口变化幅度较大；
- `random_0 ... random_9`：固定随机种子的非共线 3D 赛道，用于验证泛化。

命令：

```bash
python -m togt_timevarying_window.experiments --suite default --outdir togt_timevarying_window/results
```

## 4. 对比方法

实验中比较四种方法。

### WaypointCenter

始终穿过窗口中心。

它代表最简单的 waypoint 方法。缺点是没有利用窗口内部空间，也容易产生更激烈的轨迹。

### StaticTOGT

按静态窗口 `G_i(0)` 做规划，然后拿到真实动态窗口 `G_i(t)` 上验证。

它用来说明：如果窗口真的在动，忽略动态会导致穿越失败。

### DiscreteDynamic

只做动态窗口内的离散搜索，不做连续优化。

它用来说明：仅靠采样可以得到可行解，但通常轨迹质量不如连续优化。

### DynaTOGT

本文算法。先离散搜索得到初值，再连续优化穿越时间和窗口内部穿越点。

## 5. 输出目录

实验输出结构：

```text
togt_timevarying_window/results/<suite>/summary.csv
togt_timevarying_window/results/<suite>/trajectories/*.csv
togt_timevarying_window/results/<suite>/figures/*.png
togt_timevarying_window/results/<suite>/gifs/*.gif
```

其中：

- `summary.csv`：所有场景和方法的指标汇总；
- `trajectories/*.csv`：每条轨迹的穿越点和密集采样；
- `figures/*.png`：静态示意图；
- `gifs/*.gif`：动态演示。

## 6. 指标说明

`summary.csv` 包含以下列：

```text
scenario
baseline
mode
success
order
duration
path_length
total_cost
min_gate_margin
max_speed
max_acceleration
mean_jerk
optimization_time
```

通俗解释：

- `success`：是否所有指定窗口都成功穿越；
- `order`：实际穿越顺序；
- `duration`：总飞行时间；
- `path_length`：轨迹长度；
- `total_cost`：算法目标函数值；
- `min_gate_margin`：所有穿越中最小的窗口安全裕度；
- `max_speed`：最大速度；
- `max_acceleration`：最大加速度；
- `mean_jerk`：平均 jerk，越小轨迹越平滑；
- `optimization_time`：规划耗时。

## 7. 如何判断真的穿过窗口

每个轨迹 CSV 里都有 `crossing` 行。例如：

```text
section,index,name,t,x,y,z,...,plane_error,gate_margin,contains
crossing,1,G1,1.310433,...,0.000000000,0.172733375,True
```

判断标准：

- `contains=True`：穿越点在窗口内部；
- `plane_error` 接近 0：穿越点在窗口平面上；
- `gate_margin > 0`：穿越点距离边界有正裕度，没有贴边撞框。

GIF 中也会在对应时刻显示：

```text
Gx 穿越成功
裕度=...
```

这些是组会展示时最直接的证据。

## 8. 期望实验现象

正常情况下应看到：

1. `DynaTOGT` 在 canonical 和动态消融场景中成功率高；
2. `StaticTOGT` 在动态窗口评估中容易失败，因为它忽略了窗口随时间变化；
3. `WaypointCenter` 虽然有时能穿过，但轨迹代价和速度/加速度可能更差；
4. `DiscreteDynamic` 通常可行，但优化质量不如 DynaTOGT；
5. 在 GIF 中，绿色点对应的穿越时刻会显示“穿越成功”。

## 9. 与原论文实验的区别

原论文主要证明：

```text
考虑 gate 几何形状，比只走 waypoint 更优。
```

本项目主要证明：

```text
当 gate/window 会随时间变化时，算法必须考虑 G_i(t)，
否则静态规划可能在真实动态窗口上失败。
```

两者关系：

- 原论文是静态 gate TOGT；
- 本项目是动态时变窗口 DynaTOGT；
- 本项目继承“不要只走中心点，要利用窗口几何空间”的思想；
- 本项目新增“窗口状态随时间变化，因此时间和几何约束耦合”的问题。
