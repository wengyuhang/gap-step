# 窗口运动速度实验：在线三窗规划 vs 独立 GP-MPPI

2026-09-18。固定七窗口布局、幅度、x500 物理模型，扫描运动频率，不改变仿真播放速度。
协议见 [PROTOCOL.md](PROTOCOL.md)，独立文献实现及差异见 [BASELINE.md](BASELINE.md)。
论文代码未公开，用户同意独立复现；禁止将这里的 MPPI 成绩描述为作者官方结果。

## 最新：局部穿越约束修复

原算法以下负结果保留。新默认将全周期最坏速度下界替换为实际时刻的局部穿越条件，
详见 [LOCAL_SWEEP.md](../../receding_gate_racing/LOCAL_SWEEP.md)。

| 倍率 | 成功 | 平均完赛时间 | 最大完整规划周期 |
|---|---:|---:|---:|
| 1 | 3/3 | 33.344 s | 80.182 ms |
| 3 | 3/3 | 33.985 s | 75.314 ms |
| 6 | 3/3 | 33.263 s | 88.831 ms |
| 10 | 3/3 | 32.536 s | 79.921 ms |

12次均零门接触、真值采样球体净空非负（全局最小0.354877 m）、严格返航，
六侧工作区边界另行复核通过。安全/动力学仍是模型约束和采样证据，不是连续认证。
低频并未提速；主要修复是6/10倍不再被保守约束卡死。

[修复前后图](results/local_sweep_frozen_20260918/before_after.png)、
[实际速度](results/local_sweep_frozen_20260918/speed_profiles.png)、
[穿越位置](results/local_sweep_frozen_20260918/crossing_positions.png)。
文献独立复现已按用户要求在[复现/waypoint_mppi](../../../复现/waypoint_mppi/README.md)单独维护，
原比较目录中的GP实现保留作历史版本，不混入新实现的性能统计。

## 实验设计

4 档频率 × 3 个相位条件 × 2 个本地实现。相位是固定的 0、120、240° 增量，不是大量随机重复。
每一档均使用相同参数。保留失败和未完赛，未完赛不能按超时时间算“圈速”。

| 频率倍率 | 平移周期 | 旋转周期 | 中心最大速度约 | 最大角速度约 |
|---|---|---|---:|---:|
| 1 | 10–14.8 s | 8.5–12.7 s | 0.143 m/s | 8.94°/s |
| 3 | 3.33–4.93 s | 2.83–4.23 s | 0.428 m/s | 26.81°/s |
| 6 | 1.67–2.47 s | 1.42–2.12 s | 0.855 m/s | 53.61°/s |
| 10 | 1–1.48 s | 0.85–1.27 s | 1.425 m/s | 89.35°/s |

速度峰值来自解析运动函数的数值扫描，不是门框点速度。门框点还包含旋转引起的速度。
平移单轴幅度仍为0.10–0.20 m，姿态单轴幅度仍为5–10°。这是小幅高频测试，不代表大幅运动测试。

## 原算法冻结批次

| 倍率 | 物理/实时成功 | 成功圈平均时间 | 成功圈平均实际速度 | 最大完整规划周期 |
|---|---|---:|---:|---:|
| 1 | 3/3、3/3 | 32.296 s | 6.920 m/s | 78.039 ms |
| 3 | 3/3、3/3 | 32.556 s | 7.190 m/s | 76.186 ms |
| 6 | 0/3、0/3 | — | — | 89.447 ms |
| 10 | 0/3、0/3 | — | — | 3.560 ms |

来源：[逐次汇总](results/frozen_20260918/summary.json)。所有运行零门框接触，但6倍、10倍各三次
没有接受任何规划、没有过第一门，悬停直到60 s上限。这六次全部是失败，不是“安全成功”。
10倍快速返回失败，是约束下界/上界冲突等不可接受结果，不代表高效完成规划。
当前保守法向条件在第一门要求约5.12/8.00/12.23/17.83 m/s，随运动频率增长非常快。
6倍的失败只能表述为预算内未找到可执行解；10倍的首门法向下界已高于代码上界。
这暴露算法约束保守性，不能推断真实物理赛道不可飞。

## 图

- [速度—时间曲线](results/frozen_20260918/speed_profiles.png)：固定选相位0，标记实际过门时间；失败时的悬停也保留。
- [窗口局部穿越位置](results/frozen_20260918/crossing_positions.png)：黑线是真实开口，点是机心；不是只用世界坐标投影。
- [实际轨迹按速度着色](results/frozen_20260918/trajectory_speed.png)：圆圈表示过门点，星号为起终点。
- [频率与成功率/圈速/耗时](results/frozen_20260918/frequency_summary.png)：圈速只画成功结果，失败不填0秒。

## 文献对照当前未完成

已完成原算法12次正式扫描；独立 MPPI 尚未建立可靠的 PX4 闭环，不能作公平的竞速排名。
原推力变化率实现7次集成试飞均失败，停止批次后又检查了电机遥测单位，并测试直接 CTBR
预测/执行适配与更长时域，仍未完赛。详见 [复现差异与失败记录](BASELINE.md)。
`independent_comparison.png` 等历史比较图只用于接口诊断，**不是论文性能结论**。
下一步应先验证预测与真实内环的匹配，而非继续批跑一个未验证的控制器。

## 复现

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run --no-capture-output -n wyh \
 python -m convex_timevarying_window.comparisons.motion_rate_racing.run --outdir NEW_OURS_DIRECTORY
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=4 conda run --no-capture-output -n wyh \
 python -m convex_timevarying_window.comparisons.motion_rate_racing.run --method gp --outdir NEW_GP_DIRECTORY
conda run -n wyh python -m convex_timevarying_window.comparisons.motion_rate_racing.summarize NEW_OURS_DIRECTORY --gp NEW_GP_DIRECTORY
conda run -n wyh python -m convex_timevarying_window.comparisons.motion_rate_racing.plot NEW_OURS_DIRECTORY
```

两组必须顺序执行，不能共享同一个 PX4/Gazebo 容器并发运行。
测试：`conda run -n wyh pytest -q convex_timevarying_window/comparisons/motion_rate_racing/tests`。
构建需要 g++/OpenMP；编译发生在启动仿真前。参数、源码哈希、ULog与原始接触记录均保存在结果目录。
