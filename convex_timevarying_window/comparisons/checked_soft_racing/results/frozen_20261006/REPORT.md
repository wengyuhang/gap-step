# Checked-Soft：同设置历史对比

本次完成 33/38 条件；每条件一个固定参数有效尝试。历史结果只读，未重新运行旧方法。
新增项偏好机体净空 0.50 m，原 0.35 m 硬余量和 checked 准入不变。
主验收为七窗返航、零接触、真值采样净空/房间检查与完整周期 <100 ms；不是连续安全认证。

## 主矩阵与扩展矩阵

同一矩阵内同条件可比；不同矩阵不能直接比较成功率。partial 表示历史批次未完成完整矩阵，仅对已记录条件比较。有效圈时只统计实时合格圈，失败保留在分母。

| 矩阵 | 方法 | 实时合格/条件 | 合格圈时中位数 s | 合格净空中位数 m | 最大完整周期 ms |
|---|---|---:|---:|---:|---:|
| center24 | Checked-Soft (new) | 22/24 | 27.324 | 0.345 | 87.268 |
| center24 | Checked | 23/24 | 26.124 | 0.305 | 74.343 |
| center24 | Wang 2023 independent | 17/24 | 26.608 | 0.115 | 68.124 |
| center24 | Direct-CTBR MPPI | 22/24 | 35.844 | 0.365 | 79.884 |
| amplitude16 | Checked-Soft (new) | 14/15 | 30.484 | 0.257 | 77.005 |
| amplitude16 | Checked | 15/16 | 27.976 | 0.258 | 90.614 |
| amplitude16 | Local-sweep | 13/16 | 32.200 | 0.326 | 83.092 |
| amplitude16 | Wang 2023 independent | 8/16 | 28.448 | 0.152 | 68.473 |
| amplitude16 | Document V3 MPPI | 0/16 | — | — | 76.200 |
| amplitude_partial | Checked-Soft (new) | 10/10 | 26.688 | 0.315 | 77.005 |
| amplitude_partial | Continuation unchecked | 8/10 | 25.094 | 0.319 | 82.061 |
| rate12 | Checked-Soft (new) | 8/8 | 27.270 | 0.349 | 87.268 |
| rate12 | Local-sweep | 12/12 | 33.030 | 0.415 | 88.831 |
| rate12 | Worst-case crossing | 6/12 | 32.620 | 0.444 | 89.447 |
| rate_partial | Checked-Soft (new) | 5/5 | 26.600 | 0.314 | 87.268 |
| rate_partial | GP-MPPI independent | 0/7 | — | — | 25.414 |

注意：原幅度矩阵的 Checked 15/16 包含一条缺真值日志，不能说那一条已证明碰撞；
Continuation unchecked 只运行了部分矩阵，且含基础设施失败。表中是“证据通过数”，不是纯碰撞概率。

## 同条件、双方均合格的圈时差

负数表示新方法快。必须结合前表失败数看，不能仅用成功交集宣称整体更优。

| 矩阵 | 旧方法 | 双方合格数 | 新方法更快数 | 新减旧中位数 s | 净空差中位数 m | 仅新通过 / 仅旧通过 |
|---|---|---:|---:|---:|---:|---:|
| center24 | Checked | 21 | 4 | 1.164 | 0.047 | 1 / 2 |
| center24 | Wang 2023 independent | 16 | 6 | 0.624 | 0.283 | 6 / 1 |
| center24 | Direct-CTBR MPPI | 21 | 21 | -8.220 | -0.017 | 1 / 1 |
| amplitude16 | Checked | 13 | 4 | 0.996 | 0.024 | 1 / 1 |
| amplitude_partial | Continuation unchecked | 8 | 1 | 1.048 | 0.044 | 2 / 0 |
| amplitude16 | Local-sweep | 12 | 9 | -3.558 | -0.030 | 2 / 1 |
| amplitude16 | Wang 2023 independent | 7 | 3 | 0.940 | 0.169 | 7 / 1 |
| amplitude16 | Document V3 MPPI | 0 | 0 | — | — | 14 / 0 |
| rate12 | Local-sweep | 8 | 8 | -5.878 | -0.067 | 0 / 0 |
| rate12 | Worst-case crossing | 4 | 4 | -6.174 | -0.049 | 4 / 0 |
| rate_partial | GP-MPPI independent | 0 | 0 | — | — | 5 / 0 |

## 本次负结果

- r/t/a/phase=(10.0, 2.0, 1.0, 0.0)：未完成七窗返航；工作区超界；gates=0，failure=timeout，净空=14.689860822523176，周期=67.76992790400982 ms，审计错误=None。
- r/t/a/phase=(10.0, 4.0, 1.0, 2.094395102)：未完成七窗返航；Gazebo接触；工作区超界；真值采样整机净空为负；gates=4，failure=timeout，净空=-0.18738549667042415，周期=71.78046694025397 ms，审计错误=None。

## 共同真值动力学复核（独立于历史主验收）

统一使用原始约 20 ms 真值网格、姿态旋转的 0.24 m 机体偏移、数值差分；界为 12 m/s、8 m/s²、42°。
该诊断含差分误差，不是连续认证；原主口径没有强制实际飞行满足这些名义界。


## 同期旧Checked控制组与额外迁移条件

以下是单独的小样本诊断，不并入38条件，不挑选替代正式结果。

| 运行 | 主验收 | 有效圈时 s | 净空 m |
|---|---|---:|---:|

## 其他已比较方法与证据边界

- 三阶段 MPPI 的两批四条件 PX4 结果逐条保存在 comparison.json；原失败未删除。
- 后续 CTBR/safety-filter/flow-smooth/coupled 开发变体逐条列在 SPARSE_BASELINES.md，包括失败和重试，不挑最快版本代表整类方法。
- 2026 状态 RL 独立迁移与 waypoint MPPI 独立实现的已有 PX4 诊断列在 comparison.json。相位 0.4、r/t/a=1/1/1 可对照本次开发试飞；其他条件未冒充正式配对。学习方法尚未完成同任务迁移，不作为作者官方性能。
- 早期 GS-MPPI/三阶段名义模型与离线 TOGT 的名义时间不是 x500/PX4 有效圈时，不放入正式圈时排行。
- 旧 checked/新方法使用相同 PVA 与预览；direct CTBR 直接输出推力/角速度。它们是系统比较，不是只改变目标函数的消融。
- 历史跨日期单次测试受截止时间与负载影响；一两十分之一秒差异不能推断统计显著。

原始来源路径及 SHA-256、所有配对和负结果见 comparison.json。
