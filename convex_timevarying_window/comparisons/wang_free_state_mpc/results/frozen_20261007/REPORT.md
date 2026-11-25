# Wang-MINCO 自由状态 / MPC 事件版：38条件完整实验

全部 38/38 条件已执行，共 38 趟；首尝试主验收 32/38，允许设施重试后 32/38。
飞行算法与event_v4开发版63项源码逐项一致，整个矩阵固定参数，不调参、不重试算法失败。
本批全部重新飞行，不将开发两趟复用为正式数据。x500/PX4/Gazebo真实动力学闭环，不是轨迹动画。

## 如何算成功

旧口径：七窗返航、零门框接触、10ms真值整机球净空非负、房间界、所有规划/MPC/发送调用<100ms。
本次还要求：七窗各一次正向穿越、顺序正确、整机离面且不在有限窗附近返穿。历史对照用相同事件审计重新只读复核；旧标签单列，原文件不改。
12m/s、8m/s²、42°名义超限依用户要求只作诊断。采样/接触证据不是连续安全证明，每条件一次也不是随机总体成功概率。
本批旧口径通过 32/38；增加事件条件后 32/38。
合格圈时中位数 25.102 s，范围 24.408–43.392 s；最小合格净空 0.2862 m。
所有尝试中规划/MPC/发送最大值 74.23/34.00/34.40 ms。

## 与已有方法对照

| 矩阵 / 历史方法 | 旧标签成功 | 加事件审计后 | 本次对应条件成功 | 双方合格 | 本次更快 | 本次减旧中位s |
|---|---:|---:|---:|---:|---:|---:|
| center24 / Checked | 23/24 | 20/24 | 23/24 | 19 | 16 | -0.776 |
| center24 / Wang 2023 independent | 17/24 | 0/24 | 23/24 | 0 | 0 | — |
| center24 / Direct-CTBR MPPI | 22/24 | 22/24 | 23/24 | 21 | 21 | -10.728 |
| amplitude16 / Checked | 15/16 | 11/16 | 11/16 | 8 | 5 | -0.968 |
| amplitude_partial / Continuation unchecked | 8/10 | 8/10 | 9/10 | 7 | 3 | 0.236 |
| amplitude16 / Local-sweep | 13/16 | 11/16 | 11/16 | 9 | 7 | -6.576 |
| amplitude16 / Wang 2023 independent | 8/16 | 2/16 | 11/16 | 1 | 0 | 4.836 |
| amplitude16 / Document V3 MPPI | 0/16 | 0/16 | 11/16 | 0 | 0 | — |
| rate12 / Local-sweep | 12/12 | 12/12 | 12/12 | 12 | 12 | -8.096 |
| rate12 / Worst-case crossing | 6/12 | 6/12 | 12/12 | 6 | 6 | -7.838 |
| rate_partial / GP-MPPI independent | 0/7 | 0/7 | 7/7 | 0 | 0 | — |
| union38 / Checked-Soft / optimized MPC | 38/38 | 32/38 | 32/38 | 26 | 23 | -0.970 |
| soft_partial / Checked-Soft / PVA | 31/33 | 23/33 | 28/33 | 18 | 17 | -1.664 |
| mpc_partial / MPC before optimization | 9/10 | 7/10 | 9/10 | 6 | 6 | -5.268 |
| development2 / TOGT-MINCO-s4 / MPC | 2/2 | 1/2 | 2/2 | 1 | 1 | -6.096 |
| development2 / Wang free-state v1 / MPC | 2/2 | 1/2 | 2/2 | 1 | 1 | -0.468 |
| development2 / Wang center-state v1 / MPC | 1/2 | 0/2 | 2/2 | 0 | 0 | — |

历史共读取264个不同运行目录；10趟无法完整复核事件（字段缺失、真值覆盖或日志缺失）。它们不算已证实返穿；本批这些缺证据趟在旧口径下也未成功。

### 旧验收口径的计时追溯（不是严格各窗一次的竞速排名）

| 矩阵 / 历史方法 | 同条件双方旧口径合格 | 本次更快 | 本次减旧中位s |
|---|---:|---:|---:|
| center24 / Checked | 22 | 18 | -0.764 |
| center24 / Wang 2023 independent | 16 | 15 | -1.294 |
| center24 / Direct-CTBR MPPI | 21 | 21 | -10.728 |
| amplitude16 / Checked | 10 | 7 | -1.110 |
| amplitude_partial / Continuation unchecked | 7 | 3 | 0.236 |
| amplitude16 / Local-sweep | 10 | 7 | -5.898 |
| amplitude16 / Wang 2023 independent | 7 | 4 | -0.660 |
| amplitude16 / Document V3 MPPI | 0 | 0 | — |
| rate12 / Local-sweep | 12 | 12 | -8.096 |
| rate12 / Worst-case crossing | 6 | 6 | -7.838 |
| rate_partial / GP-MPPI independent | 0 | 0 | — |
| union38 / Checked-Soft / optimized MPC | 32 | 29 | -1.068 |
| soft_partial / Checked-Soft / PVA | 26 | 24 | -1.834 |
| mpc_partial / MPC before optimization | 8 | 8 | -5.268 |
| development2 / TOGT-MINCO-s4 / MPC | 2 | 2 | -6.984 |
| development2 / Wang free-state v1 / MPC | 2 | 2 | -2.298 |
| development2 / Wang center-state v1 / MPC | 1 | 1 | -4.960 |

负数表示本次更快。只比较完全相同条件和双方都合格的圈时，同时保留全部失败分母。
中心24、幅度16、频率12存在重叠；不能把它们加起来当成独立样本。development2仅两个开发条件，不与全矩阵等量齐观。
历史独立复现及动态扩展不是原作者官方性能。状态RL未完成匹配动态场景PX4迁移，无可比有效圈速；离线TOGT名义24.6608s不是安全闭环成绩。
历史试验并非同期随机重复，毫秒级差异不能当作显著提速；两个已用于开发的条件不构成独立留出。

## 本次分矩阵

| 矩阵 | 首尝试成功 | 允许设施重试后 | 合格圈时中位s |
|---|---:|---:|---:|
| center24 | 23/24 | 23/24 | 24.956 |
| amplitude16 | 11/16 | 11/16 | 26.596 |
| rate12 | 12/12 | 12/12 | 24.922 |

## 全部38个条件

| r/t/a/相位编号 | 尝试 | 主验收 | 合格圈时s | 采样净空m | 规划/MPC最大ms | 失败原因 |
|---|---:|---|---:|---:|---|---|
| 1/1/1/0 | 1 | 通过 | 24.408 | 0.3786 | 69.20/22.88 |  |
| 1/1/1/1 | 1 | 通过 | 24.780 | 0.3842 | 69.32/24.28 |  |
| 1/2/1/0 | 1 | 失败 | — | 0.4395 | 68.71/27.07 | timeout; not_exactly_one_ordered_forward_passage_per_gate; incomplete_body_exit |
| 1/2/1/1 | 1 | 通过 | 24.712 | 0.4241 | 72.94/30.13 |  |
| 1/4/1/0 | 1 | 通过 | 24.800 | 0.3497 | 72.32/25.96 |  |
| 1/4/1/1 | 1 | 通过 | 24.856 | 0.3454 | 70.98/25.44 |  |
| 3/1/1/0 | 1 | 通过 | 24.724 | 0.4125 | 68.01/26.02 |  |
| 3/1/1/1 | 1 | 通过 | 24.432 | 0.3726 | 71.73/31.22 |  |
| 3/2/1/0 | 1 | 通过 | 24.960 | 0.4015 | 70.57/26.12 |  |
| 3/2/1/1 | 1 | 通过 | 24.956 | 0.3206 | 69.70/24.15 |  |
| 3/4/1/0 | 1 | 通过 | 24.788 | 0.4018 | 71.12/24.79 |  |
| 3/4/1/1 | 1 | 通过 | 24.420 | 0.3410 | 74.23/25.33 |  |
| 6/1/1/0 | 1 | 通过 | 24.552 | 0.3372 | 71.34/24.29 |  |
| 6/1/1/1 | 1 | 通过 | 25.444 | 0.3403 | 71.84/23.51 |  |
| 6/2/1/0 | 1 | 通过 | 24.816 | 0.3486 | 69.53/24.19 |  |
| 6/2/1/1 | 1 | 通过 | 25.696 | 0.3719 | 70.38/34.00 |  |
| 6/4/1/0 | 1 | 通过 | 26.436 | 0.3375 | 69.93/25.53 |  |
| 6/4/1/1 | 1 | 通过 | 28.220 | 0.3462 | 71.12/23.94 |  |
| 10/1/1/0 | 1 | 通过 | 25.900 | 0.4126 | 70.77/27.65 |  |
| 10/1/1/1 | 1 | 通过 | 26.596 | 0.4232 | 73.76/29.98 |  |
| 10/2/1/0 | 1 | 通过 | 28.816 | 0.4071 | 70.15/27.22 |  |
| 10/2/1/1 | 1 | 通过 | 28.756 | 0.3638 | 68.55/23.88 |  |
| 10/4/1/0 | 1 | 通过 | 32.464 | 0.2862 | 71.90/28.44 |  |
| 10/4/1/1 | 1 | 通过 | 34.476 | 0.2885 | 71.82/25.73 |  |
| 1/2/2/0 | 1 | 通过 | 25.016 | 0.3453 | 71.52/21.54 |  |
| 1/2/2/1 | 1 | 通过 | 25.496 | 0.3761 | 69.90/26.25 |  |
| 1/4/4/0 | 1 | 失败 | — | 0.3746 | 69.23/23.05 | RuntimeError('missed_current_opening'); not_exactly_one_ordered_forward_passage_per_gate; incomplete_body_exit |
| 1/4/4/1 | 1 | 通过 | 27.132 | 0.3792 | 68.86/26.40 |  |
| 10/2/2/0 | 1 | 通过 | 43.392 | 0.3228 | 71.83/26.75 |  |
| 10/2/2/1 | 1 | 通过 | 32.532 | 0.3517 | 72.37/29.86 |  |
| 10/4/4/0 | 1 | 失败 | — | 0.5571 | 72.85/20.52 | RuntimeError('missed_current_opening'); not_exactly_one_ordered_forward_passage_per_gate; incomplete_body_exit |
| 10/4/4/1 | 1 | 失败 | — | 0.5907 | 69.54/21.37 | RuntimeError('missed_current_opening'); not_exactly_one_ordered_forward_passage_per_gate; incomplete_body_exit |
| 10/1/4/0 | 1 | 失败 | — | 0.5093 | 72.15/21.13 | RuntimeError('missed_current_opening'); not_exactly_one_ordered_forward_passage_per_gate; incomplete_body_exit |
| 10/1/4/1 | 1 | 失败 | — | 0.5778 | 70.26/21.86 | RuntimeError('missed_current_opening'); not_exactly_one_ordered_forward_passage_per_gate; incomplete_body_exit |
| 1/1/1/2 | 1 | 通过 | 24.788 | 0.3531 | 68.06/23.32 |  |
| 3/1/1/2 | 1 | 通过 | 25.056 | 0.4005 | 68.85/26.30 |  |
| 6/1/1/2 | 1 | 通过 | 25.148 | 0.4105 | 71.18/27.57 |  |
| 10/1/1/2 | 1 | 通过 | 26.500 | 0.2954 | 71.98/24.39 |  |

r=运动频率倍数，t=平移幅度倍数，a=旋转幅度倍数；相位编号0/1/2为0/120/240度。
失败前运行时间不写成完成时间。详细失败阶段、接触、事件和ULog见各趟目录。

## 历史新增事件问题与缺证据

- center24 / Checked：旧合格但事件未通过3趟，其中缺证据0趟。
  - 条件(3.0, 2.0, 1.0, 0.0, 0.0)：returned_near_gate:3, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/amplitude_racing/results/center_motion_checked_20260923/r3_t2_a1_p0_attempt1`。
  - 条件(6.0, 4.0, 1.0, 2.094395102, 0.0)：returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/amplitude_racing/results/center_motion_checked_20260923/r6_t4_a1_p1_attempt1`。
  - 条件(10.0, 2.0, 1.0, 2.094395102, 0.0)：returned_near_gate:3, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/amplitude_racing/results/center_motion_checked_20260923/r10_t2_a1_p1_attempt1`。
- center24 / Wang 2023 independent：旧合格但事件未通过17趟，其中缺证据0趟。
  - 条件(1.0, 1.0, 1.0, 0.0, 0.0)：returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/center_motion_wang2023_20260923/case_00_r1_t1_a1_p0_attempt1`。
  - 条件(1.0, 1.0, 1.0, 2.094395102, 0.0)：returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/center_motion_wang2023_20260923/case_01_r1_t1_a1_p1_attempt1`。
  - 条件(1.0, 2.0, 1.0, 0.0, 0.0)：returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/center_motion_wang2023_20260923/case_02_r1_t2_a1_p0_attempt1`。
  - 条件(1.0, 4.0, 1.0, 0.0, 0.0)：returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/center_motion_wang2023_20260923/case_04_r1_t4_a1_p0_attempt1`。
  - 条件(3.0, 1.0, 1.0, 0.0, 0.0)：not_exactly_one_ordered_forward_passage_per_gate, incomplete_body_exit；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/center_motion_wang2023_20260923/case_06_r3_t1_a1_p0_attempt1`。
  - 条件(3.0, 1.0, 1.0, 2.094395102, 0.0)：returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/center_motion_wang2023_20260923/case_07_r3_t1_a1_p1_attempt1`。
  - 条件(3.0, 2.0, 1.0, 2.094395102, 0.0)：not_exactly_one_ordered_forward_passage_per_gate, incomplete_body_exit；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/center_motion_wang2023_20260923/case_09_r3_t2_a1_p1_attempt1`。
  - 条件(3.0, 4.0, 1.0, 0.0, 0.0)：returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/center_motion_wang2023_20260923/case_10_r3_t4_a1_p0_attempt1`。
  - 条件(3.0, 4.0, 1.0, 2.094395102, 0.0)：returned_near_gate:5, returned_near_gate:6, not_exactly_one_ordered_forward_passage_per_gate, incomplete_body_exit；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/center_motion_wang2023_20260923/case_11_r3_t4_a1_p1_attempt1`。
  - 条件(6.0, 1.0, 1.0, 0.0, 0.0)：returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/center_motion_wang2023_20260923/case_12_r6_t1_a1_p0_attempt1`。
  - 条件(6.0, 1.0, 1.0, 2.094395102, 0.0)：returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/center_motion_wang2023_20260923/case_13_r6_t1_a1_p1_attempt2`。
  - 条件(6.0, 2.0, 1.0, 2.094395102, 0.0)：returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/center_motion_wang2023_20260923/case_15_r6_t2_a1_p1_attempt1`。
  - 条件(6.0, 4.0, 1.0, 0.0, 0.0)：returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/center_motion_wang2023_20260923/case_16_r6_t4_a1_p0_attempt1`。
  - 条件(6.0, 4.0, 1.0, 2.094395102, 0.0)：returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/center_motion_wang2023_20260923/case_17_r6_t4_a1_p1_attempt1`。
  - 条件(10.0, 1.0, 1.0, 0.0, 0.0)：returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/center_motion_wang2023_20260923/case_18_r10_t1_a1_p0_attempt1`。
  - 条件(10.0, 1.0, 1.0, 2.094395102, 0.0)：returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/center_motion_wang2023_20260923/case_19_r10_t1_a1_p1_attempt1`。
  - 条件(10.0, 2.0, 1.0, 0.0, 0.0)：returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/center_motion_wang2023_20260923/case_20_r10_t2_a1_p0_attempt1`。
- amplitude16 / Checked：旧合格但事件未通过4趟，其中缺证据0趟。
  - 条件(1.0, 4.0, 4.0, 0.0, 0.0)：returned_near_gate:3, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/continuation_racing/results/frozen_checked_20260919/r1_t4_a4_p0_attempt1`。
  - 条件(10.0, 1.0, 1.0, 2.094395102, 0.0)：returned_near_gate:3, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/continuation_racing/results/frozen_checked_20260919/r10_t1_a1_p1_attempt1`。
  - 条件(10.0, 4.0, 1.0, 2.094395102, 0.0)：returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/continuation_racing/results/frozen_checked_20260919/r10_t4_a1_p1_attempt1`。
  - 条件(10.0, 1.0, 4.0, 0.0, 0.0)：returned_near_gate:6；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/continuation_racing/results/frozen_checked_20260919/r10_t1_a4_p0_attempt1`。
- amplitude16 / Local-sweep：旧合格但事件未通过2趟，其中缺证据0趟。
  - 条件(10.0, 2.0, 2.0, 2.094395102, 0.0)：returned_near_gate:3, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/amplitude_racing/results/frozen_20260919/r10_t2_a2_p1_attempt1`。
  - 条件(10.0, 1.0, 4.0, 0.0, 0.0)：returned_near_gate:3, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/amplitude_racing/results/frozen_20260919/r10_t1_a4_p0_attempt1`。
- amplitude16 / Wang 2023 independent：旧合格但事件未通过6趟，其中缺证据0趟。
  - 条件(1.0, 1.0, 1.0, 0.0, 0.0)：returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/two_papers_20260919/polynomial_matrix/case_00_r1_t1_a1`。
  - 条件(1.0, 1.0, 1.0, 2.094395102, 0.0)：not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/two_papers_20260919/polynomial_matrix/case_01_r1_t1_a1`。
  - 条件(1.0, 2.0, 2.0, 2.094395102, 0.0)：returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/two_papers_20260919/polynomial_matrix/case_03_r1_t2_a2`。
  - 条件(10.0, 1.0, 1.0, 0.0, 0.0)：returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/two_papers_20260919/polynomial_matrix/case_06_r10_t1_a1`。
  - 条件(10.0, 1.0, 1.0, 2.094395102, 0.0)：returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/two_papers_20260919/polynomial_matrix/case_07_r10_t1_a1`。
  - 条件(10.0, 2.0, 2.0, 0.0, 0.0)：returned_near_gate:5, returned_near_gate:6, not_exactly_one_ordered_forward_passage_per_gate, incomplete_body_exit；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/paper_baselines_2026/results/two_papers_20260919/polynomial_matrix/case_08_r10_t2_a2`。
- union38 / Checked-Soft / optimized MPC：旧合格但事件未通过6趟，其中缺证据0趟。
  - 条件(3.0, 1.0, 1.0, 0.0, 0.0)：returned_near_gate:3, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/checked_soft_mpc_optimized/results/frozen_20261006/r3_t1_a1_p0_attempt1`。
  - 条件(3.0, 4.0, 1.0, 0.0, 0.0)：returned_near_gate:3, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/checked_soft_mpc_optimized/results/frozen_20261006/r3_t4_a1_p0_attempt1`。
  - 条件(6.0, 2.0, 1.0, 0.0, 0.0)：returned_near_gate:3, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/checked_soft_mpc_optimized/results/frozen_20261006/r6_t2_a1_p0_attempt1`。
  - 条件(6.0, 4.0, 1.0, 0.0, 0.0)：returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/checked_soft_mpc_optimized/results/frozen_20261006/r6_t4_a1_p0_attempt1`。
  - 条件(6.0, 4.0, 1.0, 2.094395102, 0.0)：returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/checked_soft_mpc_optimized/results/frozen_20261006/r6_t4_a1_p1_attempt1`。
  - 条件(10.0, 2.0, 2.0, 2.094395102, 0.0)：returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/checked_soft_mpc_optimized/results/frozen_20261006/r10_t2_a2_p1_attempt1`。
- soft_partial / Checked-Soft / PVA：旧合格但事件未通过8趟，其中缺证据0趟。
  - 条件(1.0, 1.0, 1.0, 2.094395102, 0.0)：returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/checked_soft_racing/results/frozen_20261006/r1_t1_a1_p1_attempt1`。
  - 条件(3.0, 1.0, 1.0, 0.0, 0.0)：returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/checked_soft_racing/results/frozen_20261006/r3_t1_a1_p0_attempt1`。
  - 条件(3.0, 2.0, 1.0, 0.0, 0.0)：returned_near_gate:3, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/checked_soft_racing/results/frozen_20261006/r3_t2_a1_p0_attempt1`。
  - 条件(6.0, 1.0, 1.0, 0.0, 0.0)：returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/checked_soft_racing/results/frozen_20261006/r6_t1_a1_p0_attempt1`。
  - 条件(6.0, 2.0, 1.0, 0.0, 0.0)：returned_near_gate:3, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/checked_soft_racing/results/frozen_20261006/r6_t2_a1_p0_attempt1`。
  - 条件(6.0, 4.0, 1.0, 2.094395102, 0.0)：returned_near_gate:3, returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/checked_soft_racing/results/frozen_20261006/r6_t4_a1_p1_attempt1`。
  - 条件(10.0, 2.0, 2.0, 0.0, 0.0)：returned_near_gate:3, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/checked_soft_racing/results/frozen_20261006/r10_t2_a2_p0_attempt1`。
  - 条件(10.0, 2.0, 2.0, 2.094395102, 0.0)：returned_near_gate:3, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/checked_soft_racing/results/frozen_20261006/r10_t2_a2_p1_attempt1`。
- mpc_partial / MPC before optimization：旧合格但事件未通过2趟，其中缺证据0趟。
  - 条件(1.0, 2.0, 1.0, 2.094395102, 0.0)：returned_near_gate:5, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/checked_soft_mpc_racing/results/frozen_20261006/r1_t2_a1_p1_attempt1`。
  - 条件(3.0, 2.0, 1.0, 0.0, 0.0)：returned_near_gate:3, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/checked_soft_mpc_racing/results/frozen_20261006/r3_t2_a1_p0_attempt1`。
- development2 / TOGT-MINCO-s4 / MPC：旧合格但事件未通过1趟，其中缺证据0趟。
  - 条件(10.0, 1.0, 1.0, 2.094395102, 0.0)：returned_near_gate:3, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/togt_minco_mpc/results/px4_v3_lbfgs_20261007/r10_t1_a1_p1_attempt1`。
- development2 / Wang free-state v1 / MPC：旧合格但事件未通过1趟，其中缺证据0趟。
  - 条件(10.0, 1.0, 1.0, 2.094395102, 0.0)：not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/wang_free_state_mpc/results/free_v1_20261007/r10_t1_a1_p1_attempt1`。
- development2 / Wang center-state v1 / MPC：旧合格但事件未通过1趟，其中缺证据0趟。
  - 条件(1.0, 1.0, 1.0, 0.0, 0.0)：returned_near_gate:6, not_exactly_one_ordered_forward_passage_per_gate；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/wang_free_state_mpc/results/center_v1_20261007/r1_t1_a1_p0_attempt1`。

## 保留全部失败和重试

- [1, 2, 1, 0] attempt1：timeout; not_exactly_one_ordered_forward_passage_per_gate; incomplete_body_exit；最后时间59.996 s；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/wang_free_state_mpc/results/frozen_20261007/r1_t2_a1_p0_attempt1`。
- [1, 4, 4, 0] attempt1：RuntimeError('missed_current_opening'); not_exactly_one_ordered_forward_passage_per_gate; incomplete_body_exit；最后时间4.808 s；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/wang_free_state_mpc/results/frozen_20261007/r1_t4_a4_p0_attempt1`。
- [10, 4, 4, 0] attempt1：RuntimeError('missed_current_opening'); not_exactly_one_ordered_forward_passage_per_gate; incomplete_body_exit；最后时间4.196 s；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/wang_free_state_mpc/results/frozen_20261007/r10_t4_a4_p0_attempt1`。
- [10, 4, 4, 1] attempt1：RuntimeError('missed_current_opening'); not_exactly_one_ordered_forward_passage_per_gate; incomplete_body_exit；最后时间3.892 s；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/wang_free_state_mpc/results/frozen_20261007/r10_t4_a4_p1_attempt1`。
- [10, 1, 4, 0] attempt1：RuntimeError('missed_current_opening'); not_exactly_one_ordered_forward_passage_per_gate; incomplete_body_exit；最后时间4.172 s；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/wang_free_state_mpc/results/frozen_20261007/r10_t1_a4_p0_attempt1`。
- [10, 1, 4, 1] attempt1：RuntimeError('missed_current_opening'); not_exactly_one_ordered_forward_passage_per_gate; incomplete_body_exit；最后时间3.876 s；原目录 `/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/wang_free_state_mpc/results/frozen_20261007/r10_t1_a4_p1_attempt1`。

### 失败日志中的直接证据

下列是日志观察，不等于证明赛道不可行，也不代替候选轨迹重放和独立消融。
- [1, 2, 1, 0]：已记中心过窗2个，接受/尝试规划=5/295；最后接受={'time': 2.5080000000000027, 'made_at': 2.5940000000000025, 'gate': 0, 'phase': 'approach', 'durations': [0.10551130589547801, 0.10551130589547801, 1.7074859889058198, 1.7074859889058198, 1.2721021082230268, 1.2721021082230268]}；拒绝计数={'geometry_or_dynamics': 23, 'non_single_passage:1': 1, 'non_single_passage:3': 266}；MPC失败调用=0。
- [1, 4, 4, 0]：已记中心过窗1个，接受/尝试规划=2/22；最后接受={'time': 2.5080000000000027, 'made_at': 2.5940000000000025, 'gate': 0, 'phase': 'approach', 'durations': [0.43010136842121566, 0.43010136842121566, 1.8157329096631918, 1.8157329096631918, 1.1533003537843596, 1.1533003537843596]}；拒绝计数={'geometry_or_dynamics': 20}；MPC失败调用=0。
  - 中止前最后记录的位置与目标窗几何：{'gate': 1, 'measurement_time': 4.795999999999999, 'position': [10.542910585179925, 4.497149790287949, 5.890596618652344], 'window_local_q': [3.87999071859615, -14.525691878213925], 'unoriented_signed_normal_coordinate': 0.015780092947562996, 'lateral_distance_m': 15.034961008170333, 'aperture_enclosing_radius_m': 1.35, 'inside_aperture': False, 'note': 'Last recorded estimated sample, possibly just before the terminating observation; not an exact reconstruction of the unlogged terminating call'}。
- [10, 4, 4, 0]：已记中心过窗1个，接受/尝试规划=4/18；最后接受={'time': 3.520000000000003, 'made_at': 3.606000000000003, 'gate': 0, 'phase': 'approach', 'durations': [0.1591132598035758, 0.1591132598035758, 2.108687336213781, 2.108687336213781, 1.1598293418350893, 1.1598293418350893]}；拒绝计数={'geometry_or_dynamics': 8, 'optimizer_residual': 1, 'delayed_current_passage': 5}；MPC失败调用=0。
  - 中止前最后记录的位置与目标窗几何：{'gate': 1, 'measurement_time': 4.176000000000002, 'position': [-0.5031469401437789, -2.4349725618958473, 6.902172317504883], 'window_local_q': [4.64098224454396, -26.07616780287036], 'unoriented_signed_normal_coordinate': 1.3360011564077652, 'lateral_distance_m': 26.48594426252583, 'aperture_enclosing_radius_m': 1.35, 'inside_aperture': False, 'note': 'Last recorded estimated sample, possibly just before the terminating observation; not an exact reconstruction of the unlogged terminating call'}。
- [10, 4, 4, 1]：已记中心过窗1个，接受/尝试规划=5/17；最后接受={'time': 3.3160000000000025, 'made_at': 3.4020000000000024, 'gate': 0, 'phase': 'approach', 'durations': [0.15428502759996296, 0.15428502759996296, 2.496772521069699, 2.496772521069699, 1.0790875648685123, 1.0790875648685123]}；拒绝计数={'geometry_or_dynamics': 4, 'delayed_current_passage': 8}；MPC失败调用=0。
  - 中止前最后记录的位置与目标窗几何：{'gate': 1, 'measurement_time': 3.868000000000002, 'position': [-0.968092923052609, -2.7065353328362107, 7.195403804779053], 'window_local_q': [4.781909150592721, -26.7673858050275], 'unoriented_signed_normal_coordinate': 1.4707633514291059, 'lateral_distance_m': 27.19116764612565, 'aperture_enclosing_radius_m': 1.35, 'inside_aperture': False, 'note': 'Last recorded estimated sample, possibly just before the terminating observation; not an exact reconstruction of the unlogged terminating call'}。
- [10, 1, 4, 0]：已记中心过窗1个，接受/尝试规划=5/19；最后接受={'time': 3.1159999999999997, 'made_at': 3.2019999999999995, 'gate': 0, 'phase': 'approach', 'durations': [0.10212707445954382, 0.10212707445954382, 2.5244473729027668, 2.5244473729027668, 1.3679834692357236, 1.3679834692357236]}；拒绝计数={'geometry_or_dynamics': 14}；MPC失败调用=0。
  - 中止前最后记录的位置与目标窗几何：{'gate': 1, 'measurement_time': 4.148000000000003, 'position': [2.5644357714336365, -3.04708144813776, 7.125038852691651], 'window_local_q': [4.8417651108094955, -24.751422557478445], 'unoriented_signed_normal_coordinate': 1.7758597357605717, 'lateral_distance_m': 25.220539407536567, 'aperture_enclosing_radius_m': 1.35, 'inside_aperture': False, 'note': 'Last recorded estimated sample, possibly just before the terminating observation; not an exact reconstruction of the unlogged terminating call'}。
- [10, 1, 4, 1]：已记中心过窗1个，接受/尝试规划=1/18；最后接受={'time': 0.08800000000000097, 'made_at': 0.1740000000000009, 'gate': 0, 'phase': 'approach', 'durations': [1.619634993294841, 1.619634993294841, 2.3567901773416944, 2.3567901773416944, 1.3170196369454494, 1.3170196369454494]}；拒绝计数={'geometry_or_dynamics': 17}；MPC失败调用=0。
  - 中止前最后记录的位置与目标窗几何：{'gate': 1, 'measurement_time': 3.852000000000004, 'position': [0.32174576818943024, -3.6202768310904503, 6.9070608520507815], 'window_local_q': [4.543055960509453, -26.808434084739464], 'unoriented_signed_normal_coordinate': 1.8613840056613151, 'lateral_distance_m': 27.19065088474605, 'aperture_enclosing_radius_m': 1.35, 'inside_aperture': False, 'note': 'Last recorded estimated sample, possibly just before the terminating observation; not an exact reconstruction of the unlogged terminating call'}。

## 图

![相同口径对比](center24_comparison.png)

![38条件完整结果](matrix_times.png)

图中的失败不显示为有效圈速；历史加事件复核标签不覆盖历史原标签。

## 可追溯文件

- `protocol.json` / `source_snapshot/`：参数、环境、65项编排与飞行源码快照。
- `summary.json` / `first_attempt_summary.json` / `condition_summary.json`：全部尝试、首尝试与按协议选择结果。
- `historical_passage_audits.json`：只读历史ULog事件复核、result/ULog SHA256。
- `historical_metadata_views/`：旧频率矩阵未保存幅度字段；依其固定幅度协议补1倍的分析输入副本，ULog链接原文件，原日志不改；初次字段缺失错误仍保存在审计历史。
- `baseline_manifest.json` / `historical_comparison_rows.json` / `comparison.json`：历史来源、逐项配对及摘要。
- 每趟 `passage_audit.json`、`result.json`、ULog、接触、遥测及求解日志。
