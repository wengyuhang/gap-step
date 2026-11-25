# IPOPT软约束_第一个正式结果

本目录归档用户指定的2026-11-09正式结果`higher_swing_10x5_full_v1_20261109/PLANNING.md`，以及生成该结果的完整冻结源码、所有规划候选、首次实飞和随后三条补飞记录。

## 直接查看

- [原始规划表](PLANNING.md)
- [首次50趟实飞表](FLIGHT_首次.md)
- [随后三条补飞完成后的实飞表](FLIGHT_补飞后.md)
- [原正式批报告](records/higher_swing_10x5_full_v1_20261109/REPORT.md)
- [后续补飞报告](records/higher_swing_incomplete_completion_v1_20261109/REPORT.md)
- [本次归档完整性验证](provenance/VALIDATION.json)

三个表均附同名JSON、CSV。时间覆盖七窗与返航。

## 完整代码

`code/`保存原正式批`source_snapshot`的2558个冻结文件，包含当时的规划、基线、动力学辅助、MPC、PX4/Gazebo执行接口、模型、窗口网格和运行相关文件。全部冻结文件与原协议的SHA-256一致。

当时的规划实现采用Python/CasADi与IPOPT，包含C++数值与执行辅助。轨迹使用当时的全PVAJ表达式，优化器坐标为按频率缩放的累计到达时刻及MINCO基准导数偏移，八个独立自由点与八个中心初值后再运行最佳中心释放分支。源文件按当时内容归档。

| 入口 | 归档内路径 |
|---|---|
| 正式矩阵与调度 | `code/convex_timevarying_window/comparisons/论文实验/实验2/higher_swing/matrix.py` |
| 单候选规划 | `code/convex_timevarying_window/comparisons/论文实验/实验2/higher_swing/matrix_worker.py` |
| 本文规划器与初始化 | `code/convex_timevarying_window/comparisons/论文实验/实验2/soft_dynamics/` |
| 有限IPOPT问题 | `code/convex_timevarying_window/comparisons/论文实验/实验2/soft_dynamics/fast_ipopt.py` |
| MINCO、动力学与几何表达式 | `code/convex_timevarying_window/comparisons/论文实验/实验2/soft_dynamics/kernels.py` |
| 原安全交换循环 | `code/convex_timevarying_window/global_gate_minco_probe/planner.py` |
| 赛道运动 | `code/convex_timevarying_window/comparisons/论文实验/实验2/higher_swing/scenario.py` |
| 实飞流水 | `code/convex_timevarying_window/comparisons/论文实验/实验2/higher_swing/matrix_flight.py` |
| MPC/PX4执行 | `code/convex_timevarying_window/comparisons/论文实验/实验2/radius_ablation/execution.py` |
| MPC参考与跟踪 | `code/convex_timevarying_window/comparisons/论文实验/实验2/gazebo_matrix/reference.py` |
| 飞控通信与状态对齐 | `code/convex_timevarying_window/togt_style_mpc/aligned_session.py` |
| x500底层模型与数值辅助 | `code/convex_dynamic_seven_window_gazebo/x500_togt/` |
| Wang独立复现 | `code/复现/wang2023_verified/` |

启动时需要的`higher_swing/MATRIX_PROTOCOL.md`从原正式批`PROTOCOL.md`复制补齐，来源和SHA见`provenance/EXTRA_RELATED_FILES.json`。原规划环境为conda `wyh`，原Gazebo/PX4镜像标识和控制参数见原`protocol.json`及冻结源码。归档中的源码根目录为`code/`。

## 规划与实飞原始记录

`records/higher_swing_10x5_full_v1_20261109/`保存原正式批全部记录，源码集中在上面的`code/`。

- `candidates/`含340个候选任务的初值、逐轮优化状态和结果。
- `plans/`含50个最终参考、几何评估和诊断。
- `flights/`含50趟首次实飞及启动相关记录，保留ULog、八路接触、窗口位姿、真值、遥测、MPC记录、世界文件和参考轨迹。
- 原协议、统计、异常、压缩清单和历史验证均保留原文件。

`records/higher_swing_incomplete_completion_v1_20261109/`保存后来对`mixed_base/ours`、`a50_f1/ours`、`a50_f6/dynamic_togt_center`三项中止的补飞。该目录含原`run_frozen.py`、CPU分组和状态观测协议、三趟原始证据与更新后的实飞表。两批目录保留原名称，补飞脚本中的相对基础目录关系可对应。

首次执行的中止记录和补飞结果均保留。碰撞、中止、地面接触及末端状态沿用原表和原字段。补飞后的表有三格采用新增记录，其余47格沿用首次记录。

## 快速定位0.75／6自由点

- 选中规划为`records/higher_swing_10x5_full_v1_20261109/plans/a75_f6_ours/result.json`，完整参考19.172615343秒。
- 成功规划来源为同批`candidates/a75_f6/plans/ours_5/`。
- 该批对应实飞位于`records/higher_swing_10x5_full_v1_20261109/flights/a75_f6_ours/attempt_1/`，完整实际时钟19.179秒，原记录为七窗通过、窗框与地面无接触。

## 归档校验

本次核对2558个冻结源码文件、50份规划来源SHA、53趟实飞的参考系数和时长对应关系，以及583个原生压缩档案的SHA，全部通过。逐文件复制也通过SHA一致性检查。

- `provenance/MANIFEST.json`记录6780个原始文件的原位置、归档位置、大小和SHA。
- `provenance/FLIGHT_REFERENCE_CHECKS.json`记录50趟首次执行与3趟补飞的轨迹对应关系。
- `provenance/NATIVE_ARCHIVE_CHECKS.json`记录583个原生压缩档案的校验。
- `provenance/PATH_MAP.json`映射记录内的原始绝对路径。
- `PACKAGE_SHA256.txt`用于在本目录执行`sha256sum -c PACKAGE_SHA256.txt`。

本次验收针对归档完整性与原记录对应关系。原有规划数值证据、优化终止状态和实际接触证据随原始文件保存。
