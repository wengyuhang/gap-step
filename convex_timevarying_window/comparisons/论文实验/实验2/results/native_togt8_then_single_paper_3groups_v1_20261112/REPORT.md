# 动态TOGT中心八初值后，本文各单初值

三个原条件分别执行八路动态TOGT中心，选其原目标最低的求解成功候选，然后将该候选的段时间、中心穿越点和实际VAJ交给本文自由点与中心点，各执行一个初值。本文两方法并行2路。

## 设置

TOGT八初值直接使用前批存档的独立过窗时间和相位样本，MINCO从相应动态中心和段时间生成内部VAJ。优化器为作者原C++L-BFGS，past=3、delta=1e-5、g_epsilon=0、max_iterations=0，原权重与16区间积分。几何复查作为记录，选初值按TOGT原目标排序。

本文沿用直接几何坐标，圆窗rho/theta区间约束，多边形xy半空间。采用独立完整PVAJ MincoPV(order=3)，时间/软动力学权重1/1000，33节点积分，x500限值95%，安全约束与复查半径0.62594472/0.62094472m，节点正向速度下界0.1m/s。段时间仅由TOGT正映射保证正值，工作空间与段时长范围已移除。单个有限NLP失败结束该初值，安全补点后接续原始变量与乘子，未进行本文中心释放。

## 初值生成

| 条件 | 选中编号 | TOGT完整轨迹 / s | 八路计算墙钟 / s | 初值几何复查 |
|---|---|---:|---:|---|
| 0.25／6 | 5 | 20.527392 | 0.495774 | nominal_safe_complete |
| 0.5／6 | 7 | 20.164725 | 0.502768 | nominal_collision |
| 0.75／6 | 7 | 21.174130 | 0.509242 | nominal_collision |

编号采用0至7。初值的碰撞记录与本文最终结果分开列示。

## 本文结果

完整时间覆盖起点、七窗和全部返航。计算时间包含共享TOGT八路墙钟、本方法初值转换和本方法进程，两个并行方法的耗时独立记录。几何复查耗时另存各assessment.json。

| 条件 | 本文自由点 / s | 本文中心点 / s | 自由计算 / s | 中心计算 / s | 自由／中心有限优化次数 |
|---|---:|---:|---:|---:|---|
| 0.25／6 | **16.450442** | 17.356459 | 56.091713 | 19.678031 | 8／1 |
| 0.5／6 | 未找到合格规划 | 18.309501 | 212.057603 | 85.484791 | 7／8 |
| 0.75／6 | 未找到合格规划 | 未找到合格规划 | 22.353994 | 99.261984 | 1／1 |

有限优化次数表示同一初值的补点求解轮数。本文每方法每条件均只有一个初值。本批只做规划和独立实体几何复查，Gazebo实飞尚未进行。

## 求解状态

| 条件 | 方法 | 每轮迭代数 | 每轮状态 |
|---|---|---|---|
| 0.25／6 | 自由 | 701, 182, 161, 98, 212, 177, 214, 148 | Solve_Succeeded, Solve_Succeeded, Solve_Succeeded, Solve_Succeeded, Solve_Succeeded, Solve_Succeeded, Solve_Succeeded, Solve_Succeeded |
| 0.25／6 | 中心 | 741 | Solve_Succeeded |
| 0.5／6 | 自由 | 1314, 506, 674, 335, 215, 429, 5000 | Solve_Succeeded, Solve_Succeeded, Solve_Succeeded, Solve_Succeeded, Solve_Succeeded, Solve_Succeeded, Maximum_Iterations_Exceeded |
| 0.5／6 | 中心 | 882, 441, 462, 297, 154, 264, 125, 361 | Solve_Succeeded, Solve_Succeeded, Solve_Succeeded, Solve_Succeeded, Solve_Succeeded, Solve_Succeeded, Solve_Succeeded, Solve_Succeeded |
| 0.75／6 | 自由 | 1039 | Infeasible_Problem_Detected |
| 0.75／6 | 中心 | 5000 | Maximum_Iterations_Exceeded |

## 完整性核对

三组24个TOGT候选均已记录，选中者逐项核对为原TOGT目标最小的求解成功候选。八个输入时间来自原存档，相应正时间参数回代误差不超过1e-14秒。六个本文任务均只有一个初值，双方T与VAJ逐项相同，MincoPV初值重建系数最大误差见VALIDATION.json。

0.25/6两种本文方法均合格；0.5/6中心合格，自由在第7轮有限NLP达到5000次上限；0.75/6自由首轮返回Infeasible_Problem_Detected，中心首轮达到5000次上限。所有失败后均结束该初值。已找到轨迹全部通过独立窗框、顺序及端点复查。源码、输入与冻结二进制SHA核对通过。
