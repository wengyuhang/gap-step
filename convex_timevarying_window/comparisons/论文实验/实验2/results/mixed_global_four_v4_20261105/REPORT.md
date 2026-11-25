# 实验2：四方法全局七窗混合运动完整对比

状态：完成，36/36条记录。

三赛道×静态/1Hz2m/3Hz4m×四全局方法，各一次初相零。每条均4个平移窗、3个绕外支点摆动窗。
全部一次规划七窗和返航，无停车中间点、无滚动视域或执行中重规划。Wang不在本轮对比，旧记录独立保留。
完整竞速从执行开始到七窗全部整机退出、返航和减速；赛前规划耗时另表。只有实际无碰撞完赛才填有效时间。

|赛道|运动等级|本文自由点|本文中心|动态TOGT自由点|动态TOGT中心|
|---|---|---|---|---|---|
| original | static | 27.705 s · 无碰撞 | 未找到合格规划 | 碰撞 | 未找到合格规划 |
| original | medium | 未找到合格规划 | 未找到合格规划 | 碰撞 | 碰撞 |
| original | high | 52.307 s · 无碰撞 | 52.328 s · 无碰撞 | 碰撞 | 碰撞 |
| slalom | static | 15.641 s · 无碰撞 | 16.243 s · 无碰撞 | 碰撞 | 未找到合格规划 |
| slalom | medium | 未找到合格规划 | 15.917 s · 无碰撞 | 碰撞 | 碰撞 |
| slalom | high | 32.713 s · 无碰撞 | 40.416 s · 无碰撞 | 碰撞 | 碰撞 |
| height | static | 未找到合格规划 | 未找到合格规划 | 15.887 s · 无碰撞 | 未找到合格规划 |
| height | medium | 未找到合格规划 | 未找到合格规划 | 16.999 s · 无碰撞 | 17.853 s · 无碰撞 |
| height | high | 23.667 s · 无碰撞 | 29.257 s · 无碰撞 | 碰撞 | 碰撞 |

## 完整规划及计算成本

|赛道/等级|方法|结果|全局四初值求解s|名义轨迹s|在线规划请求|MPC最大ms|原始记录|
|---|---|---|---:|---:|---:|---:|---|
| original/static | 本文自由点 | 无碰撞完赛 | 184.777 | 28.226 | 0 | 44.146 | [原始记录](../mixed_global_four_v3_20261105/original_static_ours/result.json) |
| original/static | 本文中心 | 未找到合格规划 | 180.523 | — | 0 | — | [原始记录](../mixed_global_v1_20261105/plans/original_static_center/result.json) |
| original/static | 动态TOGT自由点 | 碰撞 | 0.247 | 35.599 | 0 | 61.020 | [原始记录](../mixed_global_four_v3_20261105/original_static_dynamic_togt/result.json) |
| original/static | 动态TOGT中心 | 未找到合格规划 | 0.149 | — | 0 | — | [原始记录](../mixed_global_four_v3_20261105/plans/original_static_dynamic_togt_center/result.json) |
| original/medium | 本文自由点 | 未找到合格规划 | 110.717 | — | 0 | — | [原始记录](../mixed_global_v1_20261105/plans/original_medium_ours/result.json) |
| original/medium | 本文中心 | 未找到合格规划 | 59.803 | — | 0 | — | [原始记录](../mixed_global_v1_20261105/plans/original_medium_center/result.json) |
| original/medium | 动态TOGT自由点 | 碰撞 | 1.085 | 36.051 | 0 | 61.793 | [原始记录](../mixed_global_four_v3_20261105/original_medium_dynamic_togt/result.json) |
| original/medium | 动态TOGT中心 | 碰撞 | 0.236 | 38.533 | 0 | 61.589 | [原始记录](../mixed_global_four_v3_20261105/original_medium_dynamic_togt_center/result.json) |
| original/high | 本文自由点 | 无碰撞完赛 | 219.344 | 52.789 | 0 | 46.696 | [原始记录](original_high_ours/result.json) |
| original/high | 本文中心 | 无碰撞完赛 | 67.867 | 52.807 | 0 | 42.431 | [原始记录](original_high_center/result.json) |
| original/high | 动态TOGT自由点 | 碰撞 | 0.329 | 41.951 | 0 | 61.632 | [原始记录](original_high_dynamic_togt/result.json) |
| original/high | 动态TOGT中心 | 碰撞 | 0.184 | 42.178 | 0 | 62.479 | [原始记录](original_high_dynamic_togt_center/result.json) |
| slalom/static | 本文自由点 | 无碰撞完赛 | 60.965 | 16.072 | 0 | 40.184 | [原始记录](slalom_static_ours/result.json) |
| slalom/static | 本文中心 | 无碰撞完赛 | 44.917 | 16.687 | 0 | 43.902 | [原始记录](slalom_static_center/result.json) |
| slalom/static | 动态TOGT自由点 | 碰撞 | 0.272 | 19.218 | 0 | 62.869 | [原始记录](slalom_static_dynamic_togt/result.json) |
| slalom/static | 动态TOGT中心 | 未找到合格规划 | 0.161 | — | 0 | — | [原始记录](plans/slalom_static_dynamic_togt_center/result.json) |
| slalom/medium | 本文自由点 | 未找到合格规划 | 282.320 | — | 0 | — | [原始记录](plans/slalom_medium_ours/result.json) |
| slalom/medium | 本文中心 | 无碰撞完赛 | 198.226 | 16.368 | 0 | 44.116 | [原始记录](slalom_medium_center/result.json) |
| slalom/medium | 动态TOGT自由点 | 碰撞 | 0.529 | 18.641 | 0 | 61.688 | [原始记录](slalom_medium_dynamic_togt/result.json) |
| slalom/medium | 动态TOGT中心 | 碰撞 | 0.270 | 19.000 | 0 | 61.284 | [原始记录](slalom_medium_dynamic_togt_center/result.json) |
| slalom/high | 本文自由点 | 无碰撞完赛 | 156.950 | 33.150 | 0 | 40.545 | [原始记录](slalom_high_ours/result.json) |
| slalom/high | 本文中心 | 无碰撞完赛 | 61.680 | 40.866 | 0 | 49.677 | [原始记录](slalom_high_center/result.json) |
| slalom/high | 动态TOGT自由点 | 碰撞 | 1.159 | 19.299 | 0 | 61.037 | [原始记录](slalom_high_dynamic_togt/result.json) |
| slalom/high | 动态TOGT中心 | 碰撞 | 0.168 | 21.935 | 0 | 61.517 | [原始记录](slalom_high_dynamic_togt_center/result.json) |
| height/static | 本文自由点 | 未找到合格规划 | 254.624 | — | 0 | — | [原始记录](plans/height_static_ours/result.json) |
| height/static | 本文中心 | 未找到合格规划 | 164.880 | — | 0 | — | [原始记录](plans/height_static_center/result.json) |
| height/static | 动态TOGT自由点 | 无碰撞完赛 | 0.330 | 16.386 | 0 | 40.991 | [原始记录](height_static_dynamic_togt/result.json) |
| height/static | 动态TOGT中心 | 未找到合格规划 | 0.175 | — | 0 | — | [原始记录](plans/height_static_dynamic_togt_center/result.json) |
| height/medium | 本文自由点 | 未找到合格规划 | 180.876 | — | 0 | — | [原始记录](plans/height_medium_ours/result.json) |
| height/medium | 本文中心 | 未找到合格规划 | 35.142 | — | 0 | — | [原始记录](plans/height_medium_center/result.json) |
| height/medium | 动态TOGT自由点 | 无碰撞完赛 | 1.108 | 17.521 | 0 | 41.128 | [原始记录](height_medium_dynamic_togt/result.json) |
| height/medium | 动态TOGT中心 | 无碰撞完赛 | 0.431 | 18.381 | 0 | 40.257 | [原始记录](height_medium_dynamic_togt_center/result.json) |
| height/high | 本文自由点 | 无碰撞完赛 | 261.840 | 24.118 | 0 | 39.629 | [原始记录](height_high_ours/result.json) |
| height/high | 本文中心 | 无碰撞完赛 | 92.453 | 29.669 | 0 | 41.149 | [原始记录](height_high_center/result.json) |
| height/high | 动态TOGT自由点 | 碰撞 | 0.292 | 21.136 | 0 | 60.648 | [原始记录](height_high_dynamic_togt/result.json) |
| height/high | 动态TOGT中心 | 碰撞 | 0.114 | 21.534 | 0 | 62.262 | [原始记录](height_high_dynamic_togt_center/result.json) |

## 方法与口径

- 本文自由点/中心：完整PVAJ八段曲线，时间目标，名义节点执行量硬约束和全部有限窗框局部浮点包络/见证交换；中心精确锁点。未找到合格规划不飞行，原样保留。
- 动态TOGT自由点/中心：发布版位置型MINCO-s4/C6，原生x500目标/伴随、发布版L-BFGS；p_i=c_i(t_i)+R_i q_i，时刻和选点联合优化、完整时间链式梯度。无本文安全目标/轨迹安全过滤。
- TOGT给予机体半径+0.10m的过面内缩，四组中心空间初值/时间尺度一致，按原TOGT目标选最低收敛候选；本文选最短验收通过候选。二者原目标和约束处理不同，不能当成仅更换选点的消融。
- TOGT使用已有x500适配原生目标（二进制、C++包装源码均冻结）：质量/惯量/推力适配平台，动力学罚权1000；不是静态作者原机参数。本文沿用既有90%规划推力配置。
- 接触以任务区间七窗框+地面20路1000Hz（按7窗+地面汇总）传感器为准；ULog独立核对七窗正向各一次、整机退出、返航终点。窗口真实pose/info与查询逐趟核对。
- 1ms物理与单次仿真观测不升级为闭环安全证书；各条件一次不估计随机成功概率。优化器收敛和名义过窗不等于实飞完成。
- 仅保留旧批次算法/几何未变的全局规划请求，来源见reuse_audit.json；旧实飞接触覆盖不全均排除，本批完整记录所有框边。
- 仅双方无碰撞完成时比较时间，未完成/碰撞不赋有效圈时；planning_not_found不表示已证明无解。
