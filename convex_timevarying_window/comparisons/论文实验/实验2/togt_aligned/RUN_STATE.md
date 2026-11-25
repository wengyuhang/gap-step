# 当前五方法机体碰撞口径已更正并完成

2026-11-15。0.30米球用于规划安全裕度，物理碰撞用现有QuadA仿真碰撞体半径0.15米。50条既有完整曲线已全部C++重查，正式表results/body_collision_recheck_20261115/PLANNING.md。窗框/任务通过数依次本文自由10、中心10、动态TOGT自由7、中心7、Wang9。地面另记。原始轨迹与求解墙钟保留，详情five_method_safety030/RUN_STATE.md。下文为历史阶段记录。

# 最新TOGT裕度实验已完成

2026-11-15，当前统一选点内缩每侧0.30m，机体球半径0.15m，中心/自由每档各单初值。自由点7/10档通过，中心7/10，静态自由8.789s。另测0.40m自由5/10档通过。结果results/author_source_inset_comparison_20261115/REPORT.md，参数dynamic_togt_from_source/config/quada_inset030.json。全部进程已结束，旧结果保留。

# 最新动态TOGT源码复测已完成

2026-11-15，正常动态TOGT入口改为作者TrajSolver最小动态扩展。十档20条已结束，中心7通过/3碰撞，自由10碰撞。静态自由8.638437s与作者未修改静态源码逐值一致，但当前整机几何检查碰第1/2/5门。结果results/author_source_rebuild_20261115/REPORT.md。新代码dynamic_togt_from_source/README.md。只规划，原结果保留。

# 最新静态航点验证已完成

2026-11-15，用户要求第7处恢复作者原SingleBall。静态四方法已完成，16个本文独立初值全部通过。最新结果results/point7_static_20261115/REPORT.md，输入results/inputs_point7_static_20261115。协调PID42354、matrix和全部求解进程已结束。本文自由7.356s、中心8.066s，TOGT双方10.011s。本文中心地面相交单列。下文保留上一批十档实体门的完成记录。

# 当前运行状态

2026-11-14：四方法十档规划已全部完成，40/40行、160/160独立初值审计通过。协调PID2794369和全部matrix/sip_planner/TOGT工作进程已退出，未启动Gazebo竞速。不要再次启动已完成批次。

输入：results/inputs_v3_mesh512_20261114；输出：results/planning_v3_mesh512_20261114。
最终表：FINAL_PLANNING.md；数据：FINAL_SUMMARY.json；初值审计：FINAL_CANDIDATE_AUDIT.json；源码核对：FROZEN_SOURCE_CHECK.json。原始SUMMARY/PLANNING保留，两个明确任务失败已从证据不足更正为未完成，记录在REPORT_CLASSIFICATION_CORRECTIONS.json。

本文自由点通过8/10、中心5/10；TOGT自由10碰撞、中心8碰撞/2未完成。自由点与中心均通过的五档，自由点快9.38%～11.00%。这里的通过是名义窗框、门序、端点检查；地面另列，动力学为软惩罚，不是Gazebo成绩。

第7门由原混合七门SingleBall改为方框，末段返回原作者终点（第6门中心），存在折返；静态TOGT也发生出第7门后的碰框，不可据此把全部失败归因于高速动态运动。160候选中87找到合格轨迹、70因no_new_witness停止、3因有限求解未收敛停止；大量任务重复过窗问题尚未解决，未改冻结算法修补。
