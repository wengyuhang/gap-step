# IPOPT独立收敛修复，2026-11-17

固定现有Split-S moderate、AlphaPilot moderate、AirSim freq4场景；沿用八条共同原始初始曲线。自由与中心独立求解，没有跨方法优化解初始化。不更改窗口、运动、车辆、0.20米规划球、0.15米碰撞球、PVAJ MINCO、TOGT时间映射、16区间积分、权重1/1000与严格有限成功/安全复查门槛。

首先在Split-S相同seed0比较直接目标旧L-BFGS/无缩放、直接目标精确Hessian/梯度缩放、辅助变量目标精确Hessian/梯度缩放。保留全部失败。随后三场景8+8独立候选测试辅助变量精确Hessian。duration_units是同一物理问题可逆变量单位的单独诊断，不能混合逐行最佳结果。

辅助变量表达为s>=r(x)、s>=0，目标T+1000积分phi(s)。phi为原同一个单调平滑L1，消去s后等于原T+1000积分phi(r(x))，没有新增物理约束或惩罚项。精确Hessian由CasADi通过实际PVAJ MINCO解析导数链获得。

本批旧协调器SUMMARY中finite_solver仍硬编码SLSQP，已在源码修复；原始每候选result.json/solver日志实际为IPOPT。后续报告以原始候选为准，不改写原始输出。计算时间包含构图、有限求解、安全复查等的实际候选墙钟；并行时间不累加。

单候选达到上限或不可行不补点。数值几何通过另用mesh_check检查实际球与门框、顺序和终点。只规划，不实飞。不声称所有初值能收敛或局部解保证自由点更快。

## 后续诊断及中止

增加辅助变量线性目标penalty_epigraph、mu_max=1e-4、L-BFGS历史50、原目标无缩放/monotone障碍策略对照。penalty_epigraph在活跃罚下与直接目标值/曲线相同，但无稳定改善，源码仅存诊断快照，已从现行入口移除。

duration_units的三项连续1000余轮零步长恢复被中止，单列INTERRUPTED.json。旧公式AirSim八初值中14项返回Restoration_Failed，2项仍在运行时发现原姿态退化分支问题而中止；不是16项全部求解到结束。

发现动态恢复在1+b3z=0.001处切换到不同公式。改成分母稳定恒等式并去掉替代物理模型的fallback，普通非奇异区公式不变；原退化分支区域的模型行为改变，单独记录chart_fixed_probe与final_regression，不能把前面的结果当成修正后结果。精确反向推力极点及零推力仍是恢复公式的真实定义域奇点。本次没有添加倾角硬约束或罚项。
