# 动态单窗穿越实验总汇（2026-10-09）

## 1. 范围和主要结论

本汇总覆盖当前 `dynamic_passage_velocity_minco/results/` 下截至本次整理的全部67份 `result.json`：52趟PX4/Gazebo仿真飞行、15次离线开发或固定输入求解测试。不是全仓历史实验总汇，不包括此前七窗竞速、Wang复现和三倍窗口批次。

当前主算法为 **hard_v2 名义动力学硬约束版**。旧flight、repaired和hard_v1分开统计，不拼接为同一算法成绩。配对报告引用的历史自由点记录不重复计数。

- 低档8条件双方重新飞行，共16趟，全部无窗框接触完赛。低频3条件基本持平；后5条件自由点本次快17.9%～28.4%。
- 较高档9条件：自由点7成功、1碰撞、1未完成；中心版9条件均无初始可接纳规划、40秒超时。不能把求解失败解释为中心必撞或物理无解。
- 4 Hz／4 m中心版用户指定重测一次，仍无首解超时；自由版两次GUI现场飞行均成功，6.488和6.532秒。
- 当前hard_v2合计37趟：25无碰撞完赛、1碰撞、11未完成。这是非均匀条件、含指定重测的记录计数，**不是成功概率估计**。
- 自由点仍有真实碰撞，当前算法不能宣称保证闭环安全；节点动力学硬约束也不是连续域认证。

## 2. 共同实验设置和统计口径

| 项目 | 设置 |
|---|---|
| 场景 | 原小窗W1，单个矩形窗口；固定平面，仅中心面内水平正弦平移 |
| 开口 | 2.90 m × 2.30 m（局部顶点±1.45、±1.15 m） |
| 运动 | 实际幅度A米、频率f Hz，初相0；运动完全已知、外生变化 |
| 飞行器/飞控 | Gazebo x500、PX4，MPC闭环跟踪 |
| 起点 | 世界坐标(-16, 4, 3.2) m；实际起飞后状态有微小偏差 |
| 窗口锚点 | (-2.42, -3.52, 6.48) m |
| 终点 | (3.58, -3.52, 6.48) m，即窗后6 m；终点VAJ为零 |
| 完成规则 | 合法正向穿窗、整机离窗、到达原目标；位置容差0.55 m、速度低于1 m/s |
| 碰撞证据 | Gazebo实体窗框接触；不能用净空、优化收敛或出窗代替 |
| 时间 | 从竞速起点计时到完成终点，含等首解、穿窗、窗后飞行和减速；不含飞前编译和起飞准备 |
| 上限/重复 | 竞速上限40 s；批次各条件各一次；指定重测、GUI展示另列 |
| 对比差异 | 自由点允许窗口内二维选点；中心固定q0=q1=0，仍开放穿窗三维速度 |

这是**单窗到终点**实验，不返回起点，不能称七窗全程或返航成绩。世界模型可以含原赛道辅助命名，但实际只保留一个物理窗口。

hard_v2采用两段七次、三个几何节点的C5 minimum-snap变体，优化两段时长、窗口二维点和过窗三维速度；不是未修改TOGT的C6位置-only MINCO。目标仅T1+T2。每段17个节点约束单电机推力、机体角速度、工作空间；局部动态安全采用浮点包络/见证交换。软件求解预算90 ms，目标每次规划不超过100 ms，但并非硬实时认证。

## 3. 当前hard_v2：低档新飞配对

双方均为本批新飞。所有表内时间均为无碰撞完整完赛时间。

| 幅度m | 频率Hz | 自由点s | 中心s | 自由点节省s | 相对中心节省 |
|---:|---:|---:|---:|---:|---:|
| 1 | 0.5 | 3.932 | 3.960 | 0.028 | 0.7% |
| 2 | 1 | 3.928 | 3.968 | 0.040 | 1.0% |
| 3 | 2 | 3.600 | 3.660 | 0.060 | 1.6% |
| 4 | 3 | 4.788 | 6.428 | 1.640 | 25.5% |
| 4 | 3.5 | 5.820 | 7.864 | 2.044 | 26.0% |
| 4.5 | 3 | 5.260 | 6.732 | 1.472 | 21.9% |
| 5 | 3 | 5.584 | 6.804 | 1.220 | 17.9% |
| 5.5 | 3 | 5.916 | 8.268 | 2.352 | 28.4% |

八份配对配置审计均确认只有free_points不同，MPC/PX4/case一致。源码运行期不变、无重试。双方均无碰撞，**本批没有碰撞差异证据**；各一次，几十毫秒差距不解释为稳定优势。

来源：[低档配对报告](results/hard_lower_paired_v1_20261009/REPORT.md)。

## 4. 当前hard_v2：较高档配对

自由版来自两档硬动力学验证和七档升档，中心版随后补测，不把引用记录再次算作新飞。

| 幅度m | 频率Hz | 自由点 | 中心 |
|---:|---:|---|---|
| 4 | 4 | 无碰撞，6.504 s | 无首解，40 s超时 |
| 4 | 4.5 | 无碰撞，8.696 s | 无首解，40 s超时 |
| 4 | 5 | 无首解，40 s超时 | 无首解，40 s超时 |
| 4.5 | 3.5 | 无碰撞，5.488 s | 无首解，40 s超时 |
| 5 | 3.5 | 无碰撞，6.592 s | 无首解，40 s超时 |
| 5 | 4 | 实体碰框，无有效完赛时间 | 无首解，40 s超时 |
| 5.5 | 3.5 | 无碰撞，6.896 s | 无首解，40 s超时 |
| 6 | 3.5 | 无碰撞，8.064 s | 无首解，40 s超时 |
| 6.5 | 3.5 | 无碰撞，7.968 s | 无首解，40 s超时 |

中心九组都没有接受初始轨迹，没有实际穿窗，零窗框接触。没有双方成功的条件，因此不计算时间优势。升档路线停止点不构成物理可行性阈值，难度和结果不保证单调。

来源：[初始硬动力学验证](results/hard_validation_v2_20261009/REPORT.md)、[升档报告](results/hard_sweep_v1_20261009/REPORT.md)、[中心补测报告](results/hard_center_paired_v1_20261009/REPORT.md)。

### 4.1 4 Hz／5 m自由点的真实碰撞

真实中心约3.199 s过平面，首次窗框接触在3.220 s，机体尚未完全离窗。接触附近跟踪误差约0.358 m。表内最大误差11.489 m包含碰撞后偏离，不能当作碰撞前的原因。

原始程序虽记录后来到达终点、用时8.776 s，但存在碰撞，**该时间不是有效竞速成绩**。当前证据显示名义安全与实际执行存在缺口，不足以确定全部因果来源。

### 4.2 为什么有些完成要7～8秒

| 幅度m | 频率Hz | 首解接受s | 整机离窗s | 完成s | 离窗后耗时s |
|---:|---:|---:|---:|---:|---:|
| 4.5 | 3.5 | 0.056 | 3.252 | 5.488 | 2.236 |
| 5 | 3.5 | 0.064 | 3.120 | 6.592 | 3.472 |
| 5.5 | 3.5 | 0.064 | 3.112 | 6.896 | 3.784 |
| 6 | 3.5 | 1.292 | 4.260 | 8.064 | 3.804 |
| 6.5 | 3.5 | 0.104 | 3.256 | 7.968 | 4.712 |
| 4 | 4.5 | 0.980 | 4.092 | 8.696 | 4.604 |

主要差异在窗后轨迹和减速到终点；部分还等首解较久。求解策略是首个收敛且通过局部检查的候选，不保证全局最快。比如6.5 m首条规划已经给出约5.37 s尾段，不能全归咎于MPC跟踪拖慢。

## 5. 用户指定重测与现场可视化

这些不是挑最好成绩替换批次，全部独立保留。

| 条件 | 方式 | 结果 | 完整时间 |
|---|---|---|---:|
| 4 Hz／4 m中心 | 用户指定重测一次 | 137次请求，零接受初始轨迹；40 s超时，零接触 | — |
| 4 Hz／4 m自由 | GUI现场新规划，第1次 | 无碰撞完赛 | 6.488 s |
| 4 Hz／4 m自由 | GUI现场新规划，第2次 | 无碰撞完赛 | 6.532 s |

中心重测的config/MPC/PX4/case与先前一致。自由GUI两次都是新执行，不是回放。初态、实时调度和求解时延有细微差异，因此不能期待闭环成绩逐位相同；本轮未做重复性统计设计。

第二次GUI真实中心穿越时刻为 **2.932191 s**，整机离窗确认约 **2.968 s**。窗口局部穿越点为(0.200054, 0.623220) m，偏离中心约0.654542 m。

![真实穿窗截面](results/hard_free_f4_a4_gui_20261009_02/crossing_section.png)

图由Gazebo真值重建，非名义轨迹。橙色盘为半径0.38594472 m的机体外接球与窗口平面的截面，虚线是球的投影，**不是实际x500网格轮廓或精确碰撞体截面**。蓝箭头为窗口运动方向；面内局部坐标不等于世界上下左右。穿越前后20 ms窗口发生反向运动。

## 6. 旧版本历史：不要与hard_v2混合

### 6.1 原速度开放版与执行修复版

| 幅度m | 频率Hz | 原速度开放版 | 执行修复v2 |
|---:|---:|---|---|
| 4 | 3 | 无碰撞4.288 s（已有基准） | 本轮该版本未重飞 |
| 4 | 3.5 | 无碰撞28.520 s | 无碰撞5.080 s |
| 4 | 4 | 超时未完成 | 漏窗中止 |
| 4.5 | 3 | 无碰撞12.996 s | 无碰撞4.232 s |
| 4.5 | 3.5 | 超时未完成 | 漏窗中止 |
| 5 | 3 | 无碰撞13.496 s | 无碰撞4.952 s |
| 5.5 | 3 | 超时、无合法穿窗 | 无碰撞5.408 s |

旧升档6趟均零窗框接触，但只有3趟合法完赛；“不撞且到终点”不替代合法穿窗。执行修复v2共6趟、4完赛2漏窗中止。修复包括耦合分配/电机响应跟踪预测、阻力修正代价和窗后恢复，不是单一改动消融。

另有执行修复v1的90%推力准入试验（3 Hz／4.5 m）40 s无可接纳初始规划，负结果保留。

来源：[旧升档](results/difficulty_sweep_v1_20261009/REPORT.md)、[执行修复](results/repair_validation_v2_20261009/REPORT.md)。

### 6.2 hard_v1

4 Hz／4 m无碰撞完赛21.004 s，其中等首解14.728 s。hard_v2该条件6.504 s。两版初始化等设置发生变化，不能把全部改善归于硬动力学约束本身。hard_v1另一档固定输入探针失败，不是实飞。

## 7. 求解耗时、动力学和证据边界

| 批次 | 规划最长ms | MPC最长ms | 1 ms离线诊断最大单电机节点间超限 |
|---|---:|---:|---:|
| hard_v2两档初始验证 | 86.32 | 37.39 | 5.095% |
| hard_v2七档升档 | 103.00 | 57.63 | 6.321% |
| hard_v2低档16趟配对 | 78.58 | 44.48 | 2.437% |

升档103 ms那次worker本身约101.59 ms，不只是队列延时。因此不能写“全部规划严格小于100 ms”。表只概括这些批次，不代替中心高档等其他记录的逐次时延。

硬动力学界只在优化节点施加；名义映射也不是完整PX4闭环/电机初态逆模型。1 ms离线诊断只用于报告，不用于修改已飞条件；以上超限说明不能宣称连续硬约束认证。接触日志为本次仿真实体碰撞证据，不等同真机认证。

验证记录：当前方法11项测试通过；此前修复联合回归40项、硬动力学联合回归42项，测试集合不同，不相加为93项。各批源码快照、参数审计及原始数据保留。GUI现场运行有图形渲染负载，与无GUI批次分列。

## 8. 当前支持与不支持的结论

可以报告：在本批低档一次性配对中，自由点8条件均不慢于中心；较剧烈的5条件完整时间减少17.9%～28.4%。在较高档，当前自由版有7个可执行成功实例，而当前中心求解器这些条件未返回初始可接纳轨迹。

不能报告：自由点一定更快、中心物理不可行、中心必撞、自由点保证安全，或随机成功率显著更高。当前自由版确有一趟实体碰框。

尚待解决：中心高档求解瓶颈；实际穿窗偏差与名义安全余量的关系；窗后尾段局部最优；节点间动力学间隙及偶发超过100 ms。

## 附录A：全部52趟实飞原始记录

下面逐条列原始记录，不把引用历史结果重复计数。成功列采用原始完成且无碰撞；正式合法穿窗结论以相应批次审计为准。失效时间不作有效成绩。
“未完成”原因按原始failure，不推断物理不可行。

| 原始目录 | 类型 | 幅度m | 频率Hz | 结果 | 有效时间s／失败 |
|---|---|---:|---:|---|---|
| [amplitude_ours_f3_a4.5_p000](results/difficulty_sweep_v1_20261009/amplitude_ours_f3_a4.5_p000/result.json) | ours | 4.5 | 3 | 完成（见审计） | 12.996 |
| [amplitude_ours_f3_a5.5_p000](results/difficulty_sweep_v1_20261009/amplitude_ours_f3_a5.5_p000/result.json) | ours | 5.5 | 3 | 未完成 | timeout |
| [amplitude_ours_f3_a5_p000](results/difficulty_sweep_v1_20261009/amplitude_ours_f3_a5_p000/result.json) | ours | 5 | 3 | 完成（见审计） | 13.496 |
| [frequency_ours_f3.5_a4_p000](results/difficulty_sweep_v1_20261009/frequency_ours_f3.5_a4_p000/result.json) | ours | 4 | 3.5 | 完成（见审计） | 28.520 |
| [frequency_ours_f4_a4_p000](results/difficulty_sweep_v1_20261009/frequency_ours_f4_a4_p000/result.json) | ours | 4 | 4 | 未完成 | timeout |
| [joint_ours_f3.5_a4.5_p000](results/difficulty_sweep_v1_20261009/joint_ours_f3.5_a4.5_p000/result.json) | ours | 4.5 | 3.5 | 未完成 | timeout |
| [flight_forward_v2_f3_a4_20261009_01](results/flight_forward_v2_f3_a4_20261009_01/result.json) | ours | 4 | 3 | 完成（见审计） | 4.288 |
| [hard_center_f4_a4_retest_20261009_01](results/hard_center_f4_a4_retest_20261009_01/result.json) | center | 4 | 4 | 未完成 | timeout |
| [center_ablation_center_f3.5_a4.5_p000](results/hard_center_paired_v1_20261009/center_ablation_center_f3.5_a4.5_p000/result.json) | center | 4.5 | 3.5 | 未完成 | timeout |
| [center_ablation_center_f3.5_a5.5_p000](results/hard_center_paired_v1_20261009/center_ablation_center_f3.5_a5.5_p000/result.json) | center | 5.5 | 3.5 | 未完成 | timeout |
| [center_ablation_center_f3.5_a5_p000](results/hard_center_paired_v1_20261009/center_ablation_center_f3.5_a5_p000/result.json) | center | 5 | 3.5 | 未完成 | timeout |
| [center_ablation_center_f3.5_a6.5_p000](results/hard_center_paired_v1_20261009/center_ablation_center_f3.5_a6.5_p000/result.json) | center | 6.5 | 3.5 | 未完成 | timeout |
| [center_ablation_center_f3.5_a6_p000](results/hard_center_paired_v1_20261009/center_ablation_center_f3.5_a6_p000/result.json) | center | 6 | 3.5 | 未完成 | timeout |
| [center_ablation_center_f4.5_a4_p000](results/hard_center_paired_v1_20261009/center_ablation_center_f4.5_a4_p000/result.json) | center | 4 | 4.5 | 未完成 | timeout |
| [center_ablation_center_f4_a4_p000](results/hard_center_paired_v1_20261009/center_ablation_center_f4_a4_p000/result.json) | center | 4 | 4 | 未完成 | timeout |
| [center_ablation_center_f4_a5_p000](results/hard_center_paired_v1_20261009/center_ablation_center_f4_a5_p000/result.json) | center | 5 | 4 | 未完成 | timeout |
| [center_ablation_center_f5_a4_p000](results/hard_center_paired_v1_20261009/center_ablation_center_f5_a4_p000/result.json) | center | 4 | 5 | 未完成 | timeout |
| [hard_free_f4_a4_gui_20261009_01](results/hard_free_f4_a4_gui_20261009_01/result.json) | ours | 4 | 4 | 完成（见审计） | 6.488 |
| [hard_free_f4_a4_gui_20261009_02](results/hard_free_f4_a4_gui_20261009_02/result.json) | ours | 4 | 4 | 完成（见审计） | 6.532 |
| [lower_center_f0.5_a1_p000](results/hard_lower_paired_v1_20261009/lower_center_f0.5_a1_p000/result.json) | center | 1 | 0.5 | 完成（见审计） | 3.960 |
| [lower_center_f1_a2_p000](results/hard_lower_paired_v1_20261009/lower_center_f1_a2_p000/result.json) | center | 2 | 1 | 完成（见审计） | 3.968 |
| [lower_center_f2_a3_p000](results/hard_lower_paired_v1_20261009/lower_center_f2_a3_p000/result.json) | center | 3 | 2 | 完成（见审计） | 3.660 |
| [lower_center_f3.5_a4_p000](results/hard_lower_paired_v1_20261009/lower_center_f3.5_a4_p000/result.json) | center | 4 | 3.5 | 完成（见审计） | 7.864 |
| [lower_center_f3_a4.5_p000](results/hard_lower_paired_v1_20261009/lower_center_f3_a4.5_p000/result.json) | center | 4.5 | 3 | 完成（见审计） | 6.732 |
| [lower_center_f3_a4_p000](results/hard_lower_paired_v1_20261009/lower_center_f3_a4_p000/result.json) | center | 4 | 3 | 完成（见审计） | 6.428 |
| [lower_center_f3_a5.5_p000](results/hard_lower_paired_v1_20261009/lower_center_f3_a5.5_p000/result.json) | center | 5.5 | 3 | 完成（见审计） | 8.268 |
| [lower_center_f3_a5_p000](results/hard_lower_paired_v1_20261009/lower_center_f3_a5_p000/result.json) | center | 5 | 3 | 完成（见审计） | 6.804 |
| [lower_ours_f0.5_a1_p000](results/hard_lower_paired_v1_20261009/lower_ours_f0.5_a1_p000/result.json) | ours | 1 | 0.5 | 完成（见审计） | 3.932 |
| [lower_ours_f1_a2_p000](results/hard_lower_paired_v1_20261009/lower_ours_f1_a2_p000/result.json) | ours | 2 | 1 | 完成（见审计） | 3.928 |
| [lower_ours_f2_a3_p000](results/hard_lower_paired_v1_20261009/lower_ours_f2_a3_p000/result.json) | ours | 3 | 2 | 完成（见审计） | 3.600 |
| [lower_ours_f3.5_a4_p000](results/hard_lower_paired_v1_20261009/lower_ours_f3.5_a4_p000/result.json) | ours | 4 | 3.5 | 完成（见审计） | 5.820 |
| [lower_ours_f3_a4.5_p000](results/hard_lower_paired_v1_20261009/lower_ours_f3_a4.5_p000/result.json) | ours | 4.5 | 3 | 完成（见审计） | 5.260 |
| [lower_ours_f3_a4_p000](results/hard_lower_paired_v1_20261009/lower_ours_f3_a4_p000/result.json) | ours | 4 | 3 | 完成（见审计） | 4.788 |
| [lower_ours_f3_a5.5_p000](results/hard_lower_paired_v1_20261009/lower_ours_f3_a5.5_p000/result.json) | ours | 5.5 | 3 | 完成（见审计） | 5.916 |
| [lower_ours_f3_a5_p000](results/hard_lower_paired_v1_20261009/lower_ours_f3_a5_p000/result.json) | ours | 5 | 3 | 完成（见审计） | 5.584 |
| [amplitude_ours_f3.5_a5.5_p000](results/hard_sweep_v1_20261009/amplitude_ours_f3.5_a5.5_p000/result.json) | ours | 5.5 | 3.5 | 完成（见审计） | 6.896 |
| [amplitude_ours_f3.5_a5_p000](results/hard_sweep_v1_20261009/amplitude_ours_f3.5_a5_p000/result.json) | ours | 5 | 3.5 | 完成（见审计） | 6.592 |
| [amplitude_ours_f3.5_a6.5_p000](results/hard_sweep_v1_20261009/amplitude_ours_f3.5_a6.5_p000/result.json) | ours | 6.5 | 3.5 | 完成（见审计） | 7.968 |
| [amplitude_ours_f3.5_a6_p000](results/hard_sweep_v1_20261009/amplitude_ours_f3.5_a6_p000/result.json) | ours | 6 | 3.5 | 完成（见审计） | 8.064 |
| [frequency_ours_f4.5_a4_p000](results/hard_sweep_v1_20261009/frequency_ours_f4.5_a4_p000/result.json) | ours | 4 | 4.5 | 完成（见审计） | 8.696 |
| [frequency_ours_f5_a4_p000](results/hard_sweep_v1_20261009/frequency_ours_f5_a4_p000/result.json) | ours | 4 | 5 | 未完成 | timeout |
| [joint_ours_f4_a5_p000](results/hard_sweep_v1_20261009/joint_ours_f4_a5_p000/result.json) | ours | 5 | 4 | 碰撞 | — |
| [hard_v1_f4_a4_20261009_01](results/hard_v1_f4_a4_20261009_01/result.json) | ours | 4 | 4 | 完成（见审计） | 21.004 |
| [hard_v2_f3.5_a4.5_20261009_01](results/hard_v2_f3.5_a4.5_20261009_01/result.json) | ours | 4.5 | 3.5 | 完成（见审计） | 5.488 |
| [hard_v2_f4_a4_20261009_01](results/hard_v2_f4_a4_20261009_01/result.json) | ours | 4 | 4 | 完成（见审计） | 6.504 |
| [repair_v1_f3_a4.5_20261009_01](results/repair_v1_f3_a4.5_20261009_01/result.json) | ours | 4.5 | 3 | 未完成 | timeout |
| [repair_v2_f3.5_a4.5_20261009_01](results/repair_v2_f3.5_a4.5_20261009_01/result.json) | ours | 4.5 | 3.5 | 未完成 | RuntimeError('missed_opening_no_reverse_reentry') |
| [repair_v2_f3.5_a4_20261009_01](results/repair_v2_f3.5_a4_20261009_01/result.json) | ours | 4 | 3.5 | 完成（见审计） | 5.080 |
| [repair_v2_f3_a4.5_20261009_01](results/repair_v2_f3_a4.5_20261009_01/result.json) | ours | 4.5 | 3 | 完成（见审计） | 4.232 |
| [repair_v2_f3_a5.5_20261009_01](results/repair_v2_f3_a5.5_20261009_01/result.json) | ours | 5.5 | 3 | 完成（见审计） | 5.408 |
| [repair_v2_f3_a5_20261009_01](results/repair_v2_f3_a5_20261009_01/result.json) | ours | 5 | 3 | 完成（见审计） | 4.952 |
| [repair_v2_f4_a4_20261009_01](results/repair_v2_f4_a4_20261009_01/result.json) | ours | 4 | 4 | 未完成 | RuntimeError('missed_opening_no_reverse_reentry') |

## 附录B：15次离线开发/固定输入测试

这些不是实飞，表内名义时间不能与闭环完赛时间混比。不同开发测试预算/模型可能不同；失败耗时保留在原始error字段，不冒充成功求解耗时。found只代表当前探针找到候选，非连续认证。

| 原始目录 | found | 成功求解耗时ms | 名义轨迹总时间s |
|---|---|---:|---:|
| [development_compiled_forward_v1_20261009](results/development_compiled_forward_v1_20261009/result.json) | 是 | 82.843 | 4.311 |
| [development_compiled_forward_v2_20261009](results/development_compiled_forward_v2_20261009/result.json) | 是 | 67.343 | 4.608 |
| [development_forward_log_v_v1_20261009](results/development_forward_log_v_v1_20261009/result.json) | 是 | 125.060 | 4.311 |
| [development_forward_log_va_v1_20261009](results/development_forward_log_va_v1_20261009/result.json) | 否 | — | — |
| [development_forward_v_v1_20261009](results/development_forward_v_v1_20261009/result.json) | 否 | — | — |
| [development_forward_va_v1_20261009](results/development_forward_va_v1_20261009/result.json) | 否 | — | — |
| [development_free_va_v1_20261009](results/development_free_va_v1_20261009/result.json) | 是 | 121.286 | 7.698 |
| [development_interpreter_v1_20261009](results/development_interpreter_v1_20261009/result.json) | 是 | 57.906 | 8.241 |
| [development_kinematic_v_v1_20261009](results/development_kinematic_v_v1_20261009/result.json) | 是 | 84.255 | 14.942 |
| [development_kinematic_va_v1_20261009](results/development_kinematic_va_v1_20261009/result.json) | 是 | 205.186 | 11.002 |
| [development_short_seed_v1_20261009](results/development_short_seed_v1_20261009/result.json) | 是 | 68.339 | 8.190 |
| [hard_v1_probe_f3.5_a4.5_20261009_01](results/hard_v1_probe_f3.5_a4.5_20261009_01/result.json) | 否 | — | — |
| [hard_v1_probe_f4_a4_20261009_01](results/hard_v1_probe_f4_a4_20261009_01/result.json) | 是 | 91.222 | 6.257 |
| [hard_v2_probe_f3.5_a4.5_20261009_01](results/hard_v2_probe_f3.5_a4.5_20261009_01/result.json) | 是 | 37.861 | 5.884 |
| [hard_v2_probe_f4_a4_20261009_01](results/hard_v2_probe_f4_a4_20261009_01/result.json) | 是 | 69.076 | 6.770 |

