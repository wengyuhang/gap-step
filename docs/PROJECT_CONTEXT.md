# 当前项目状态

更新：2026-09-10。本页来自当前源码、方法报告、本地正式运行和 Git 历史；实验数字均标明来源与证据等级。

## 当前重心

仓库已从连续二维生成迷宫的纯 PPO 教师，发展为多个独立问题族和多种规划/学习方法。最近提交集中在非凸时变窗口：8 月的整机安全约束、SC/SIP 比较与认证加速、AVS 安全强化学习，以及 9 月的 RotSync 和 PhaseGuard-RL。因而“二维 PPO 是唯一主线”不再适合作为全仓定位；GAP-Step 的历史实验仍保留其局部约定。

当前目标包括非凸可行域内选点、穿越时间优化、整机在穿越前后连续时间内的安全、动力学限制，以及在相同物理模型下评估规划与学习的实际贡献。固定平面旋转任务、完整平移/RPY/缩放任务和连续局部形变任务不能互相代替。

## 方法状态与结论边界

2026-09-10 新增并列的 [凸时变窗口 TOGT 实验](../convex_timevarying_window/README.md)：
沿七异形闭合赛道的位置布置 2 个矩形、3 个精确圆、1 个正五边形和 1 个正六边形，所有窗口均作周期三轴平移和
roll/pitch/yaw 三轴旋转。求解严格复用 TOGT 发布 standard 目标及 L-BFGS 数值配置。
当前后端直接封装发布版 C++ 手写四旋翼梯度和 MINCO 伴随回传；凸多边形改用论文的
平方重心映射，平面圆使用 Ball 映射的二维同构，D 维数为 `4/2/5/2/6/2/4`。所有
TOGT margin 已进一步提高到外接球直径的 1.1 倍 `0.8347300205 m`，严格使用发布版
Rectangle/Ball/Pentagon/Hexagon 各自不同的缩量规则。当前冻结运行正常收敛，飞行时间
`18.724959468 s`，初始化 `0.001796 s`、282 次评价的 L-BFGS `0.965737 s`；
但最大单旋翼推力 `5.047529 N > 5 N`，安全验收 W1/W3/W5/W6 通过，
W2/W4/W7 仍分别侵入约 `97/301/310 mm`。故动力学和安全验收仍均失败。指定穿越时刻的点净空足够，但 margin 不约束相邻时刻的轨迹与运动门框。结果是最大 1 ms 步长的密集采样证据，
不是连续域认证。

2026-09-10，Dual-Constraint CEM SC-DynaTOGT 的安全软积分已补入该新方法的前置 L-BFGS：目标为原 TOGT 目标加固定权重的全轨迹可微安全积分，梯度经 MINCO、窗口时间导数和 SC Jacobian 回传到原生 `[K,D]`。原始 `sc_dynatogt/`、Fixed-WP 未修改，多中心与停滞重启未加入。七窗口无内缩旧负结果早于此修改，保留作问题定位，不能当成当前流程实验结果。

同日完成无历史轨迹/候选关联的无内缩冷启动正式运行。L-BFGS 用时 `2356.510 s`，CEM 用时 `102.775 s`，预处理、L-BFGS、CEM 合计 `2466.570 s`。512 条 CEM 候选中有 24 条当前离散网格严格双零且完整动力学均通过，但 24 条真实外接球检测全部失败，故没有验收解。失败暴露出每段最多 32 区间的全段安全积分会漏掉 `0.02–0.18 s` 的短接触区间；软积分为零仍不能替代密集安全检测。

三窗最新成功原型：新增 [Feasibility-Guided CEM SC-DynaTOGT](../nonconvex_timevarying_window/feasibility_guided_cem_sc_dynatogt/README.md)。它显式复用两条单窗硬筛选通过轨迹的安全相位，以周期别名生成 2820 个多窗前端候选，再从峰值速度最低的三窗几何通过者启动 7 轮、每轮 256 个样本的完整协方差 CEM。总计 4612 次评估得到 3 条全部中间硬约束通过轨迹；最短者 T=7.390546627 s、最大速度 6.999106268 m/s，三个窗口最终真实姿态长方体审计均为零碰撞采样，最小门框净空 50.215/266.307/58.077 mm。总运行 278.328 s，其中最终审计 156.572 s。失败样本只指导提议分布，最终排序只含全约束通过者。结果是单场景、单种子的名义模型采样证据，不是连续认证或一般成功率结论；详见[三窗结果](../nonconvex_timevarying_window/feasibility_guided_cem_sc_dynatogt/THREE_WINDOW_RESULTS.md)。

曲线边界三方法对比新增一条 11.2 m 开放赛道，依次为利马松、五瓣波浪曲线和直线–三次 Bézier 混合窗口，三窗固定平面并分别以 1.5/-2.0/2.5 rad/s 自旋。原始 Fixed-WP 为 3.878918907 s、原始 SC-DynaTOGT 为 3.497544115 s，两者整机碰撞约束均通过但旋翼推力约束失败；因此不进入合格时间排名。Feasibility-Guided CEM 在 552 个候选中找到 309 个中间硬筛选通过者，最终选择 3.468057675 s，动力学与三窗真实姿态长方体碰撞审计均通过。结果见[曲线赛道报告](../nonconvex_timevarying_window/comparisons/curved_rotating_sc_fixed_wp/results/three_way_20260909/REPORT.md)，证据仍为密集采样而非连续认证。

七窗口混合压力案例依次为均衡 U、利马松、星形、均衡 U、五瓣波浪、Line/Bézier、均衡 U，平面和中心固定且窗口只做面内自旋。四个插入窗口沿此前独立通过验收的三 U 轨迹布置，所以这是有已知可行种子的流程验证案例，不作为未知赛道无偏基准。Fixed-WP/原始 SC-DynaTOGT 分别为 8.666957342/8.666957333 s，二者均超速且在首个 U 窗口发生整机碰撞；Feasibility-Guided CEM 返回 7.390546627 s 的硬约束合格种子，七窗整机审计全部通过。结果见[七窗口报告](../nonconvex_timevarying_window/comparisons/seven_mixed_reference_sc_fixed_cem/results/three_way_trial2_20260909/REPORT.md)。本轮 256 个局部扰动均未新增硬约束合格解，必须保留这一负结果；采样通过不等于连续认证。

新增七种不同形状的分散闭环压力赛道，参考 `20260717_paper_irregular_closed` 的中心和姿态布局，起终点均为 `(-16,4,3.2) m`，窗口固定中心/平面且只做面内自旋。原始 SC-DynaTOGT 正常收敛到 `T=24.127878371 s`。按上传约束的有限零厚度门框模型，仅在全部 `|z|≤r_s` 区间做最大 `0.2 ms`、临界处 `0.05 ms` 球体检查；W5 波浪、W6 Line/Bézier、W7 均衡 U 碰撞，W1–W4 通过。结果与赛道图见[七种形状闭环实验](../nonconvex_timevarying_window/comparisons/seven_unique_sc_sphere/README.md)。若把开口外整张无限平面视作实体，闭合轨迹的回程平面交叉会产生结构性碰撞，因此该口径不用于这条有限门框赛道。

三窗最新续跑：按用户要求进一步扩大独立随机扰动，D 半径比例为 1/2/4/8，K 直接噪声比例为 0.5/1/2/4，共 8000 候选。57 个通过三窗球体和顺序检查，但全部超速，最低最大速度仍为 10.333593 m/s；无最终整机检测候选。搜索 89.793 s，总计 92.015 s（重放原 SC，无本次求解耗时）。未决、边界/C3 数值超限与数值失败均保留。详见[宽范围三窗结果](../nonconvex_timevarying_window/random_dk_sc_dynatogt/MULTI_WINDOW_WIDE_RESULTS.md)。

同日最新多窗试验：新增 Random-DK 三个自旋 U 窗口的开放赛道，固定平面、18 rad/s，使用原生四段 MINCO、4 个 K 标量和 3 组二维 D。4000 个候选中 877/12/4 个依次通过前 1/2/3 窗球体检查；最后 4 个均超速，最低最大速度仍为 10.57885 m/s。没有合格解、没有调用最终整机检测；见[多窗结果](../nonconvex_timevarying_window/random_dk_sc_dynatogt/MULTI_WINDOW_RESULTS.md)。单窗成功不能外推为已完成多窗修复。

2026-09-09 当前新增任务为 [Random-DK SC-DynaTOGT](../nonconvex_timevarying_window/random_dk_sc_dynatogt/README.md)：固定中心/平面、窗口自旋；原始 SC 最终解不合格时直接随机扰动 D/K。用户提供的球体距离约束仅检查全部球平面接触区间，全程动力学仍为硬筛选；只在全部合格候选中选最短时间，真实整机留到最终检测。零厚度均衡 U、18 rad/s、初相位 1.1 rad 的首次两组各 300 候选失败，负结果完整保留。用户随后要求扩大范围；D 半径比例扩大至 0.25/0.5/1/2，K 加噪声比例至 0.1/0.25/0.5/1，重放原解。各 400 候选仍失败；各 4000 候选时分别找到 1 条合格轨迹，T=3.167446933/3.523313547 s，最终最大 0.2 ms 加密整机与动力学检测均通过、零碰撞样本。两者都来自 D/K 联合扰动；详见[扩大范围结果](../nonconvex_timevarying_window/random_dk_sc_dynatogt/EXPANDED_SEARCH_RESULTS.md)。这是本地未提交原型的单窗采样修复结果，不是连续认证或普适成功率证据。

| 方法 | 当前实现/已有证据 | 未完成或不能推出的结论 |
|---|---|---|
| [AtlasDynaTOGT](../nonconvex_timevarying_window/atlas_dynatogt/README.md) | 非凸三角 chart atlas、时空联合优化；历史 default 记录为 14/14 场景成功 | Hermite 轨迹及指标与 SC/MINCO 不同，不能直接当同模型速度基线 |
| [SC-DynaTOGT](../nonconvex_timevarying_window/sc_dynatogt/TEST_RESULTS.md) | Chang 边界重采样、Clipper2 内缩、SC 内部映射、degree-7 MINCO；E0–E5 default 已有记录 | 6/6 名义穿越合法不代表整条轨迹动力学通过；强运动演示明确记录 `sampled_dynamic_limits_satisfied=false` |
| [MSR-DynaTOGT](../nonconvex_timevarying_window/msr_dynatogt/TEST_RESULTS.md) | SC 外层多初值和时间修复；formal 775 个任务、9,300 行；A2/A3 采样动力学可行率均 100% | 是密集采样验收；A3 代价较大，匹配起点数/墙钟预算时 A2/A3 选择相同结果，不能无条件宣称多初值更优 |
| [SIP-DynaTOGT](../nonconvex_timevarying_window/sip_dynatogt/COMPLETE_ALGORITHM_SPECIFICATION.md) | SLSQP witness 循环和 Arb 连续域认证，真实原始曲线、姿态长方体与动力学硬约束；有证书重放入口 | 仅名义模型、局部优化；不保证全局最优、固定求解时限或真实跟踪鲁棒性 |
| [Planar-RS-DynaTOGT](../nonconvex_timevarying_window/planar_rs_dynatogt/TEST_RESULTS.md) | 固定平面排除加原始曲线认证；普通单窗端到端 37.59 s，六窗极难赛道约 30 分 6 秒获认证 | 仅固定中心/平面、面内旋转和统一缩放；不能推广成任意赛道一分钟求解 |
| [RotSync-SC-TOGT](../nonconvex_timevarying_window/rot_sync_sc_togt/README.md) | 固定局部点 Sync 以 PVAJ 连接七阶 MINCO；有正式赛道、现实尺度和单窗对照入口 | 仅固定平面法向匀速自旋；整机碰撞/动力学验收采用密集采样，不是 SIP 连续域证书 |
| [Interpolated-RotSync-SC-TOGT](../nonconvex_timevarying_window/interpolated_rot_sync_sc_togt/README.md) | 旧双输入线性 Sync 可复现；主入口已扩展为 8 点七阶 SC 输入曲线和可变速单调法向曲线，解析到 snap 并以世界 PVAJ 连接七阶 MINCO | 高转速形状诊断虽显著削弱尖峰，仍未通过动力学或短于 Fixed-WP；安全与动力学为密集采样验收，不是连续域证书 |
| [AVS-PPO](../nonconvex_timevarying_window/avs_ppo/TEST_RESULTS.md) | 动作掩码与可恢复盾牌；三窗平移/球形模型 200 ID + 200 OOD 回合均完赛、零违规 | 这是特定确定性模型的实验统计，不能外推为姿态/电机/不确定性下的硬安全保证 |
| [PhaseGuard-RL](../nonconvex_timevarying_window/phaseguard_rl/README.md) | 精简核心：相位/状态观测、点/时间动作、固定 MINCO 轨迹、认证准入、一步完整规划 PPO 与测试 | 尚未正式场景训练和性能实验；`train.py` 提供 Python 函数，尚不是配套完整场景 CLI |
| [FAPP-PPO](../closed_loop_deformable_window/fapp_ppo/TEST_RESULTS.md) | 外生非周期开放日程、连续局部形变、未来预览和残差 CTBR PPO；有验证训练与机制演示 | 最终 ID pilot 为 0/10，后期退化；早期成功视频不能替代最终模型结果 |
| [MDG](../closed_loop_deformable_window/mdg/docs/IMPLEMENTATION_STATUS.md) | 移动安全圆盘、时空图、DP、MINCO 适配与 Lazy Repair；E1–E6 smoke 已记录 | 正式 2,090 次实验矩阵尚未全量运行，完整验收待完成 |

[WBSC/CWB/Exact-Area](../nonconvex_timevarying_window/README.md)已置于 `废案/`，作为旧方案和非凸整机反例保留；不能再列成当前新增主方法。

## 必须保留的负结果

- **RotSync 正式赛道**：2026-09-02 报告中 D1/D2 完整通过，D3/D4 虽几何、C3、闭合与采样零碰撞通过，最大速度仍为 7.1894/7.1040 m/s，超过 7 m/s 上限，必须保留失败标记。见[正式结果](../nonconvex_timevarying_window/rot_sync_sc_togt/FORMAL_EXPERIMENTS.md)。
- **RotSync 单窗口比较**：2026-09-05 的 L/U/star × 三档转速九对案例中，两方法均 9/9 合格，固定点基线全部更快。当前结果未证明 Sync 性能优势；两方法同时改变选点与同步结构，不能单独归因。见[方法 README 的比较记录](../nonconvex_timevarying_window/rot_sync_sc_togt/README.md)。
- **双 SC 输入 Sync 斜向单窗验证**：2026-09-07 本地运行中，新方法的两组优化输入距离为 0.121204，映射端点距离为 0.113786 m；C3 最大跳变 2.737e-14，最大 1 ms 动力学网格限制通过，实姿长方体 0/5001 碰撞采样。原 Sync 对照也通过轨迹采样验收，但优化器异常停止；两种状态分开记录。详见[独立方法 README](../nonconvex_timevarying_window/interpolated_rot_sync_sc_togt/README.md)。
- **零厚度固定/双 SC 输入对比**：均衡 U、尺寸比 1.9、4.5 rad/s、初相位 1.1 rad 下，窗口厚度改为零，Sync 两端定义为规划球与平面相切（法向坐标 `-rho/+rho`）。对照已纠正为固定一组 SC 输入的原 RotSync，不再用两段普通 MINCO Fixed-WP。运行前 0–4 阶导数嵌入误差不超过 `7.088e-13`；将固定输入最终解精确热启动到双输入法后，两者均为 `T=5.811201508 s`、`J=5.829428767`。两者均为 `0/5999` 碰撞样本，但 TOGT-code 1 ms 审计都有 192 个动力学违规样本，轨迹验收失败。详见[当前对比报告](../nonconvex_timevarying_window/interpolated_rot_sync_sc_togt/results/zero_thickness_nested_warmstart_togt_code_unlimited_20260908/REPORT.md)。
- **Fixed-WP 反推双输入初值**：保留普通两段 MINCO Fixed-WP 对照，从其零厚度轨迹反求 `-rho/+rho` 相切时刻和两个局部点，再做 SC 逆映射。反推初值 `T=2.245913723 s`，但 `0.087751947 s` 的 Sync 段导致初始动力学惩罚 `504891.2374`；无预算优化后为 `T=2.674461462 s`、`J=2530.901485`，独立 1 ms 审计有 2704 个动力学违规样本。普通 Fixed-WP 在两相切面之间仍是 MINCO 多项式，不是插值 Sync，因此它不是新参数化的可行子集。详见[反推初值报告](../nonconvex_timevarying_window/interpolated_rot_sync_sc_togt/results/zero_thickness_fixed_wp_seeded_togt_code_unlimited_20260908/REPORT.md)。
- **SC-DynaTOGT 单点自由选点对比**：零厚度均衡 U 窗口上，Fixed-WP 和 SC-DynaTOGT 共享两段 degree-7 MINCO、目标、求解器和起始轨迹，只开放一个二维 SC 穿越点。运行前嵌入的目标及 0–4 阶轨迹误差均为零；自由选点将 `T` 从 `2.486309570 s` 降到 `2.340844067 s`（`5.85066%`）。但两条轨迹的原生 1 ms 动力学审计均失败，自由解的 SC 数值回代点超出安全多边形 `5.122 nm`，故仍保留失败标记。详见[自由选点对比报告](../nonconvex_timevarying_window/interpolated_rot_sync_sc_togt/results/zero_thickness_sc_dynatogt_vs_fixed_wp_unlimited_20260908/REPORT.md)。
- **ICRA 实验三聚焦转速**：2026-09-07 本地正式运行 L/U 两形状、偏轴可行/轴心过窄几何和 0/1.5/3/4.5/6 rad/s，共 10 场景。Fixed-WP 10/10、Optimized-MINCO 9/10、SC+Sync 6/10 通过；U 形 4.5 rad/s 的 Optimized-MINCO 同时碰撞与动力学超限，L/U 的 SC+Sync 都在 4.5/6 rad/s 动力学实质超限。所有超限都大于 5% 敏感性带；证据为最大 1 ms 加密采样，不是连续域证书。见[实验报告](../nonconvex_timevarying_window/rot_sync_sc_togt/icra_experiments/03_sync_single/focused_results/REPORT.md)。
- **AVS 极难六窗**：严格盾版用 16.15 s 完成闭环，但平均可行动作比例 0.08764、masked entropy 为零。应记录为盾牌几乎接管，不能作为 PPO 学习成功证据。见[极难比较报告](../nonconvex_timevarying_window/avs_ppo/HARDEST_COMPARISON_REPORT.md)。
- **FAPP-PPO**：100 次更新/102,400 步验证训练的最终 ID 为 0/10，两个 nominal 基线各 1/10。旧记录诊断穿越后切换势函数目标带来负奖励跳变；修复和正式复验不能仅凭诊断标为完成。见[测试记录](../closed_loop_deformable_window/fapp_ppo/TEST_RESULTS.md)。

## 跨方法比较与本地工作

[SC/SIP 宽域快速六窗比较](../nonconvex_timevarying_window/comparisons/sc_sip_fast_closed_loop/README.md)保留 SC 整机实体相交和动力学违规，以及 SIP 最终连续域认证。该案例经过续跑、witness 批量补充和规划裕量调整，是历史压力案例，不是冻结协议后一次性无偏基准，也不能推出任意赛道上的普遍优劣。

[运动速率基准](../nonconvex_timevarying_window/comparisons/sc_sip_motion_rate_benchmark/README.md)有 12 种子 × 3 速率、36 实例的冻结输入和独立审计实现。需分别报告实体相交、15 mm 净距违规和未决状态；本页不因入口已提交就声称全量结果已验收。

审计开始时以下两部分为 **Git 未跟踪的用户本地工作**；2026-09-07 已在 ICRA 方案目录下实现并执行收窄后的实验三：

| 本地内容 | 已有内容 | 尚不能宣称 |
|---|---|---|
| [ICRA_EXPERIMENT_PLAN.md](../nonconvex_timevarying_window/实验方案/ICRA_EXPERIMENT_PLAN.md) | RotSync 的五项实验设计；[实验三收窄版](../nonconvex_timevarying_window/rot_sync_sc_togt/icra_experiments/03_sync_single/README.md)已实现并运行 5 个转速场景 | 其余四项实验已实现或执行；聚焦实验也不能证明 Sync 优势 |
| [Gazebo 适配层](../nonconvex_timevarying_window/comparisons/sc_sip_fast_closed_loop/gazebo/README.md) | 六窗世界导出、运动桥接和启动脚本；说明周期平移/RPY 接入 | 均匀缩放已完整复现、PX4 闭环接通、实物验证或替代 SIP 证书 |

工作区文件可能不随 Git 克隆存在。后续状态更新应重新查看 `git status`，不要永久把它们标成未提交。

## 保留研究线

GAP-Step 的较新入口是 `window_maze_env.py -> train_window.py / evaluate_window.py / visualize_window.py`。旧 2026-05-15 记录为 ID 71.5%、窗口 OOD 54.0%、迷宫 OOD 74.5%（各 200 回合），是历史验收数字，不是本轮测试。旧 `env.py/train.py` 和 passage、planner/BC 辅助代码仍存在；纯 PPO 配置默认不启用辅助，不应通过抹去代码事实维持“纯 PPO”叙述。路径及配置见 [RUNBOOK](RUNBOOK.md)。

DynaTOGT 在 `togt_timevarying_window/` 使用动态窗口、离散热启动、L-BFGS-B 和 Hermite 轨迹，保留重复穿越任务支持；SC 等目录的 MINCO 后端不代表此目录已替换为 MINCO。TOGT C++ 复现的历史构建结论见[复现审计](TOGT_REPRODUCTION_AUDIT.md)。

下一步见 [ROADMAP](ROADMAP.md)，变更依据见 [DECISIONS](DECISIONS.md)，提交时间线见 [TASK_LOG](TASK_LOG.md)。

## 2026-09-09：双软约束 CEM 的七异形闭环验收

新增 [Dual-Constraint CEM SC-DynaTOGT](../nonconvex_timevarying_window/dual_constraint_cem_sc_dynatogt/README.md)：删除按窗口设计的结构化前端，直接在原始 SC-DynaTOGT 最终 `[K,D]` 周围更新完整协方差。每个候选并行计算 TOGT 动力学软积分与含 15 mm 规划余量的可微门框安全软积分；固定 `epsilon=1e-6` 仅作为排序数值平台，合格仍要求两个原始积分严格等于零。首次严格双零后再搜索 5 轮。

在七种不同形状的分散闭环赛道上，共冻结 3072 条随机候选；首次严格双零位于第 42 轮，随后完整运行第 43–47 轮。按飞行时间排序的首条严格双零候选 `id=2701` 即通过独立硬验收，`T=24.709999069 s`，两个软积分均为零；全程 1 ms 动力学检测和七窗口真实外接球检测均通过，最小真实球体余量 `13.878932 mm`。Fixed-WP 与原始 SC-DynaTOGT 分别为 `24.217926422/24.127878371 s`，但二者动力学与球体碰撞检测都失败。正式结果见[三方法报告](../nonconvex_timevarying_window/comparisons/seven_unique_dual_constraint_cem/results/formal_final_post5_sphere_only_20260909/REPORT.md)。平面重复穿越只保留为诊断，不参与有限门框球体碰撞成败；所有硬验收仍是密集采样证据，不是连续域证书。

2026-09-10 新增[无内缩 SC 消融](../nonconvex_timevarying_window/comparisons/seven_unique_dual_constraint_cem_no_inset/README.md)。七个原始物理开口 SC 建图全部成功，但改前的原始无内缩 SC 将 W6/W7 点推至物理边界，50×64 个 CEM 候选无严格双零。已知成功轨迹可以等价映射到无内缩参数化并保持双零，说明失败来自时间/动力学基线与局部 CEM 的搜索盆地，而非无内缩参数化排除了可行解。该实验暴露的安全软项未进入前置 L-BFGS 问题已在新方法封装中修复；旧负结果保留，改后完整实验尚未运行。

## 2026-09-10：七异形闭环 Gazebo 场景

同一赛道已导出为[Gazebo Harmonic 场景组](../nonconvex_timevarying_window/comparisons/seven_unique_dual_constraint_cem/gazebo/README.md)。当前赛道源是独立的 `course_spec.py`：多边形门由工程尺寸顶点定义，三类曲线门由解析方程或 Bézier 控制点定义，并仅为 Gazebo 网格按 2 mm 弦偏差离散；赛道生成和验证的 `algorithm_inputs=[]`，不读取 SC 预处理、安全内缩或规划结果。赛道资产世界采用室内实验场风格，使用 Gazebo 内置 DART 与 1 ms 步长，85 mm 可见门套就是实际碰撞体。PX4 手动世界保留同一场地与七门几何，按官方 x500 配置使用 4 ms 步长，由 PX4 Gazebo 桥生成 x500 并接入 IMU/GPS/磁场、PX4 SITL、MAVLink 和 QGroundControl；支持鼠标虚拟摇杆与独立键盘遥控，不含路径规划或轨迹跟踪。算法结果仅能通过显式 `--with-replay` 加入另外两个复放世界；计入 10 mm 门框和 2 mm 网格误差后，旧正式轨迹保守净空为 `1.878932 mm`。Gazebo/PX4 手动飞行不替代连续域证书或飞控跟踪实验。
