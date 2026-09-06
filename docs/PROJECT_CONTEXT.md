# 当前项目状态

更新：2026-09-06；核对 HEAD：`40692e8`。本页来自当前源码、方法报告和 Git 历史；本轮是文档审计，没有重新运行算法实验，以下实验数字均为带来源的已有记录。

## 当前重心

仓库已从连续二维生成迷宫的纯 PPO 教师，发展为多个独立问题族和多种规划/学习方法。最近提交集中在非凸时变窗口：8 月的整机安全约束、SC/SIP 比较与认证加速、AVS 安全强化学习，以及 9 月的 RotSync 和 PhaseGuard-RL。因而“二维 PPO 是唯一主线”不再适合作为全仓定位；GAP-Step 的历史实验仍保留其局部约定。

当前目标包括非凸可行域内选点、穿越时间优化、整机在穿越前后连续时间内的安全、动力学限制，以及在相同物理模型下评估规划与学习的实际贡献。固定平面旋转任务、完整平移/RPY/缩放任务和连续局部形变任务不能互相代替。

## 方法状态与结论边界

| 方法 | 当前实现/已有证据 | 未完成或不能推出的结论 |
|---|---|---|
| [AtlasDynaTOGT](../nonconvex_timevarying_window/atlas_dynatogt/README.md) | 非凸三角 chart atlas、时空联合优化；历史 default 记录为 14/14 场景成功 | Hermite 轨迹及指标与 SC/MINCO 不同，不能直接当同模型速度基线 |
| [SC-DynaTOGT](../nonconvex_timevarying_window/sc_dynatogt/TEST_RESULTS.md) | Chang 边界重采样、Clipper2 内缩、SC 内部映射、degree-7 MINCO；E0–E5 default 已有记录 | 6/6 名义穿越合法不代表整条轨迹动力学通过；强运动演示明确记录 `sampled_dynamic_limits_satisfied=false` |
| [MSR-DynaTOGT](../nonconvex_timevarying_window/msr_dynatogt/TEST_RESULTS.md) | SC 外层多初值和时间修复；formal 775 个任务、9,300 行；A2/A3 采样动力学可行率均 100% | 是密集采样验收；A3 代价较大，匹配起点数/墙钟预算时 A2/A3 选择相同结果，不能无条件宣称多初值更优 |
| [SIP-DynaTOGT](../nonconvex_timevarying_window/sip_dynatogt/COMPLETE_ALGORITHM_SPECIFICATION.md) | SLSQP witness 循环和 Arb 连续域认证，真实原始曲线、姿态长方体与动力学硬约束；有证书重放入口 | 仅名义模型、局部优化；不保证全局最优、固定求解时限或真实跟踪鲁棒性 |
| [Planar-RS-DynaTOGT](../nonconvex_timevarying_window/planar_rs_dynatogt/TEST_RESULTS.md) | 固定平面排除加原始曲线认证；普通单窗端到端 37.59 s，六窗极难赛道约 30 分 6 秒获认证 | 仅固定中心/平面、面内旋转和统一缩放；不能推广成任意赛道一分钟求解 |
| [RotSync-SC-TOGT](../nonconvex_timevarying_window/rot_sync_sc_togt/README.md) | 解析 Sync 与七阶 MINCO 按 PVAJ 连接；四条正式赛道、现实尺度极限场景和单窗口固定点比较入口 | 仅固定平面法向匀速自旋；整机碰撞/动力学验收采用密集采样，不是 SIP 连续域证书 |
| [AVS-PPO](../nonconvex_timevarying_window/avs_ppo/TEST_RESULTS.md) | 动作掩码与可恢复盾牌；三窗平移/球形模型 200 ID + 200 OOD 回合均完赛、零违规 | 这是特定确定性模型的实验统计，不能外推为姿态/电机/不确定性下的硬安全保证 |
| [PhaseGuard-RL](../nonconvex_timevarying_window/phaseguard_rl/README.md) | 精简核心：相位/状态观测、点/时间动作、固定 MINCO 轨迹、认证准入、一步完整规划 PPO 与测试 | 尚未正式场景训练和性能实验；`train.py` 提供 Python 函数，尚不是配套完整场景 CLI |
| [FAPP-PPO](../closed_loop_deformable_window/fapp_ppo/TEST_RESULTS.md) | 外生非周期开放日程、连续局部形变、未来预览和残差 CTBR PPO；有验证训练与机制演示 | 最终 ID pilot 为 0/10，后期退化；早期成功视频不能替代最终模型结果 |
| [MDG](../closed_loop_deformable_window/mdg/docs/IMPLEMENTATION_STATUS.md) | 移动安全圆盘、时空图、DP、MINCO 适配与 Lazy Repair；E1–E6 smoke 已记录 | 正式 2,090 次实验矩阵尚未全量运行，完整验收待完成 |

[WBSC/CWB/Exact-Area](../nonconvex_timevarying_window/README.md)已置于 `废案/`，作为旧方案和非凸整机反例保留；不能再列成当前新增主方法。

## 必须保留的负结果

- **RotSync 正式赛道**：2026-09-02 报告中 D1/D2 完整通过，D3/D4 虽几何、C3、闭合与采样零碰撞通过，最大速度仍为 7.1894/7.1040 m/s，超过 7 m/s 上限，必须保留失败标记。见[正式结果](../nonconvex_timevarying_window/rot_sync_sc_togt/FORMAL_EXPERIMENTS.md)。
- **RotSync 单窗口比较**：2026-09-05 的 L/U/star × 三档转速九对案例中，两方法均 9/9 合格，固定点基线全部更快。当前结果未证明 Sync 性能优势；两方法同时改变选点与同步结构，不能单独归因。见[方法 README 的比较记录](../nonconvex_timevarying_window/rot_sync_sc_togt/README.md)。
- **AVS 极难六窗**：严格盾版用 16.15 s 完成闭环，但平均可行动作比例 0.08764、masked entropy 为零。应记录为盾牌几乎接管，不能作为 PPO 学习成功证据。见[极难比较报告](../nonconvex_timevarying_window/avs_ppo/HARDEST_COMPARISON_REPORT.md)。
- **FAPP-PPO**：100 次更新/102,400 步验证训练的最终 ID 为 0/10，两个 nominal 基线各 1/10。旧记录诊断穿越后切换势函数目标带来负奖励跳变；修复和正式复验不能仅凭诊断标为完成。见[测试记录](../closed_loop_deformable_window/fapp_ppo/TEST_RESULTS.md)。

## 跨方法比较与本地工作

[SC/SIP 宽域快速六窗比较](../nonconvex_timevarying_window/comparisons/sc_sip_fast_closed_loop/README.md)保留 SC 整机实体相交和动力学违规，以及 SIP 最终连续域认证。该案例经过续跑、witness 批量补充和规划裕量调整，是历史压力案例，不是冻结协议后一次性无偏基准，也不能推出任意赛道上的普遍优劣。

[运动速率基准](../nonconvex_timevarying_window/comparisons/sc_sip_motion_rate_benchmark/README.md)有 12 种子 × 3 速率、36 实例的冻结输入和独立审计实现。需分别报告实体相交、15 mm 净距违规和未决状态；本页不因入口已提交就声称全量结果已验收。

审计开始时以下两部分为 **Git 未跟踪的用户本地工作**，本轮只读取并纳入状态：

| 本地内容 | 已有内容 | 尚不能宣称 |
|---|---|---|
| [ICRA_EXPERIMENT_PLAN.md](../nonconvex_timevarying_window/实验方案/ICRA_EXPERIMENT_PLAN.md) | RotSync 的五项实验设计：空间表示、同轨迹几何处理、窄窗/高速同步、多窗和跟踪；计划输出到方法下 `icra_experiments/` | 五项实验已实现或执行、文中论文已由本次审计重新查证 |
| [Gazebo 适配层](../nonconvex_timevarying_window/comparisons/sc_sip_fast_closed_loop/gazebo/README.md) | 六窗世界导出、运动桥接和启动脚本；说明周期平移/RPY 接入 | 均匀缩放已完整复现、PX4 闭环接通、实物验证或替代 SIP 证书 |

工作区文件可能不随 Git 克隆存在。后续状态更新应重新查看 `git status`，不要永久把它们标成未提交。

## 保留研究线

GAP-Step 的较新入口是 `window_maze_env.py -> train_window.py / evaluate_window.py / visualize_window.py`。旧 2026-05-15 记录为 ID 71.5%、窗口 OOD 54.0%、迷宫 OOD 74.5%（各 200 回合），是历史验收数字，不是本轮测试。旧 `env.py/train.py` 和 passage、planner/BC 辅助代码仍存在；纯 PPO 配置默认不启用辅助，不应通过抹去代码事实维持“纯 PPO”叙述。路径及配置见 [RUNBOOK](RUNBOOK.md)。

DynaTOGT 在 `togt_timevarying_window/` 使用动态窗口、离散热启动、L-BFGS-B 和 Hermite 轨迹，保留重复穿越任务支持；SC 等目录的 MINCO 后端不代表此目录已替换为 MINCO。TOGT C++ 复现的历史构建结论见[复现审计](TOGT_REPRODUCTION_AUDIT.md)。

下一步见 [ROADMAP](ROADMAP.md)，变更依据见 [DECISIONS](DECISIONS.md)，提交时间线见 [TASK_LOG](TASK_LOG.md)。
