# AGENTS.md

时变非凸窗口穿越研究仓库的代理约定。更新于 2026-09-06，依据当前源码与 Git 历史（审计基点 `40692e8`）。项目状态读 `docs/PROJECT_CONTEXT.md`，结构读 `docs/ARCHITECTURE.md`，命令读 `docs/RUNBOOK.md`；完整记忆索引见 `docs/README.md`。

## 项目定位与目录边界

- 仓库已发展为多方法窗口穿越研究。近期开发重点在 `nonconvex_timevarying_window/`，包括非凸选点、连续整机安全、旋转同步和安全强化学习；不要继续把全仓限制为二维纯 PPO。
- `nonconvex_timevarying_window/PROBLEM_DEFINITION.md` 保存基础问题定义。Atlas、SC、MSR、SIP、Planar-RS、RotSync、AVS-PPO、PhaseGuard-RL 各在独立方法目录维护；跨方法实验放在 `comparisons/`，旧 WBSC/CWB/Exact-Area 保留在 `废案/`。
- `closed_loop_deformable_window/PROBLEM_DEFINITION.md` 保存连续局部形变、安全区可为空、开放机会选择和完整初态返回的问题定义。`fapp_ppo/` 与 `mdg/` 是其并列方法，不能移回非凸总目录。
- `togt_timevarying_window/` 保留独立 DynaTOGT 动态窗口实验；`gap_step/` 保留早期二维迷宫研究；`复现/` 保留外部 TOGT 复现包。不得混淆它们的模型、指标和入口。
- 新算法在对应研究总目录下建独立子目录，源码、测试、文档与产物按方法组织。允许通过明确接口复用 SC/MINCO/SIP 基础能力；修改被复用模块时检查受影响的方法，不能把“并列方法”误解为没有依赖。
- 重要改动前读对应问题定义、方法 README、算法说明及实验协议。更具体的方法假设只约束该方法，不外推为全仓禁令。

## 几何、动力学与证据

- 基础非凸问题限于无洞、无自交的简单闭区域，默认按指定顺序各穿越一次。DynaTOGT 支持重复穿越；其他方法的重复穿越、闭合赛道和终态条件以各自协议为准。
- 使用真实非凸区域验证，不能用凸包替代。名义穿越点合法、投影角点合法、整机安全、动力学可行是不同结论；非凸边界中端点合法不保证整条边合法。
- 区分优化器收敛、离散/密集采样通过、连续域认证和实物实验。SC/MSR/RotSync 的采样结果不能写成 SIP 证书，历史 CWB 的 `NUMERICALLY_VERIFIED` 也不能升级为认证。
- SIP/Planar-RS 的 `CERTIFIED_FEASIBLE` 仅表示所认证名义模型与轨迹的全域证明。`VIOLATED`、`UNRESOLVED`、`NUMERICAL_FAILURE` 都不能解释为安全；没有认证候选不等于已证明全局不可行。
- 原始曲线实验必须把真实 Line/Arc/Bézier/B-spline 原语传给认证器，不得用用于 SC 映射的稠密折线冒充原始曲线证明。缺少 `python-flint` 时不能降级采样后宣称认证。
- PhaseGuard-RL 仅允许认证通过的候选替换执行轨迹；其当前实现是一步完整规划 PPO 核心，不能描述成已完成正式训练或真实飞控闭环。
- Planar-RS 仅支持固定中心、固定平面下的面内旋转/统一缩放；RotSync 进一步限定仅绕法向匀速旋转。不要直接用它们替代完整平移/RPY 动态窗口模型。
- 连续形变问题中物理开口非空不代表安全可通行。安全区允许暂时为空；窗口演化是外生过程，不能按无人机到达触发开放。穿越时刻、顺序、飞行时间和动力学必须联合满足。区分物理不可行、离散图无路径与当前求解预算未找到解。
- 实验比较遵守各自冻结协议，保留失败、超限和负结果，分别报告飞行时间、求解/认证耗时及训练代价。离线视频、Gazebo 场景适配不等于飞控接入、跟踪鲁棒性或真机认证。

## GAP-Step 局部约定

以下仅适用于 `gap_step/` 的纯 PPO 教师实验，不限制其他研究目录：

- 生成窗口迷宫入口是 `window_maze_env.py`、`train_window.py`、`evaluate_window.py`、`visualize_window.py`；旧 `env.py/train.py` 等入口仍保留，选入口时核对任务。
- 教师使用纯 PyTorch GNN，完整图/窗口动力学观测，连续二维加速度；观测接口为 `GraphObs(global_features, node_features, node_type, edge_index, edge_features)`，维度以源码为准。
- 显式旧策略采样、当前策略更新，更新后同步 `model_old <- model`；KL 使用标准非负近似，不用可能为负的 `mean(old_logp - new_logp)` 作为主 KL。
- progress reward 基于连续几何；撞墙或撞门不能保留正 progress reward。不得无依据更改碰撞、成功和迷宫生成语义。
- 纯 PPO 实验保持 planner/BC/专家辅助关闭。仓库实际保留 `window_planner.py`、`train_window_bc.py` 和可选 planner auxiliary 路径，不能说它们不存在，也不能仅因文件存在就启用。视觉学生、世界模型、主动感知、3D/四旋翼扩展不属于该二维实验的默认任务。

## 验证、文件与记忆维护

- 使用 conda 环境 `wyh`；依赖按方法 README/requirements 安装。根 `environment.yml` 是早期 GAP-Step 环境，不代表所有研究方法依赖齐全。
- 根 `pytest.ini` 的 `testpaths = gap_step/tests`，所以裸 `pytest -q` 只验证 GAP-Step。修改其他方法必须显式指定相关测试目录；文档改动检查链接、路径和约定一致性，不必重跑训练。
- 保留 `data/`、`checkpoints/`、`logs/`、`runs/`、`results/`、`复现/` 的忽略规则。不要删除或覆盖历史结果；SC 结果迁移沿用 dry-run、迁移清单和大小/SHA-256 校验。
- 用户的未提交文件是工作区现状，不能删除、覆盖或描述为已提交成果。论文方案、实验入口、已运行结果和验收结论分开记录。
- `AGENTS.md` 是代理规范源，修改后必须同步复制到 `CLAUDE.md`，两者内容完全一致。
- 当前状态写 `docs/PROJECT_CONTEXT.md`，待办写 `docs/ROADMAP.md`，变化依据写 `docs/DECISIONS.md`，已做工作写 `docs/TASK_LOG.md`。记录日期、提交或结果来源；历史快照只用于追溯，不作为当前指令。
