# PhaseGuard-RL（精简版）

这个版本只实现用户提出的核心结构：

- 强化学习网络读取无人机状态和窗口相位；
- 网络自主选择非中心穿越点和飞行时间；
- 固定底层控制器执行生成的七阶轨迹；
- 新轨迹只有通过连续时间整机与动力学检查才能执行；
- PPO 用总飞行时间作为主要优化目标。

不再组合 HOCBF、MPC 参数学习、可达管、复杂备份控制和自动课程系统。

安全保证限定于仓库问题定义中的已知名义模型。VIOLATED 和 UNRESOLVED 都不会被下发执行；
只有 CERTIFIED_FEASIBLE 可以进入底层控制器。

文件：

- [ALGORITHM.md](ALGORITHM.md)：精简算法；
- [LITERATURE_REVIEW.md](LITERATURE_REVIEW.md)：只保留直接相关的 2025–2026 文献；
- [EXPERIMENT_PROTOCOL.md](EXPERIMENT_PROTOCOL.md)：最小实验要求。

核心代码已经实现：

- model.py：相位观测和连续 Actor-Critic；
- planner.py：穿越点/时间到固定 MINCO 轨迹；
- shield.py：只有完整连续认证通过才替换当前轨迹；
- environment.py、train.py、ppo.py：最小的一步完整规划 PPO 训练；
- tests/：动作、轨迹、禁止未认证起飞、fail-closed 切换和 PPO 更新测试。

这里复用仓库中已经验证过的原始曲线连续检查函数，但不调用 SIP 优化器。缺少 python-flint 时检查
返回失败并禁止执行。仓库标准环境为 conda 环境 wyh。

当前状态是核心实现完成，尚未进行正式场景训练和性能实验。
