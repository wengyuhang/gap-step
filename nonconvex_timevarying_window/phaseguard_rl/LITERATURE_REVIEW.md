# 2025–2026 精简文献依据

本算法只采用两条直接相关的结论，不再融合大量方法。

1. [Precise Aggressive Aerial Maneuvers with Sensorimotor Policies](https://arxiv.org/abs/2604.05828)，
   2026：RL 能自主选择窄缝穿越状态和倾斜姿态，而不必预先指定窗口中心。
2. [Learning Agile Gate Traversal via Analytical Optimal Policy Gradient](https://arxiv.org/abs/2508.21592)，
   2025：让网络输出高层参考状态、再由固定模型控制器执行，比直接输出电机命令更容易训练和解释。

因此本方法只让 Actor 输出穿越点和到达时间。连续安全检查不是从论文中额外拼接的新控制器，而是
原任务“必须保证连续整机安全”的执行门槛：检查通过才飞，检查失败或无法确定就不飞。

两篇论文都不能推出本任务的严格安全或全局时间最优；这两个结论不会写进本方法的结果。

