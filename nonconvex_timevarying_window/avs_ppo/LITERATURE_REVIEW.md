# 近两年文献依据与方法定位

检索截止 2026-08-25，只采用会议官方 proceedings / PMLR 页面。AVS-PPO 不是下列论文的逐行复现，而是针对动态非凸穿窗任务组合其中与问题匹配的原则。

## 直接支撑

1. Suttle et al., *Sampling-based Safe Reinforcement Learning for Nonlinear Dynamical Systems*, AISTATS 2024。论文强调把硬安全集合直接放进采样过程，并讨论“事后过滤动作”破坏底层 RL 收敛性质的问题；还在四旋翼避障中验证。AVS-PPO 对应采用状态依赖的 masked categorical 分布，使 PPO 的采样与更新分布一致。官方页面：<https://proceedings.mlr.press/v238/suttle24a.html>。

2. Fan et al., *Safety-Polarized and Prioritized Reinforcement Learning*, ICML 2025。MaxSafe 把安全置于回报之前，并给出 optimal action masking 的理论与大规模近似。AVS-PPO 不学习 MaxSafe 的 reachability Q，也不声称继承其定理；采用的是“先排除不可恢复动作，再在余下集合竞速”的 safety-first 排序。官方页面：<https://proceedings.mlr.press/v267/fan25i.html>。

3. Nguyen et al., *Gameplay Filters: Robust Zero-Shot Safety through Adversarial Imagination*, CoRL 2025。论文用预测 rollout 排除会在未来导致失败的动作，说明只检查单步局部安全不足。AVS-PPO 对应使用 candidate-plus-backup rollout，但本实现是已知低阶动力学和确定性窗口，不包含论文的对抗策略或 sim-to-real 误差模型。官方页面：<https://proceedings.mlr.press/v270/nguyen25a.html>。

4. Lavanakul et al., *Safety filters for black-box dynamical systems by learning discriminating hyperplanes*, L4DC 2024。论文明确提出把性能策略与可复用安全过滤器分离。AVS-PPO 同样把“快”和“安全”拆开，但安全集合来自解析动力学与真实多边形，而不是学习判别超平面。官方页面：<https://proceedings.mlr.press/v242/lavanakul24a.html>。

## 对照而非采用

Huang et al., *SafeDreamer: Safe Reinforcement Learning with World Models*, ICLR 2024，在世界模型规划中结合拉格朗日安全成本，并报告接近零成本。它适合未知/视觉动力学，但本任务已有精确低阶仿真器和解析非凸几何；用 learned world model 会额外引入模型误差，而且“近零成本”不等于逐状态零碰撞。因此本阶段选预测盾牌 PPO，不选 SafeDreamer 复现。官方页面：<https://proceedings.iclr.cc/paper_files/paper/2024/hash/ece182f93af26c64187ba3f7dfd4309a-Abstract-Conference.html>。

## 可发表性边界

当前贡献是一个任务特化、可复现的安全 RL 基线：真实非凸 membership、动态几何交点和可恢复动作 support 的耦合。它尚不足以宣称一般非线性四旋翼的安全定理。若继续做论文级工作，优先方向是：六自由度刚体/电机动力学、窗口运动不确定性集合、球体扫掠与厚门框连续域证书，以及与 MaxSafe、SafeDreamer 和无盾牌 PPO 的多种子统计比较。
