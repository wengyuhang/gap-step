# 2024--2025 相关文献与方法选择

检索截止日为 2026-08-23；以下只把论文本身支持的性质用于方法选择，不将相邻工作的收敛结论直接移植为本实现的定理。

## Provably Feasible SIP via Subdivision（2024）

Zhang 等在 *IEEE Transactions on Robotics* 的 [Provably Feasible Semi-Infinite Program Under Collision Constraints via Subdivision](https://doi.org/10.1109/TRO.2024.3391649) 直接指出：将连续碰撞约束离散为有限采样会遗漏采样间碰撞。其核心是用保守函数界与参数域细分构造可行性证明。

SIP-DynaTOGT 采用同一原则：SLSQP 采样点只负责产生候选，最终结论由完整参数域上的区间细分决定。本项目的特殊难点是约束中还包含时变 RPY 门框、MINCO 导数和四旋翼平坦性姿态。

## quADAPT 局部解收敛性（2024）

Seidel 与 Küfer 的 [On the convergence of local solutions for the method quADAPT](https://doi.org/10.1080/02331934.2024.2372386) 分析了自适应离散、最违规约束回填及局部解的收敛性。论文也指出，若缺少附加条件，迭代极限未必是原 SIP 的局部解。这同时支持了 witness 回填设计，也要求本项目不将 SLSQP 的局部终止状态当作安全证明。

本实现对其进行了保守化：局部最坏点可用于修正候选，但不能单独生成安全结论；结论仍需要 Arb 完整覆盖。

## Continuous-time SCvx（2025）

Elango 等的 [Continuous-time successive convexification for constrained trajectory optimization](https://doi.org/10.1016/j.automatica.2025.112464) 研究了连续时间路径约束及其积分重写，并讨论未松弛积分等式导致的约束资格退化。这与附件中 `H_ij=0` 的实际求解风险一致：等价性是分析结论，不代表有限求积是可靠证书。

SIP-DynaTOGT 因此不将积分等式交给 SLSQP，而是保留原始点态不等式。

## Local-reduction SIP（2025）

Gao 等的 [Semi-Infinite Programming for Collision-Avoidance in Optimal and Model-Predictive Control](https://arxiv.org/abs/2508.12335) 将 local reduction 和外部活动集结合，说明半无限碰撞约束可以用少量当前活动点驱动有限优化。

本项目不实现完整 local-reduction 框架，只保留必要部分：违规点回填和温启动。这避免了对一个低维 `[K,D]` 问题过度设计。

## 区间算术依赖

[python-flint / Arb](https://python-flint.readthedocs.io/en/latest/) 使用 midpoint-radius 球算术并向外舍入。SIP-DynaTOGT 将浮点输入按其精确二进制有理值注入 Arb，避免了「用普通 float 计算上界，再人为加 epsilon」的非证明做法。

## 结论

近两年文献给出了两个相容但不同的目标：

1. 活动集/自适应离散用于高效产生局部优化候选；
2. 保守函数界与完整域细分用于安全证明。

SIP-DynaTOGT 只组合这两点，不额外引入多套求解器、世界模型或全局最优声明。
