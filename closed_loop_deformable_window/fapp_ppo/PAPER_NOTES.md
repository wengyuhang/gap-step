# 2026 顶会论文调研与设计映射

本实现不是下列论文的复现。这里只记录论文中与本问题直接相关、且已经落到代码里的设计选择。

## 1. ICRA 2026：特权信息四旋翼导航

Jonathan Lee 等，*Quadrotor Navigation using Reinforcement Learning with Privileged Information*，ICRA 2026。

- 论文用 time-of-arrival map 的负梯度提供绕障目标速度和稠密训练信号；
- 本项目的窗口未来动态完全已知，因此改为“未来窗口预览 + 预测到达时刻的安全锚点势函数”；
- 对 actor 只提供有限时域可部署预览，对 critic 提供整条剩余窗口序列，形成非对称 actor–critic。

链接：

- https://arxiv.org/abs/2509.08177
- https://ras.papercept.net/conferences/conferences/ICRA26/program/ICRA26_ContentListWeb_3.html

## 2. ICRA 2026：SimpleFlight

Jiayu Chen 等，*What Matters in Learning A Zero-Shot Sim-to-Real RL Policy for Quadrotor Control? A Comprehensive Study*，RA-L 2025，ICRA 2026 展示。

- 论文的系统实验支持把线速度和旋转矩阵放入 actor 输入；
- 绝对时间只进入 critic，避免 actor 在超出训练时长时发生时间输入 OOD；
- 奖励包含相邻动作差平滑项；
- 本实现采用相同的三项输入/奖励原则，并使用显式旧策略 PPO。

链接：

- https://arxiv.org/abs/2412.11764
- https://github.com/thu-uav/SimpleFlight

## 3. RSS 2026：LAFR

Yunfan Ren 等，*Learning Agile Quadrotor Flight in the Real World*，RSS 2026。

- 论文用在线残差动力学、短时域可微优化和 Adaptive Temporal Scaling 逐步逼近飞行极限；
- 当前问题仅要求仿真，所以本实现不复制真实在线适应环；
- 本实现借鉴“从保守可行行为逐步提高时间尺度难度”的思想，采用静态/平移/完整形变课程，并让 PPO 学习名义预览控制器上的 CTBR 残差。

链接：

- https://arxiv.org/abs/2602.10111
- https://rpg.ifi.uzh.ch/lafr/

## 采用与未采用

已经采用：

- 未来已知几何的特权学习信号；
- actor/critic 信息非对称；
- \(R\)、\(v\) 输入，绝对时间只给 critic；
- 动作变化平滑；
- 残差策略和时间/运动难度课程；
- 四旋翼 CTBR 命令、刚体动力学和单旋翼推力分配。

当前未采用：

- 真实视觉输入、Sim2Real 和在线真机适应；
- 论文中的 ToA 栅格/FMM；
- LAFR 的 RASH-BPTT；
- 连续时间形式化安全证书；
- 对全局时间最优性的证明。

