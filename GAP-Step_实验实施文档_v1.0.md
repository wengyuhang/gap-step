# GAP-Step 实验实施文档 v1.0

**对应方案版本**：GAP-Step v2.2  
**用途**：交给 Codex 或开发者实现最小可行实验系统  
**目标**：先做一个可运行、可训练、可复现实验闭环，而不是直接实现完整 3D 无人机系统  
**核心问题**：在视觉部分可观测条件下，智能体能否学会在多个时变窗口之间选择合适窗口与穿越时机。

---

## 1. 实验总目标

本实验不追求一次性实现完整三维四旋翼、RGB-D、多窗口主动感知和完整 SITT 机制。当前版本的目标是实现一个**二维或二点五维最小可行基准环境**，验证以下三个问题：

1. 视觉学生策略能否从图像历史中判断当前窗口是否适合穿越；
2. 多个候选窗口存在时，策略能否选择更安全、更容易通过的窗口；
3. 加入窗口状态辅助预测任务后，学生策略是否比纯端到端视觉强化学习更稳定、更容易训练。

当前实验的主方法是：

```text
特权教师演示 + 视觉学生行为克隆 + 窗口状态辅助预测 + 学生强化学习微调
```

---

## 2. 最小可行环境设计

### 2.1 环境类型

建议先实现二维平面环境：

```text
workspace = [0, W] × [0, H]
```

推荐默认参数：

| 参数 | 建议值 | 含义 |
|---|---:|---|
| `W` | 12.0 | 环境宽度 |
| `H` | 8.0 | 环境高度 |
| `dt` | 0.05 | 仿真步长 |
| `max_steps` | 400 | 单回合最大步数 |
| `robot_radius` | 0.18 | 智能体碰撞半径 |
| `max_speed` | 2.0 | 最大速度 |
| `max_acc` | 3.0 | 最大加速度 |
| `num_gates` | 1 或 2 | 初始建议从 1 到 2 |

环境中间有一堵墙，例如：

```text
x = W / 2
```

墙上有一个或多个窗口。智能体从左侧区域出发，目标在右侧区域。若想从起点到目标，必须穿越某个窗口。

---

### 2.2 智能体动力学

第一版使用二维双积分点质量模型，不直接建模四旋翼姿态：

$$
\mathbf{x}_t=[p_x,p_y,v_x,v_y]
$$

动作是二维加速度：

$$
\mathbf{a}_t=[a_x,a_y]
$$

状态更新：

$$
\mathbf{v}_{t+1}=\text{clip}(\mathbf{v}_t+\mathbf{a}_t\Delta t,-v_{max},v_{max})
$$

$$
\mathbf{p}_{t+1}=\mathbf{p}_t+\mathbf{v}_{t+1}\Delta t
$$

该简化是有意为之。当前阶段重点不是飞控，而是验证“窗口选择与时机决策”。

---

## 3. 时变窗口建模

### 3.1 窗口几何

第 `i` 个窗口定义为墙上的一个可通行区间：

$$
g_i(t)=[y_i,d_i(t),s_i(t)]
$$

| 符号 | 含义 |
|---|---|
| `y_i` | 窗口中心在墙上的纵向位置 |
| `d_i(t)` | 当前窗口开口宽度 |
| `s_i(t)` | 当前是否安全可穿越，二值变量 |

其中：

$$
s_i(t)=\mathbb{I}[d_i(t)>2r+m]
$$

`r` 是智能体半径，`m` 是安全裕度。

推荐：

```text
safe_margin = 0.10
min_width = 0.0
max_width = 1.2
```

---

### 3.2 窗口动态

第一版只实现周期性变化：

$$
d_i(t)=d_{min}+\frac{d_{max}-d_{min}}{2}\left[1+\sin(\omega_i t+\phi_i)\right]
$$

推荐参数随机化：

| 参数 | 采样范围 |
|---|---|
| `d_min` | 0.0–0.2 |
| `d_max` | 0.9–1.4 |
| `omega_i` | 0.8–1.8 |
| `phi_i` | 0–2π |
| `y_i` | 避开边界随机采样 |

第二阶段可以加入非周期扰动：

$$
d_i(t)=d_i^{periodic}(t)+\epsilon_t
$$

但最小版本不要一开始加入噪声。

---

## 4. 观测设计

### 4.1 教师观测

教师使用特权状态，但**不允许看到未来窗口状态**。

教师观测：

$$
o_t^T=[p_x,p_y,v_x,v_y,\{y_i,d_i(t),s_i(t)\}_{i=1}^{N},p_{goal}]
$$

教师可以看到当前窗口真实宽度和当前是否安全，但不能看到 `d_i(t+Δt)`。

---

### 4.2 学生观测

学生使用视觉观测和自身状态：

```text
student_obs = image_obs + proprio_obs
```

建议第一版使用灰度栅格图像：

| 项 | 建议 |
|---|---|
| 图像尺寸 | 64×64 |
| 通道数 | 1 或 3 |
| 内容 | 墙、窗口、智能体局部视野、目标方向 |
| 视野 | 可先用全局俯视图，后续再改局部视野 |

自身状态：

$$
o_t^{prop}=[v_x,v_y,p_{goal}-p_t,a_{t-1}]
$$

为感知窗口变化，建议使用最近 `K_obs=4` 帧图像堆叠，而不是单帧图像。

---

## 5. 动作空间设计

当前版本不使用显式 `wait / observe / align / commit` 层级动作。

主策略输出：

```text
gate_choice: Discrete(num_gates)
acceleration: Box(low=-max_acc, high=max_acc, shape=(2,))
```

即：

$$
a_t=(w_t,\mathbf{a}_t)
$$

| 动作分量 | 含义 |
|---|---|
| `w_t` | 当前策略选择的目标窗口编号，用于可解释性和辅助训练 |
| `a_t` | 实际执行的二维加速度 |

注意：环境真正执行的是连续加速度。`gate_choice` 不直接改变动力学，只用于：

1. 训练策略形成窗口选择能力；
2. 计算部分诊断指标；
3. 让网络显式表达“我现在想走哪个窗口”。

如果实现混合动作 PPO 太复杂，可以第一阶段先只执行 `a_t`，把 `gate_choice` 作为辅助分类头训练，不输入环境 step。

---

## 6. 特权教师设计

### 6.1 教师类型

第一版不训练教师，使用规则教师即可。

规则教师的作用是生成演示数据，而不是作为最终方法贡献。

### 6.2 教师决策逻辑

每一步教师做三件事：

1. 读取所有窗口当前宽度 `d_i(t)` 和安全标志 `s_i(t)`；
2. 选择一个当前或即将可接近的窗口；
3. 输出朝向该窗口入口点的连续加速度。

规则示例：

```python
if any gate is currently safe:
    choose gate with smallest estimated path cost
    target = entry_point_of_chosen_gate
else:
    choose gate with largest current width
    target = staging_point_before_that_gate

acc = PD_control(current_state, target)
```

入口点定义为墙左侧、窗口中心前方一个固定距离：

$$
p_{entry,i}=[W/2-d_{entry}, y_i]
$$

出口点定义为墙右侧、窗口中心后方一个固定距离：

$$
p_{exit,i}=[W/2+d_{exit}, y_i]
$$

### 6.3 教师输出

教师输出包括：

```text
teacher_gate: int
teacher_acc: float[2]
teacher_safe_labels: s_i(t)
teacher_width_labels: d_i(t)
```

其中 `teacher_safe_labels` 和 `teacher_width_labels` 用作学生辅助预测任务的监督信号。

---

## 7. 学生网络设计

### 7.1 输入

学生输入：

```text
image_stack: [K_obs, C, H, W]
proprio: [v_x, v_y, goal_dx, goal_dy, prev_ax, prev_ay]
```

### 7.2 网络结构

建议结构：

```text
CNN(image_stack) → visual_feature
MLP(proprio) → state_feature
concat → GRU 或 MLP → latent z
```

输出 heads：

1. 连续控制 head：输出二维加速度均值 `mu_acc`；
2. 窗口选择 head：输出 `gate_logits`；
3. 窗口宽度预测 head：输出 `d_hat_i(t)`；
4. 窗口安全预测 head：输出 `s_hat_i(t)`。

### 7.3 损失函数

行为克隆损失：

$$
\mathcal{L}_{BC}=\|\hat{a}_t-a_t^T\|_2^2+\lambda_g CE(\hat{w}_t,w_t^T)
$$

窗口宽度预测损失：

$$
\mathcal{L}_{width}=\sum_i\|\hat{d}_i(t)-d_i(t)\|_1
$$

窗口安全预测损失：

$$
\mathcal{L}_{safe}=\sum_i BCE(\hat{s}_i(t),s_i(t))
$$

总监督损失：

$$
\mathcal{L}_{sup}=\mathcal{L}_{BC}+\lambda_w\mathcal{L}_{width}+\lambda_s\mathcal{L}_{safe}
$$

推荐初始权重：

```text
lambda_g = 0.5
lambda_w = 1.0
lambda_s = 1.0
```

---

## 8. 强化学习微调

### 8.1 奖励函数

学生行为克隆后，再进行 PPO 或自定义 actor-critic 微调。

奖励项保持简单：

$$
r_t=r_{goal}+r_{cross}+r_{progress}-r_{collision}-r_{time}-r_{action}
$$

推荐定义：

| 奖励项 | 建议值 | 含义 |
|---|---:|---|
| `r_goal` | +10 | 到达目标 |
| `r_cross` | +3 | 成功穿越窗口 |
| `r_progress` | 0.1 × 距离减少 | 朝目标前进 |
| `r_collision` | -10 | 撞墙或撞窗口边界 |
| `r_time` | -0.01 | 每步时间惩罚 |
| `r_action` | -0.001 × `||a||²` | 动作平滑 |

不建议第一版加入复杂信息增益奖励。

### 8.2 PPO 注意事项

如果 Codex 实现自定义 PPO，需要支持：

```text
Gaussian continuous action + optional categorical gate head
```

更简单的第一版：

- PPO 只优化连续加速度；
- `gate_choice` 作为辅助 head，不作为环境动作；
- 辅助预测损失继续保留。

---

## 9. 训练流程

### 9.1 阶段 A：教师演示数据生成

生成 `N_demo` 条专家轨迹。

推荐：

```text
N_demo = 5000 episodes
train/val split = 90% / 10%
```

每条样本包含：

```text
image_stack
proprio
teacher_acc
teacher_gate
true_gate_widths
true_gate_safe_flags
reward/done/info
```

### 9.2 阶段 B：学生监督预训练

训练学生网络：

```text
input → student network → acc, gate, width, safe
```

优化：

```text
BC loss + width loss + safe loss
```

### 9.3 阶段 C：学生 RL 微调

加载预训练学生权重，在环境中继续用 PPO 微调。

建议保留辅助损失：

$$
\mathcal{L}=\mathcal{L}_{PPO}+\alpha(\mathcal{L}_{width}+\mathcal{L}_{safe})
$$

其中：

```text
alpha = 0.1
```

### 9.4 阶段 D：评估

固定随机种子，在不同难度场景中评估。

推荐每个 setting：

```text
num_eval_episodes = 300
seeds = [0, 1, 2]
```

---

## 10. 实验设置

### 10.1 难度等级

| 等级 | 场景 | 目的 |
|---|---|---|
| E1 | 单窗口，固定开放 | 检查控制和穿越逻辑 |
| E2 | 单窗口，周期变化 | 检查时机判断 |
| E3 | 双窗口，异步变化 | 检查窗口选择 |
| E4 | 双窗口 + 局部视觉 | 检查部分可观测性 |
| E5 | 双窗口 + 随机相位泛化 | 检查泛化能力 |

### 10.2 Baseline

至少实现以下方法：

| 方法 | 描述 | 目的 |
|---|---|---|
| `Teacher-Heuristic` | 特权规则教师 | 上界参考 |
| `Visual-PPO` | 视觉输入直接 PPO | 端到端 RL 对照 |
| `BC-only` | 只行为克隆教师动作 | 检查模仿学习能力 |
| `BC+Aux` | 行为克隆 + 窗口状态辅助预测 | 验证辅助预测作用 |
| `BC+Aux+PPO` | 主方法 | 验证 RL 微调提升 |
| `Oracle-PPO` | 使用真实窗口状态的 PPO | privileged upper bound，可选 |

### 10.3 主要指标

| 指标 | 含义 |
|---|---|
| `Success Rate` | 成功到达目标比例 |
| `Collision Rate` | 碰撞比例 |
| `Crossing Success Rate` | 成功穿越窗口比例 |
| `Closed-Gate Attempt Rate` | 尝试穿越关闭窗口比例 |
| `Time-to-Goal` | 到达目标平均步数 |
| `Gate Choice Accuracy` | 学生窗口选择与教师一致率 |
| `Width MAE` | 窗口宽度预测误差 |
| `Safe F1 / AUROC` | 安全可穿越预测质量 |
| `Return` | 平均回报 |

---

## 11. 消融实验

必须做以下消融：

### 11.1 去掉窗口宽度预测

方法：只保留行为克隆，不预测 `d_i(t)`。

目的：验证连续窗口状态预测是否帮助时机判断。

### 11.2 去掉窗口安全预测

方法：不预测 `s_i(t)`。

目的：验证二值安全可穿越监督是否有效。

### 11.3 去掉 RL 微调

方法：只用 `BC+Aux`。

目的：验证 PPO 微调是否提升闭环表现。

### 11.4 去掉图像历史堆叠

方法：只输入单帧图像。

目的：验证动态窗口任务是否需要历史信息。

---

## 12. 预期结果

合理预期如下：

1. `Teacher-Heuristic` 在 E1–E3 中成功率应达到 85% 以上，否则环境或教师规则有问题。
2. `Visual-PPO` 在 E2/E3 中训练较慢，稳定性较差。
3. `BC-only` 能学到基本靠近窗口，但在时变窗口场景中可能闭环失败。
4. `BC+Aux` 应明显提升窗口状态判断能力。
5. `BC+Aux+PPO` 应在成功率、碰撞率和关闭窗口误穿率上优于 `BC-only` 与 `Visual-PPO`。
6. 单帧图像版本在动态窗口场景中应明显弱于图像历史堆叠版本。

---

## 13. 最小验收标准

Codex 实现完成后，至少应满足：

1. 可以通过一条命令创建 conda 环境；
2. 可以运行环境随机策略并可视化；
3. 可以生成教师演示数据；
4. 可以训练 `BC-only` 和 `BC+Aux`；
5. 可以执行 PPO 微调；
6. 可以运行评估脚本并输出 CSV 或 JSON 结果；
7. 可以保存训练曲线、模型 checkpoint 和 rollout 视频；
8. README 中写明完整运行命令。

---

## 14. 推荐代码结构

```text
gap_step/
  envs/
    gap_step_env.py
    gate_dynamics.py
    renderer.py
  teachers/
    heuristic_teacher.py
  models/
    student_policy.py
    cnn_encoder.py
  trainers/
    generate_demos.py
    train_bc.py
    train_ppo.py
    evaluate.py
  configs/
    env_e1.yaml
    env_e2.yaml
    env_e3.yaml
    train_bc.yaml
    train_ppo.yaml
  scripts/
    render_random_policy.py
    render_trained_policy.py
  tests/
    test_env.py
    test_gate_dynamics.py
    test_teacher.py
  README.md
  environment.yml
```

---

## 15. 给开发者的实现顺序

1. 实现 `GateDynamics`，能返回任意时刻窗口宽度和安全标志；
2. 实现 `GapStepEnv`，支持 reset、step、render；
3. 实现随机策略可视化；
4. 实现规则教师；
5. 生成演示数据并保存为 `.npz` 或 `.pt`；
6. 实现学生 CNN 策略；
7. 训练 `BC-only`；
8. 加入 `width` 和 `safe` 辅助预测；
9. 加入 PPO 微调；
10. 实现评估脚本和 baseline 对比。

---

## 16. 暂不实现的内容

以下内容不要在第一版实现：

1. 真实 3D 四旋翼动力学；
2. RGB-D 真实渲染；
3. 主动相机 yaw/pitch 控制；
4. 完整 SITT 代理学生机制；
5. 未来可通行性矩阵 `p_i(Delta t_k | h_t)`；
6. 世界模型或未来视频预测；
7. 大规模复杂地图。

这些内容作为后续扩展，而不是当前版本目标。
