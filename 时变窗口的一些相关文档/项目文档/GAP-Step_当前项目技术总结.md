# GAP-Step 当前项目技术总结

本文档是对当前 GAP-Step 最小可行实验系统的阶段性总结，重点说明项目目前到底实现了什么、实验环境如何构造、教师和学生分别是什么、训练与评估流程如何工作，以及当前结果能够说明什么、不能说明什么。

## 1. 项目定位

GAP-Step 当前版本是一个二维 / 2.5D 动态多窗口穿越实验系统，用来验证以下方法链路是否可行：

```text
特权教师演示
    -> 视觉学生行为克隆 BC
    -> 窗口状态辅助预测 Aux
    -> PPO 微调
```

需要明确的是，当前实现不是完整 3D 四旋翼仿真，也不是机载第一视角视觉系统。学生看到的图像是二维俯视渲染图，也就是 top-down raster observation。它更像是把环境状态画成一张小图后交给 CNN，而不是模拟无人机前向相机看到的墙面和窗口。

因此当前项目适合验证“动态窗口选择、穿越时机判断、特权教师到视觉学生蒸馏”这条最小链路，但还不能直接声称已经实现真实无人机机载视觉策略。

## 2. 实验环境

环境是一个二维平面 workspace，智能体从左侧出发，目标位于右侧，中间有一堵竖直墙：

```text
workspace = [0, W] x [0, H]
wall_x = W / 2
```

墙上有一个或多个动态窗口。窗口由中心高度 `y_i` 和当前宽度 `d_i(t)` 描述。智能体是一个半径为 `robot_radius` 的圆盘，必须通过窗口穿过墙，最后到达目标点。

当前设置了三个难度：

| 场景 | 配置 | 含义 |
|---|---|---|
| E1 | `configs/env_e1.yaml` | 单窗口，固定开放 |
| E2 | `configs/env_e2.yaml` | 单窗口，周期变化 |
| E3 | `configs/env_e3.yaml` | 双窗口，异步周期变化 |

动态窗口宽度按正弦函数变化：

```text
d_i(t) = d_min + (d_max - d_min) / 2 * (1 + sin(omega_i * t + phi_i))
```

窗口是否安全由当前宽度决定：

```text
safe_i(t) = 1[d_i(t) > 2 * robot_radius + safe_margin]
```

当前训练和评估并不是每个 episode 随机生成新的窗口环境。窗口数量、中心位置、频率、相位等都来自固定 yaml 配置。随机性主要来自 reset 时的起点扰动、不同 seed，以及轨迹进入窗口时刻不同导致遇到的窗口相位不同。

所以当前实验测的是固定 E1/E2/E3 场景内的鲁棒性，不是任意窗口布局或任意窗口数量的泛化能力。

## 3. 观测与动作

学生策略输入由两部分组成：

```text
image_stack + proprio
```

`image_stack` 是最近 `K_obs=4` 帧灰度图像堆叠，默认尺寸为：

```text
shape = [4, 64, 64]
```

这张图是二维俯视图。画面中包含墙、窗口、智能体、目标和轨迹等信息。它不是无人机前向相机图像。因此当前学生策略不是纯机载第一视角视觉策略。

`proprio` 是低维本体 / 导航信息，默认维度为 6，主要包含：

```text
velocity_x
velocity_y
goal_relative_x
goal_relative_y
last_action_x
last_action_y
```

学生知道终点方向，是因为 `proprio` 中直接提供了目标相对向量 `goal - position`。图像主要用于感知墙和窗口状态，低维向量告诉策略“目标在哪个方向”。

动作是二维连续加速度：

```text
action = [a_x, a_y]
```

环境使用二维双积分动力学更新智能体位置和速度。

## 4. 教师策略

教师位于：

```text
gap_step/teachers/heuristic_teacher.py
```

当前教师不是神经网络，也没有训练过程和 checkpoint。它是手写启发式策略，直接读取环境真值状态，也就是 privileged state。

教师可用的信息包括：

```text
智能体当前位置
当前速度
目标位置
墙位置
每个窗口的中心高度
每个窗口当前宽度
每个窗口当前安全标志
```

教师决策逻辑可以概括为：

```text
如果存在当前安全窗口：
    选择路径代价较低的安全窗口
否则：
    选择当前宽度最大的窗口
    在墙前 staging point 附近等待或靠近
```

教师只看当前窗口状态，不使用未来窗口预测。它每一步输出：

```text
teacher_acc
teacher_gate
```

其中 `teacher_acc` 是二维加速度示范，`teacher_gate` 是教师选择的窗口编号。

## 5. 教师演示数据

教师演示数据由以下脚本生成：

```text
trainers/generate_demos.py
```

生成流程是：

```text
创建环境
创建 HeuristicTeacher
循环多个 episode
    reset 环境
    教师根据真值状态输出动作
    保存学生输入、教师动作和辅助标签
```

保存出的文件包括：

```text
data/e1/demos.npz
data/e2/demos.npz
data/e3/demos.npz
```

每个 sample 保存字段包括：

| 字段 | 含义 |
|---|---|
| `image_stack` | 学生视觉输入，最近 4 帧 64x64 灰度俯视图 |
| `proprio` | 速度、目标相对向量、上一动作 |
| `teacher_acc` | 教师输出的二维加速度 |
| `teacher_gate` | 教师选择的窗口编号 |
| `true_widths` | 每个窗口当前真实宽度 |
| `true_safe_flags` | 每个窗口当前是否安全 |
| `train_idx` / `val_idx` | 训练 / 验证划分索引 |

当前 E3 使用 2000 条教师 episode，生成了约 221k 个 step samples。`data/e3/demos.npz` 约 14G。数据大的主要原因是 `image_stack` 以 `float32` 形式未压缩保存：

```text
221105 samples * 4 frames * 64 * 64 * 4 bytes ≈ 14GB
```

后续可以通过以下方式显著减小数据：

```text
将 image_stack 存成 uint8
使用 np.savez_compressed
只存单帧，训练时动态 stack
减少演示 episode 数量
```

## 6. 学生网络

学生网络位于：

```text
gap_step/models/student_policy.py
gap_step/models/cnn_encoder.py
```

整体结构是 CNN + MLP 的多头策略网络：

```text
image_stack
    -> CNN encoder
    -> image feature

proprio
    -> MLP encoder
    -> proprio feature

image feature + proprio feature
    -> fusion MLP
    -> shared latent
    -> 多个输出 head
```

输出 head 包括：

| Head | 输出 | 用途 |
|---|---|---|
| `acc` | `[a_x, a_y]` | 实际控制动作 |
| `gate_logits` | `num_gates` 分类 logits | 模仿教师窗口选择 |
| `width` | `num_gates` 个宽度预测 | Aux 辅助任务 |
| `safe_logits` | `num_gates` 个安全性 logits | Aux 辅助任务 |
| `value` | 状态价值 | PPO actor-critic 训练 |

当前没有 RNN、LSTM、GRU、Transformer 或 temporal attention 这类显式历史编码器。历史信息只通过 `K_obs=4` 帧图像堆叠提供。也就是说，CNN 把 4 帧当成 4 个输入通道一次性处理，而不是逐帧建模时间序列。

另外，当前 `gate_logits`、`width`、`safe_logits` 的输出维度都绑定 `num_gates`。因此 E3 训练出的双窗口模型不能直接迁移到 3 个、5 个或任意数量窗口的环境。若要支持任意窗口数，需要引入 mask、固定最大窗口数，或 set-based / per-window shared scorer 结构。

## 7. Aux 是什么

`Aux` 指 auxiliary loss / auxiliary task，即辅助任务。

在当前项目中，BC-only 和 BC+Aux 的网络结构相同，区别在 loss 权重：

```text
BC-only:
    训练 acc 模仿
    训练 gate 模仿
    不训练 width/safe 辅助预测

BC+Aux:
    训练 acc 模仿
    训练 gate 模仿
    额外训练 width 预测
    额外训练 safe 预测
```

BC loss 为：

```text
L_BC = MSE(acc_pred, teacher_acc)
     + lambda_g * CE(gate_pred, teacher_gate)
```

辅助 loss 为：

```text
L_Aux = lambda_w * L1(width_pred, true_widths)
      + lambda_s * BCE(safe_pred, true_safe_flags)
```

总 loss 为：

```text
L_total = L_BC + L_Aux
```

在 `bc_only` 模式中，`lambda_w` 和 `lambda_s` 被置为 0；在 `bc_aux` 模式中，它们使用配置文件中的权重。Aux 的目的不是直接控制，而是逼学生从图像中理解窗口当前宽度和安全性。

## 8. 训练流程

当前训练流程主要由三个脚本构成：

```text
trainers/generate_demos.py
trainers/train_bc.py
trainers/train_ppo.py
```

完整链路为：

```text
1. generate_demos.py
   用 HeuristicTeacher 生成 demos.npz

2. train_bc.py
   训练 BC-only
   训练 BC+Aux

3. train_ppo.py
   从 BC+Aux checkpoint 初始化，做 PPO 微调
   或从 scratch 训练 Visual-PPO baseline
```

几个模型含义如下：

| 名称 | 含义 |
|---|---|
| `Teacher-Heuristic` | 手写特权教师，不训练 |
| `BC-only` | 只行为克隆教师动作和窗口选择 |
| `BC+Aux` | 行为克隆 + 宽度/安全性辅助预测 |
| `Visual-PPO` | 不使用 BC 初始化，从零 PPO 训练 |
| `BC+Aux+PPO` | 从 BC+Aux 初始化后 PPO 微调 |

PPO 当前是一个最小可运行版本，主要优化连续加速度策略。`gate head` 不直接作为环境动作使用，而是用于模仿、诊断和评估。

## 9. 评估流程

评估脚本为：

```text
trainers/evaluate.py
```

它固定读取某个环境配置，对每个候选模型跑 `episodes` 次 rollout，然后统计平均指标。我们当前 E2/E3 的正式评估使用：

```text
episodes = 100
```

每个模型使用同一组 reset seeds：

```text
episode 0 -> seed 0
episode 1 -> seed 1
...
episode 99 -> seed 99
```

因此不同模型之间比较是公平的。但需要注意：评估并不是随机生成 100 个不同窗口环境，而是在固定 E1/E2/E3 配置里，用 100 个 reset seed 测试稳定性。

评估指标包括：

| 指标 | 含义 |
|---|---|
| `success_rate` | 到达目标比例 |
| `collision_rate` | 碰撞比例 |
| `crossing_success_rate` | 成功穿墙比例 |
| `closed_gate_attempt_rate` | 尝试穿关闭/不安全窗口比例 |
| `time_to_goal` | 成功 episode 的平均到达步数 |
| `gate_choice_accuracy` | 学生 gate head 与教师选择的一致率 |
| `width_mae` | 窗口宽度预测误差 |
| `safe_f1` | 安全性预测 F1 |
| `return` | 平均回报 |

当前结果文件包括：

```text
logs/e1_eval.csv
logs/e2_eval.csv
logs/e3_eval.csv
```

## 10. 当前实验结果解读

E1/E2 中，BC+Aux 明显提升窗口宽度和安全性预测，同时控制成功率保持 1.0。这说明在简单单窗口任务中，辅助任务确实帮助学生学习窗口状态表征，并没有破坏控制。

E3 中结果更有意思：

```text
BC-only:
    success_rate = 1.0
    width_mae 较大
    safe_f1 较低

BC+Aux:
    success_rate = 0.0
    width_mae 很低
    safe_f1 接近 1.0

BC+Aux+PPO:
    success_rate = 1.0
    width/safe 预测仍然很好
```

这说明 BC+Aux 在 E3 中确实学会了窗口状态预测，但直接控制策略失败了。这通常可以理解为多任务训练中的任务竞争或损失权衡问题：辅助任务学得好，并不保证主控制任务更好。

PPO 微调后，BC+Aux+PPO 成功率恢复到 1.0，说明 PPO 利用环境 reward 修正了控制策略，同时保留了较好的窗口状态预测能力。

Visual-PPO 从零训练表现较弱，这是合理的。当前 PPO 预算有限，且没有向量化环境、curriculum、KL early stopping、action std annealing 等更完整的强化学习稳定化机制。从零 PPO 在这种动态穿越任务上更难学。

## 11. 可视化结果

当前已有 E3 rollout GIF：

```text
runs/e3_bc_only.gif
runs/e3_bc_aux.gif
runs/e3_bc_aux_ppo.gif
runs/e3_visual_ppo.gif
```

观察结果与评估表一致：

```text
BC-only:
    能穿过窗口并到达目标

BC+Aux:
    在窗口/墙附近失败，和 collision_rate 高一致

BC+Aux+PPO:
    PPO 后恢复正常穿越并到达目标

Visual-PPO:
    从零训练效果较弱，容易在左侧附近徘徊或失败
```

## 12. 当前局限

当前项目已经跑通了最小链路，但有几个重要局限：

1. 学生视觉不是无人机第一视角。
   当前图像是二维俯视图，不是前向机载相机图像。

2. 教师不是学习得到的。
   教师是手写启发式策略，直接读取特权真值状态。

3. 当前训练和评估不测试任意窗口泛化。
   E1/E2/E3 都是固定配置，窗口数量、位置、频率和相位不是每个 episode 随机采样。

4. 当前学生网络不支持任意窗口数。
   输出 head 维度绑定 `num_gates`，双窗口模型不能直接用于三窗口或更多窗口。

5. 数据存储效率较低。
   `image_stack` 以 `float32` 未压缩保存，导致 E3 数据达到 14G。

6. PPO 是最小可运行版本。
   当前没有 best rollout 可视化筛选、KL early stopping、vectorized envs、curriculum 或更系统的超参搜索。

## 13. 下一步建议

如果继续推进项目，建议优先做以下几件事：

1. 改成前视 / 自我中心相机渲染。
   让学生看到更接近无人机机载相机的图像，而不是 top-down 俯视图。

2. 做环境随机化。
   每个 episode 随机采样窗口数量、位置、频率、相位、开口范围、起点和目标，评估真正的泛化能力。

3. 支持多窗口可变数量。
   短期可用 `max_gates + mask`，长期可考虑 per-window shared scorer 或 set-based encoder。

4. 做数据量消融。
   比较 E3 在 100、200、500、1000、2000 episodes 下的训练效果，避免盲目堆数据。

5. 优化数据格式。
   将图像存成 `uint8`，使用压缩保存，或只存单帧并在 Dataset 中动态构造 stack。

6. 稳定 PPO。
   加入更保守的学习率、KL 监控、best checkpoint 选择和更多评估回滚机制。

## 14. 阶段性结论

当前 GAP-Step 项目已经验证了一个可运行的最小研究链路：

```text
特权教师可以生成有效演示
视觉学生可以通过 BC 学习动态窗口穿越
Aux 可以显著提升窗口状态预测
PPO 可以在 E3 中修正 BC+Aux 的控制失败
```

但当前结果应被理解为固定二维场景下的最小可行验证，而不是完整无人机机载视觉泛化系统。后续如果要支撑更强的研究结论，需要把视觉观测、环境随机化、可变窗口数和评估协议进一步升级。

