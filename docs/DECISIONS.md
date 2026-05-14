# Decisions

## 2026-05-11：只训练 PPO 特权教师

决定：当前阶段只保留连续二维时变窗口迷宫中的 PPO 特权教师。

原因：先得到可靠 teacher，再考虑学生、视觉或其他模块。

## 2026-05-11：可运行代码只放在 `gap_step/`

决定：不再使用 `trainers/`、`scripts/`、`gap_step/envs/`、`gap_step/models/`、`gap_step/teachers/` 作为活动入口。

原因：项目保持单一清晰入口。

## 2026-05-12：progress reward 使用连续几何

决定：训练可启用 `dynamic_geometry` progress shaping，但 reward potential 必须基于连续位置、墙体、窗口和可见性。

原因：cell 是拓扑元数据，不能代替连续几何距离。

## 2026-05-12：使用 tanh-squashed Gaussian PPO 动作

决定：PPO log prob 必须对应实际执行的 tanh-squashed action。

原因：不能回到 `Normal -> clamp(action)`，否则 log prob 与环境动作不一致。

## 2026-05-13：采用完整图特权教师

决定：教师观测使用 `GraphObs`，包含完整拓扑、gate 动力学、当前安全状态和全局状态。

原因：局部向量教师无法稳定通过早期动态课程。

## 2026-05-13：使用纯 PyTorch GNN

决定：不引入 PyG/DGL。

原因：依赖更少，图 batch 逻辑保持可控。

## 2026-05-14：改为逐课程训练

决定：训练模式改为 `stagewise`，顺序为：

```text
C1 -> C1_5 -> C2A -> C2B -> C3 -> C4 -> C5
```

每个课程继承前一课程模型参数，但重置优化器；每个课程单独保存最终模型和训练 CSV。

原因：上一次训练卡在 C1，需要先逐课程观察和稳定早期能力。

## 2026-05-14：取消 best checkpoint

决定：不再保存 `teacher_best.pt`，只保存每个课程的 `teacher_final.pt`。

原因：当前实验重点是逐课程最终结果和实时诊断，减少 checkpoint 语义混乱。

## 2026-05-14：训练日志改为中文实时指标

决定：训练时不使用进度条，每次 PPO update 输出中文指标。

原因：更容易直接看到成功率、碰撞率、回报、熵、KL 和 PPO 更新次数。

## 2026-05-14：降低探索噪声

决定：默认设置为：

```text
entropy_coef = 0.0001
log_std_init = -1.0
min_log_std = -2.0
max_log_std = 0.0
```

原因：上一次训练中 std 和 entropy 持续升高，策略后期退化为高噪声动作。

## 2026-05-14：提高通用导航失败代价

决定：训练配置中使用：

```text
reward_progress = 4.0
reward_timeout = -20.0
```

原因：诊断显示原地不动到超时比随机探索撞墙更划算，会把 PPO 推向“少动等死”。这不是 C1 专用改动，后续时变窗口课程也需要避免超时成为低风险局部最优。

## 2026-05-14：碰撞时不保留正 progress reward

决定：如果当前 step 发生撞墙或撞门，并且 progress reward 为正，则置零。

原因：避免策略通过冲墙获得短期进展奖励。

## 2026-05-14：GNN 读出加入 agent/goal cell

决定：actor/critic 输入从全图池化扩展为：

```text
global + mean_pool + max_pool + agent_cell + goal_cell
```

原因：只靠全图池化会稀释“当前位置”和“目标位置”的关键信息。

## 2026-05-14：PPO 改为显式旧策略流程

决定：训练时维护 `model` 和 `model_old`。`model_old` 只用于 rollout 采样，`model` 只用于 PPO 梯度更新，每次更新后同步 `model_old <- model`。

原因：与标准 PPO 流程对齐，避免采样策略和更新策略语义混在一起。

## 2026-05-14：KL 指标改为标准非负近似

决定：PPO early stop 使用：

```text
mean((exp(new_logp - old_logp) - 1) - (new_logp - old_logp))
```

同时记录裁剪率和解释方差。

原因：原来的 `old_logp - new_logp` 均值可能为负，不适合作为主要 KL 停止指标。

## 2026-05-14：C5 tune 加入几何动作先验

决定：在特权教师全局观测中加入动态几何引导特征，并将归一化动作先验作为 actor raw mean 的先验项，PPO 学残差。

新增全局特征包括：

```text
路径首段方向、动态几何 potential、门等待、下一路标距离、前方关闭门距离/等待、动作先验
```

当前 `GLOBAL_FEATURE_DIM = 26`。

原因：纯 PPO 从稀疏/弱 shaping 中学习门前等待和穿门时机效率太低；几何先验能把 C5 起点从接近 0 提高到可训练范围。

## 2026-05-14：评估和可视化复用 checkpoint env 配置

决定：`evaluate.py` 和 `visualize.py` 加载 checkpoint 后复用其中的 `config["env"]`。

原因：C5 tune 配置和 full 配置可能不同。如果评估或可视化使用默认环境，会把训练出的策略放到另一个任务口径里，导致结果不可比。

## 2026-05-14：C5 tune 当前未真实超过 70%

决定：记录当前最好正式评估结果，不把训练 rollout 成功率当作最终达标。

当前结果：

```text
checkpoint: checkpoints/C5/teacher_final.pt
eval: python -m gap_step.evaluate --checkpoint checkpoints/C5/teacher_final.pt --episodes 50 --stages C5
C5 id_test success_rate = 0.68
closed_gate_collision_rate = 0.16
wall_collision_rate = 0.10
timeout_rate = 0.06
```

该结果使用 `train_teacher_c5_tune.yaml` 的放宽口径：`robot_radius=0.1`、`safe_margin=0.0`、`max_steps=800`。原始 full 口径仍未达标。

随后运行 50 回合三 split 泛化评估：

```text
id_test = 0.68
ood_size_test = 0.64
ood_dynamics_test = 0.40
```

OOD dynamics 的主失败类型是 closed gate collision。
