# CLAUDE.md

本文件指导 Claude Code (claude.ai/code) 在此仓库中协作。

## 项目概要

GAP-Step 训练一个 PPO 特权教师（GNN Actor-Critic），用于连续二维时变窗口迷宫。教师观测完整迷宫拓扑、窗口动力学和特权状态，输出连续二维加速度动作。

**不包含**：视觉学生、行为克隆、专家演示、A*/MPC、世界模型、主动感知、三维仿真或四旋翼动力学。

## 代码布局

所有可运行代码在 `gap_step/` 下：

### 主线训练（标准 passage 环境）

- `env.py` — `ContinuousMazeEnv`，Gymnasium 环境，含连续碰撞判定、图观测、基于动态几何路网的奖励塑形、RGB 渲染。
- `curriculum.py` — 课程定义 (C1→C1_5→C2A→C2B→C3→C4→C5)、DFS 过程式迷宫生成器、带正弦开度/旋转的门动力学。
- `graph.py` — `GraphObs` / `GraphBatch` 数据类、`collate_graph_obs` 批处理。
- `model.py` — `GNNTeacherActorCritic`（别名 `TeacherActorCritic`）：多层 GNN（消息传递 + mean/max/flagged 池化）、tanh-squashed Gaussian 策略。
- `ppo.py` — Rollout 采样、GAE 计算、PPO 更新、`sync_policy_old`。
- `train.py` — 训练入口，支持三种课程模式：`stagewise`（每课程固定步数）、`adaptive`（按滚动成功率自动升阶）、`fixed`（按时间切换课程）。
- `evaluate.py` — ID/OOD 和分课程评估（确定性 rollout）。
- `visualize.py` — GIF 可视化。
- `configs/` — YAML 训练配置。

### Window maze 分支（曲线窗型变体）

- `window_maze_env.py` — `TimeVaryingWindowMazeEnv`，带折线/曲线窗的迷宫变体。
- `train_window.py` — 该变体的训练入口。
- `train_window_bc.py` — 行为克隆基线。
- `train_window_dagger.py` — DAgger 训练。
- `window_planner.py` — 参考动作规划器。
- `evaluate_window.py` — 该变体的评估。
- `visualize_window.py` — 该变体的可视化。

### 共享工具

- `utils.py` — 路径解析、YAML 加载、几何碰撞辅助函数。
- `config.py` — `utils.py` 的重新导出。
- `gif.py` — GIF 生成。
- `tests/` — pytest 测试。

## 常用命令

```bash
# 激活环境
source /home/jack/anaconda3/etc/profile.d/conda.sh && conda activate wyh

# 运行测试
pytest -q
pytest gap_step/tests/test_env.py -v
pytest gap_step/tests/test_window_maze_env.py -v

# 烟测 / 快速训练
python -m gap_step.train --config gap_step/configs/train_teacher_smoke.yaml

# 完整训练（主线）
python -m gap_step.train --config gap_step/configs/train_teacher_full.yaml

# 评估
python -m gap_step.evaluate --checkpoint checkpoints/C5/teacher_final.pt
python -m gap_step.evaluate --checkpoint checkpoints/C5/teacher_final.pt --episodes 20 --stages C1,C1_5,C2A,C2B,C3,C4,C5

# 可视化
python -m gap_step.visualize --checkpoint checkpoints/C5/teacher_final.pt
```

## 训练约定

- **课程顺序**：`C1 → C1_5 → C2A → C2B → C3 → C4 → C5`。后一课程继承前一课程的模型参数，但重置优化器。
- 每课程独立保存：`checkpoints/<课程>/teacher_final.pt`、`results/<课程>/train_metrics.csv`。
- 不保存 `teacher_best.pt`。
- PPO 使用显式 `model_old` 负责 rollout 采样，`model` 负责梯度更新，每次更新后调用 `sync_policy_old()` 同步。
- 训练日志输出中文实时指标，不使用进度条。
- KL early stop 使用标准非负近似 `(exp(log_ratio) - 1) - log_ratio`，不使用可能为负的 `old_logp - new_logp` 均值。

## 关键架构细节

- **观测**：`GraphObs(global_features: [26], node_features: [num_nodes, 32], node_type: [num_nodes], edge_index: [2, num_edges], edge_features: [num_edges, 20])`。图包含 cell 节点、gate 节点、cell-cell 边、gate-cell 边和 self-loop。
- **动作**：连续二维加速度，经 tanh 缩放到 [-max_acc, max_acc]。
- **奖励函数**：`-|reward_time| -|reward_action|*||action||^2 + progress_reward + guidance_reward + reward_goal(成功) + reward_collision(碰撞) + reward_timeout(超时)`。
- **进度奖励**：基于动态几何路网的类 Dijkstra 规划，考虑门的等待时间。碰撞时抑制正进度奖励。
- **GNN 结构**：`GNNTeacherActorCritic` 包含 global_encoder、node_encoder、edge_encoder、4 层 GNN、读出层拼接 global_h、mean_pool、max_pool、agent_node_pool、goal_node_pool。
- **图构建**：环境中构建 cell 节点和 gate 节点，通过 open/wall/gate 类型边连接。edge_index 为双向。

## Git 规则

- `data/`、`checkpoints/`、`logs/`、`runs/`、`results/`、`preview/` 已 gitignore，不要提交。
- 检查点和结果只写入这些目录。

## Checkpoint 格式

torch 字典，键包含：`model_state`、`model_type`、`global_dim`、`node_dim`、`edge_dim`、`hidden_dim`、`gnn_layers`、`max_acc`、`min_log_std`、`max_log_std`、`log_std_init`、`config`、`stages`。
