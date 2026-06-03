# GAP-Step

GAP-Step 是一个连续二维时变窗口迷宫项目。当前目标是训练 PPO 特权教师：教师看到完整拓扑图和窗口动力学，直接输出二维连续加速度动作。

当前不做视觉学生、行为克隆、专家演示、A*/MPC、世界模型、主动感知、三维仿真或四旋翼动力学。

## 代码结构

所有可运行代码都在 `gap_step/`：

- `env.py`：连续迷宫、碰撞、奖励、图观测、渲染
- `graph.py`：`GraphObs`、图 batch、特征维度
- `curriculum.py`：`C1,C1_5,C2A,C2B,C3,C4,C5` 课程迷宫
- `model.py`：纯 PyTorch GNN PPO 教师
- `ppo.py`：旧策略采样、GAE、PPO 更新、策略同步
- `train.py`：逐课程训练入口
- `evaluate.py`：ID/OOD 和分课程评估
- `visualize.py`：rollout GIF 可视化
- `configs/`：训练配置
- `tests/`：测试

不要把旧的 `trainers/`、`scripts/`、`gap_step/envs/`、`gap_step/models/`、`gap_step/teachers/` 重新作为入口。

## 环境

```bash
source /home/jack/anaconda3/etc/profile.d/conda.sh
conda activate wyh
```

## 训练

烟测：

```bash
pytest -q
python -m gap_step.train --config gap_step/configs/train_teacher_smoke.yaml
```

完整训练：

```bash
python -m gap_step.train --config gap_step/configs/train_teacher_full.yaml
```

训练现在使用逐课程模式：

```text
C1 -> C1_5 -> C2A -> C2B -> C3 -> C4 -> C5
```

后一个课程继承前一个课程的模型参数，但每个课程重新建立优化器。训练时不显示进度条，而是每次 PPO 更新后输出中文实时指标。

PPO 使用显式旧策略流程：`model_old` 只负责 rollout 采样，`model` 只负责梯度更新。每次 PPO 更新完成后同步 `model_old <- model`。回报估计仍使用 GAE，动作仍是 tanh-squashed Gaussian。

## 输出

每个课程单独保存最终模型和训练表：

```text
checkpoints/C1/teacher_final.pt
results/C1/train_metrics.csv
checkpoints/C1_5/teacher_final.pt
results/C1_5/train_metrics.csv
...
checkpoints/C5/teacher_final.pt
results/C5/train_metrics.csv
```

不再保存或依赖 `teacher_best.pt`。

评估最终 C5 模型：

```bash
python -m gap_step.evaluate --checkpoint checkpoints/C5/teacher_final.pt
```

分课程评估某个模型：

```bash
python -m gap_step.evaluate --checkpoint checkpoints/C1/teacher_final.pt --episodes 20 --stages C1
python -m gap_step.evaluate --checkpoint checkpoints/C5/teacher_final.pt --episodes 20 --stages C1,C1_5,C2A,C2B,C3,C4,C5
```

可视化：

```bash
python -m gap_step.visualize --checkpoint checkpoints/C5/teacher_final.pt
```

## 当前训练改动

- 降低初期探索噪声：`log_std_init=-1.0`，`entropy_coef=0.0001`，`max_log_std=0.0`
- PPO 调得更保守：`learning_rate=0.0001`，`target_kl=0.2`，`rollout_steps=4096`
- 通用导航奖励更重视失败代价：`reward_progress=4.0`，`reward_timeout=-20.0`
- PPO 采样改为显式旧策略，KL 改为标准非负近似，并记录裁剪率和解释方差
- 撞墙或撞门时，不允许获得正的 `progress_reward`
- GNN 读出除了全图池化，还显式加入 agent 所在 cell 和 goal cell 的表示
- 先看 C1 是否稳定，再逐级推进后续课程

## 观测约定

教师观测是图：

```text
GraphObs(
  global_features: [16],
  node_features: [num_nodes, 32],
  node_type: [num_nodes],
  edge_index: [2, num_edges],
  edge_features: [num_edges, 20],
)
```

图中有 cell node、gate node、cell-cell 拓扑边、gate-cell 边和 self-loop。环境动力学、碰撞和成功判定仍然是连续几何。

生成目录 `data/`、`checkpoints/`、`logs/`、`runs/`、`results/` 被 Git 忽略。


## TOGT 论文复现与动态门改进

本仓库还包含一个独立的新项目，用于复现和扩展论文 `复现/论文/2309.06837v3.pdf` / TOGT-Planner。该部分不属于 `gap_step/` 旧迷宫训练主线。

```text
复现/TOGT-Planner-reproduction/   TOGT-Planner 源码级复现、结果级复现脚本和记录
togt_timevarying_window/          独立论文式动态 gate/window 改进项目
docs/TOGT_REPRODUCTION_AUDIT.md   需求到证据的完成度审计
```

常用命令：

```bash
python 复现/TOGT-Planner-reproduction/check_reproduction.py
python 复现/TOGT-Planner-reproduction/analyze_trajectory.py
python -m togt_timevarying_window.demo
python -m togt_timevarying_window.export_demo
pytest -q togt_timevarying_window/tests
```

TOGT C++ 原生构建已通过本地 vendored Eigen 完成；源码树、CMake/build/ctest、结果级复现均已验证。动态门改进项目已升级为复杂 12-gate 3D 任务，动态任务约 31.4s，并通过测试、导出 CSV/PNG/GIF。
