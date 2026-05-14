# AGENTS.md

GAP-Step 项目给代码代理使用的本地约定。

## 目标

当前阶段只训练 PPO 特权教师。教师使用纯 PyTorch GNN，观测完整拓扑图、窗口动力学和全局状态，输出连续二维加速度。

不做视觉学生、BC、专家演示、A*/MPC、waypoint 执行、SITT、世界模型、主动感知、3D 仿真或四旋翼动力学。

## 代码布局

可运行代码只放在 `gap_step/`：

- `env.py`：连续迷宫、碰撞、图观测、奖励、渲染
- `graph.py`：`GraphObs`、图 batch、特征维度
- `curriculum.py`：C1-C5 课程和 C1_5/C2A/C2B 桥接课程
- `model.py`：GNN tanh-squashed Gaussian PPO 教师
- `ppo.py`：graph rollout、GAE、PPO update、旧策略同步
- `train.py`：逐课程训练入口
- `evaluate.py`：ID/OOD 和分课程评估
- `visualize.py`：GIF 可视化

不要重新启用 `trainers/`、`scripts/`、`gap_step/envs/`、`gap_step/models/`、`gap_step/teachers/`。

## 训练规则

- 当前主线是逐课程训练：`C1 -> C1_5 -> C2A -> C2B -> C3 -> C4 -> C5`
- 后一课程继承前一课程的模型参数，但重置优化器
- 每个课程只保存最终模型，不保存 `teacher_best.pt`
- 每个课程单独输出：
  - `checkpoints/<课程>/teacher_final.pt`
  - `results/<课程>/train_metrics.csv`
- 训练时不使用进度条，输出中文实时指标
- PPO 训练使用显式 `model_old` 采样，`model` 更新，更新后同步 `model_old <- model`

## 技术规则

- 重要改动前先读相关 `docs/`
- 教师观测固定为 `GraphObs(global_features, node_features, node_type, edge_index, edge_features)`
- 图包含 cell node、gate node、cell-cell 边、gate-cell 边和 self-loop
- PPO 动作用 tanh-squashed Gaussian，并计算实际执行动作的 log prob
- PPO 的 KL early stop 使用标准非负近似，不使用可能为负的 `old_logp - new_logp` 均值作为主 KL
- progress reward 必须基于连续几何，不使用 cell 作为 reward potential
- 撞墙或撞门时，不允许保留正的 progress reward
- 不要改碰撞规则、成功条件或迷宫生成语义，除非用户明确要求
- 保留 `data/`、`checkpoints/`、`logs/`、`runs/`、`results/` 的忽略规则

## 命令

```bash
source /home/jack/anaconda3/etc/profile.d/conda.sh && conda activate wyh
pytest -q
python -m gap_step.train --config gap_step/configs/train_teacher_smoke.yaml
python -m gap_step.train --config gap_step/configs/train_teacher_full.yaml
python -m gap_step.evaluate --checkpoint checkpoints/C5/teacher_final.pt
python -m gap_step.evaluate --checkpoint checkpoints/C5/teacher_final.pt --episodes 20 --stages C1,C1_5,C2A,C2B,C3,C4,C5
```
