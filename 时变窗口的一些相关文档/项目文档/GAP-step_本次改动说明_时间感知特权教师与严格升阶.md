# GAP-Step 本次改动说明：GNN 完整特权教师

## 背景

旧的局部特权教师能在 C1 学到一些行为，但在 C2 动态窗口任务上 deterministic policy 不能稳定成功。主要原因是 teacher 仍然缺少完整拓扑和完整 gate 动力学，无法可靠判断哪些 gate 在通往目标的结构上、什么时候该等待、什么时候该通过。

## 本次主要改动

1. 废弃局部向量 teacher 观测主路径。
2. 新增 `GraphObs`：
   - `global_features`
   - `node_features`
   - `node_type`
   - `edge_index`
   - `edge_features`
3. 新增 cell node 和 gate node。
4. 新增 cell-cell topology edge、gate-cell edge 和 self-loop。
5. teacher 模型改为纯 PyTorch GNN actor-critic。
6. PPO rollout 和 minibatch update 改为 graph collation。
7. 增加 target-KL early stop。
8. 新增 `teacher_best.pt`，由 deterministic promotion eval 更新。
9. 课程拆成 `C1, C1_5, C2A, C2B, C3, C4, C5`。

## 不变内容

- 环境运动仍是连续二维动力学。
- 碰撞仍使用连续几何。
- 成功条件不变。
- PPO 动作仍为 tanh-squashed Gaussian 连续加速度。
- 不引入专家演示、BC、视觉学生、A*/MPC 或 waypoint 执行。

## 配置变化

关键配置：

```yaml
model_type: gnn
gnn_hidden_dim: 128
gnn_layers: 4
entropy_coef: 0.02
min_log_std: -0.5
target_kl: 0.03
checkpoint_path: checkpoints/teacher_final.pt
best_checkpoint_path: checkpoints/teacher_best.pt
```

## 已完成验证

```bash
pytest -q
python -m gap_step.train --config gap_step/configs/train_teacher_smoke.yaml
python -m gap_step.evaluate --checkpoint checkpoints/teacher_best.pt --episodes 2 --stages C1,C1_5,C2A,C2B,C3,C4,C5 --output /tmp/gap_gnn_eval_smoke.csv
```

验证结论：

- 测试通过；
- smoke 训练完成；
- best/final checkpoint 写出；
- stage-wise 评估链路完成。

smoke 不代表策略质量，下一步需要正式训练到 C2B 并观察 deterministic success。
