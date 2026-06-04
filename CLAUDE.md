# AGENTS.md

GAP-Step / DynaTOGT 仓库给代码代理使用的本地约定。详细背景放在 `docs/` 和各子项目 README 中；本文件只保留必须遵守的规则。

## 项目边界

- `gap_step/` 是主线：连续二维时变窗口迷宫 + 纯 PPO 特权教师。
- `togt_timevarying_window/` 是独立 DynaTOGT 子项目：基于 TOGT 论文思想的 3D 动态时变窗口穿越实验。
- `复现/` 是 TOGT 论文复现包；不要把其中代码并入主线。
- 重要改动前先读相关 `docs/`；TOGT/DynaTOGT 细节见 `docs/ARCHITECTURE.md`、`docs/PROJECT_CONTEXT.md` 和 `togt_timevarying_window/README.md`。

## 主线 PPO 规则

- 当前主线只训练 PPO 特权教师：纯 PyTorch GNN，完整拓扑图/窗口动力学/全局状态输入，连续二维加速度输出。
- 不做视觉学生、BC、专家演示、A*/MPC、waypoint 执行、SITT、世界模型、主动感知、3D 仿真或四旋翼动力学，除非用户明确要求新方向。
- 教师观测固定为 `GraphObs(global_features, node_features, node_type, edge_index, edge_features)`。
- PPO 使用显式 `model_old` 采样、`model` 更新，更新后同步 `model_old <- model`。
- KL early stop 使用标准非负近似；不要用可能为负的 `old_logp - new_logp` 均值作为主 KL。
- progress reward 必须基于连续几何；撞墙或撞门时不允许保留正 progress reward。
- 不要改碰撞规则、成功条件或迷宫生成语义，除非用户明确要求。

## 训练/评估命令

```bash
source /home/jack/anaconda3/etc/profile.d/conda.sh && conda activate wyh
pytest -q
python -m gap_step.train --config configs/train_teacher_smoke.yaml
python -m gap_step.train --config configs/train_teacher_full.yaml
python -m gap_step.evaluate --checkpoint checkpoints/C5/teacher_final.pt
python -m gap_step.evaluate --checkpoint checkpoints/C5/teacher_final.pt --episodes 20 --stages C1,C1_5,C2A,C2B,C3,C4,C5
```

## DynaTOGT 常用命令

```bash
python -m togt_timevarying_window.demo --scenario canonical --mode ordered_dynamic
python -m togt_timevarying_window.export_demo --scenario canonical --mode ordered_dynamic
python -m togt_timevarying_window.export_demo --scenario canonical --mode ordered_dynamic --order G1,G6,G1,G3,G2,G5,G4,G2 --outdir togt_timevarying_window/results/repeated_demo
python -m togt_timevarying_window.experiments --suite smoke --outdir togt_timevarying_window/results
pytest -q togt_timevarying_window/tests
```

## 文件与镜像规则

- 保留 `data/`、`checkpoints/`、`logs/`、`runs/`、`results/`、`复现/` 的忽略规则。
- `AGENTS.md` 是代理规范源；`CLAUDE.md` 必须与其内容完全一致。
- 修改 `AGENTS.md` 后必须同步复制到 `CLAUDE.md`。
