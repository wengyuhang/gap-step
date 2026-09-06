# 时变非凸窗口穿越研究

本仓库研究无人机在非凸、随时间运动或形变的窗口中如何选择穿越位置与时刻，并在几何、整机和动力学约束下缩短飞行时间。项目从 GAP-Step 二维 PPO 迷宫发展到 TOGT 扩展与多方法研究；根据截至 2026-09-06 的代码和 Git 记录，近期开发重点是三维非凸窗口、连续安全认证、旋转同步穿越和安全强化学习。

## 从哪里开始

- [当前项目状态](docs/PROJECT_CONTEXT.md)：各方法进展、证据边界和未完成事项。
- [记忆文档索引](docs/README.md)：现行约定、架构、待办和历史记录的分工。
- [运行与验证](docs/RUNBOOK.md)：环境、入口及各研究线测试。
- [代理约定](AGENTS.md)：修改代码与实验时必须保留的边界；`CLAUDE.md` 与其一致。

## 研究目录

| 目录 | 当前角色 | 入口 |
|---|---|---|
| `nonconvex_timevarying_window/` | 近期主要开发区域：Atlas、SC、MSR、SIP、Planar-RS、RotSync、AVS-PPO、PhaseGuard-RL，以及跨方法比较 | [问题定义](nonconvex_timevarying_window/PROBLEM_DEFINITION.md)、[方法索引](nonconvex_timevarying_window/README.md) |
| `closed_loop_deformable_window/` | 连续局部形变、开放机会、安全区暂时为空和完整初态返回；FAPP-PPO / MDG 并列 | [问题定义](closed_loop_deformable_window/PROBLEM_DEFINITION.md)、[方法索引](closed_loop_deformable_window/README.md) |
| `togt_timevarying_window/` | 较早的独立 DynaTOGT 动态窗口实验，Hermite 轨迹，支持指定任务序列与重复穿越 | [项目说明](togt_timevarying_window/README.md) |
| `gap_step/` | 早期连续二维生成迷宫和 GNN PPO 特权教师实验 | [架构与旧入口区别](docs/ARCHITECTURE.md)、[运行命令](docs/RUNBOOK.md) |
| `复现/` | TOGT 论文及外部复现包，Git 忽略的本地资源 | [历史复现审计](docs/TOGT_REPRODUCTION_AUDIT.md) |

“纯 PPO、不使用规划器”的约定只约束 GAP-Step 的纯教师实验。三维规划、MINCO、连续整机检查、控制器和基线比较已经是其他目录的实际研究内容。

## 当前需要保留的结论

RotSync 和 PhaseGuard-RL 是最近加入的方法：前者已有正式赛道与固定点比较，后者仅完成精简核心，尚无正式训练结论。SC/MSR/RotSync 的采样验收与 SIP/Planar-RS 的名义模型连续域认证必须区分。FAPP-PPO 的后期策略退化、AVS 极难场景的盾牌接管、RotSync 部分赛道超速和固定点基线更快的结果都保留在[状态记录](docs/PROJECT_CONTEXT.md)中。

本地另有未提交的 [RotSync ICRA 五项实验方案](nonconvex_timevarying_window/实验方案/ICRA_EXPERIMENT_PLAN.md)和 [Gazebo 适配层](nonconvex_timevarying_window/comparisons/sc_sip_fast_closed_loop/gazebo/README.md)。它们属于工作区进行中内容，不代表五项实验已执行或 PX4/真机已接通；仅从 Git 检出时可能不存在。

## 最小验证

```bash
source /home/jack/anaconda3/etc/profile.d/conda.sh
conda activate wyh
pytest -q nonconvex_timevarying_window/rot_sync_sc_togt/tests
pytest -q nonconvex_timevarying_window/phaseguard_rl/tests
```

以上按需选择；完整入口见 [RUNBOOK](docs/RUNBOOK.md)。裸 `pytest -q` 只运行 `gap_step/tests`，不能代表全仓通过。生成结果与模型按方法保存在各自目录，并由 Git 忽略。
