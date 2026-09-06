# TOGT 复现历史审计

整理：2026-09-06。此页保留 2026-06 复现任务的已有证据，并标出此后入口变化。以下“当时完成”来自[原始审计快照](history/2026-09-06-before-memory-refactor/docs__TOGT_REPRODUCTION_AUDIT.md)与[历史任务日志](history/2026-09-06-before-memory-refactor/docs__TASK_LOG.md)，本轮没有重新构建 C++ 或重跑复现实验。

## 原任务与已有记录

原任务要求在 `复现/` 复现 TOGT-Planner，并在独立顶层项目中扩展论文式动态窗口，避免套用旧二维迷宫环境。

| 交付物 | 当时记录的证据 | 当前解释 |
|---|---|---|
| 本地源码/论文复现包 | `复现/TOGT-Planner-reproduction/source/`、`REPRODUCTION.md`、`BUILD_DEPS.md` | 本地资源，`复现/` 被 Git 忽略 |
| 依赖与构建 | vendored Eigen 3.4.0；CMake/build 成功；ctest 3 项通过 | 历史构建验收，不能当作当前机器刚复验 |
| 结果级复现 | `analyze_trajectory.py`：lap_time=8.21 s、path_length=83.189 m、max_speed=19.321 m/s；有绘图记录 | 原复现轨迹的指标，不是后续动态窗口算法的成绩 |
| 独立动态窗口扩展 | `togt_timevarying_window/` | 目录仍在，内部已从早期离散 prototype 重构为 DynaTOGT |
| 早期 3D 原型 | 12-gate `DynamicGate/RaceTrack`、`planner.py`、`outputs/`、3 项 Python 测试 | 都属于旧阶段表述，不能用作现行 CLI/模块清单 |
| 2026-06-04 DynaTOGT 重构 | 动态 `G_i(t)`、热启动与连续优化、Hermite、任意指定序列/重复穿越、CSV/PNG/GIF；当时记录 6 项测试通过 | 现行说明见该项目 README，测试数量以实际运行报告为准 |

## 当前入口

现行 DynaTOGT 使用 `environment.py` 的动态窗口/轨道、`optimizer.py`、`trajectory.py` 和 `experiments.py`；产物使用 `results/`，不要继续引用旧 `outputs/` 为默认目录。

```bash
python -m togt_timevarying_window.demo --scenario canonical --mode ordered_dynamic
python -m togt_timevarying_window.export_demo --scenario canonical --mode ordered_dynamic
pytest -q togt_timevarying_window/tests
```

源码、算法与实验说明见 [DynaTOGT README](../togt_timevarying_window/README.md)。后续非凸 SC/MINCO、SIP 整机认证、RotSync 与强化学习的状态由 [PROJECT_CONTEXT](PROJECT_CONTEXT.md)维护，不反向计入 2026-06 论文复现完成度。
