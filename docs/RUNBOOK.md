# 运行与验证手册

更新：2026-09-09。命令入口和参数按源码/方法说明核对；默认从仓库根目录运行，MDG 单独注明。

## 环境与输出

```bash
source /home/jack/anaconda3/etc/profile.d/conda.sh
conda activate wyh
```

根 `environment.yml` 来自早期 GAP-Step。SC、SIP、RotSync、FAPP、PhaseGuard 等依赖以各自 README 和 requirements 为准；SIP/Planar-RS/PhaseGuard 的严格检查需要 `python-flint`，缺少时不能当成采样安全通过。

已有结果、checkpoint 和外部复现资料可能被 Git 忽略，检出源码不代表它们存在。新实验使用独立结果目录，保留原运行。GAP-Step 的部分训练配置含 `clean_outputs: true`，会清空所配置输出目录；复用结果前应在新配置中指定新路径或关闭该开关。

## 测试范围

```bash
# 根 pytest.ini 只默认收集该目录
pytest -q gap_step/tests

# 根据修改范围显式选择方法
pytest -q togt_timevarying_window/tests
pytest -q nonconvex_timevarying_window/atlas_dynatogt/tests
pytest -q nonconvex_timevarying_window/sc_dynatogt/tests
pytest -q nonconvex_timevarying_window/msr_dynatogt/tests
pytest -q nonconvex_timevarying_window/sip_dynatogt/tests
pytest -q nonconvex_timevarying_window/planar_rs_dynatogt/tests
pytest -q nonconvex_timevarying_window/rot_sync_sc_togt/tests
pytest -q nonconvex_timevarying_window/interpolated_rot_sync_sc_togt/tests
pytest -q nonconvex_timevarying_window/random_dk_sc_dynatogt/tests
pytest -q nonconvex_timevarying_window/feasibility_guided_cem_sc_dynatogt/tests
pytest -q nonconvex_timevarying_window/avs_ppo/tests
pytest -q nonconvex_timevarying_window/phaseguard_rl/tests
pytest -q nonconvex_timevarying_window/comparisons/sc_sip_fast_closed_loop/tests
pytest -q nonconvex_timevarying_window/comparisons/sc_sip_motion_rate_benchmark/tests
pytest -q closed_loop_deformable_window/fapp_ppo/tests
pytest -q closed_loop_deformable_window/mdg/tests
```

以上是入口清单，不是要求每次全量运行。修改 SC 的 MINCO/动力学或 SIP 认证接口时，还需按 [ARCHITECTURE](ARCHITECTURE.md) 选择受影响调用方。旧废案测试仅在维护相应方法/反例时运行。纯文档改动核对链接、路径、镜像与 `git diff --check` 即可。

## 近期方法

Random-DK 与 Feasibility-Guided CEM 的三窗入口分别是：

```bash
python -m nonconvex_timevarying_window.random_dk_sc_dynatogt.multi_window --outdir nonconvex_timevarying_window/random_dk_sc_dynatogt/results/new_three_u_run
python -m nonconvex_timevarying_window.feasibility_guided_cem_sc_dynatogt.multi_window --outdir nonconvex_timevarying_window/feasibility_guided_cem_sc_dynatogt/results/new_three_u_run
```

后者默认读取方法 README 中列出的冻结 SC 基线和两条单窗安全模板；若迁移或清理本地忽略结果，必须用 `--baseline-json` 与两个 `--template-result` 显式提供来源。结果目录必须不存在，避免覆盖历史运行。

RotSync 的默认 suite 是 `formal`，不是 smoke；快速回归应显式选择：

```bash
python -m nonconvex_timevarying_window.rot_sync_sc_togt.experiments --suite smoke --no-animation --outdir nonconvex_timevarying_window/rot_sync_sc_togt/results/smoke_new_run
python -m nonconvex_timevarying_window.interpolated_rot_sync_sc_togt.experiments --suite oblique_smoke --no-animation --audit-dt 0.001 --collision-samples 5001 --outdir nonconvex_timevarying_window/interpolated_rot_sync_sc_togt/results/oblique_new_run
python -m nonconvex_timevarying_window.interpolated_rot_sync_sc_togt.compare_fixed_wp_counterexample --outdir nonconvex_timevarying_window/interpolated_rot_sync_sc_togt/results/zero_thickness_new_run
python -m nonconvex_timevarying_window.interpolated_rot_sync_sc_togt.compare_fixed_wp_seeded --outdir nonconvex_timevarying_window/interpolated_rot_sync_sc_togt/results/zero_thickness_fixed_wp_seeded_new_run
python -m nonconvex_timevarying_window.interpolated_rot_sync_sc_togt.compare_sc_dynatogt_fixed_wp --outdir nonconvex_timevarying_window/interpolated_rot_sync_sc_togt/results/zero_thickness_sc_dynatogt_vs_fixed_wp_new_run
python -m nonconvex_timevarying_window.rot_sync_sc_togt.experiments --suite formal --outdir nonconvex_timevarying_window/rot_sync_sc_togt/results/formal_new_run
python -m nonconvex_timevarying_window.rot_sync_sc_togt.single_window_comparison
```

最后一个入口默认生成新时间戳目录。正式判据见 [FORMAL_EXPERIMENTS](../nonconvex_timevarying_window/rot_sync_sc_togt/FORMAL_EXPERIMENTS.md)。

PhaseGuard-RL 当前使用 `train.py` 中的 Python `train(environment, model, ...)` 函数，需要先构造 `PlanningEnvironment` 与 `PhaseActorCritic`；没有可直接套用的完整场景训练 CLI。先读[核心说明](../nonconvex_timevarying_window/phaseguard_rl/README.md)和[最小实验协议](../nonconvex_timevarying_window/phaseguard_rl/EXPERIMENT_PROTOCOL.md)，不要把 `python -m ...train` 写成已可执行的正式训练流程。

SIP 生成与重放示例（第二条依赖第一条生成的具体 run）：

```bash
python -m nonconvex_timevarying_window.sip_dynatogt.experiments --suite smoke --outdir nonconvex_timevarying_window/sip_dynatogt/results/smoke_new_run
python -m nonconvex_timevarying_window.sip_dynatogt.verify --run nonconvex_timevarying_window/sip_dynatogt/results/smoke_new_run/static_1gate
```

Planar-RS 用位置参数传入 run，和 SIP 的 `--run` 不同：

```bash
python -m nonconvex_timevarying_window.planar_rs_dynatogt.experiments --case ordinary --baseline-certificate --outdir nonconvex_timevarying_window/planar_rs_dynatogt/results/ordinary_new_run
python -m nonconvex_timevarying_window.planar_rs_dynatogt.verify nonconvex_timevarying_window/planar_rs_dynatogt/results/ordinary_new_run
```

AVS-PPO 的普通训练/ID-OOD 与极难长方体实验分别按[方法 README](../nonconvex_timevarying_window/avs_ppo/README.md)和[极难报告](../nonconvex_timevarying_window/avs_ppo/HARDEST_COMPARISON_REPORT.md)运行，不混用模型与 checkpoint。

## 其他研究入口

```bash
python -m nonconvex_timevarying_window.atlas_dynatogt.experiments --suite smoke --outdir nonconvex_timevarying_window/atlas_dynatogt/results/smoke_new_run
python -m nonconvex_timevarying_window.sc_dynatogt.experiments --suite smoke
python -m nonconvex_timevarying_window.msr_dynatogt.experiments --suite smoke
python -m closed_loop_deformable_window.fapp_ppo.experiments --suite smoke
```

MDG 使用自己的 scripts/config 路径：

```bash
cd closed_loop_deformable_window/mdg
python scripts/run_benchmark.py --config configs/smoke.yaml --suite smoke
```

执行完需返回仓库根目录再使用其他命令。跨方法比较与本地 Gazebo 的命令分别在[宽域比较](../nonconvex_timevarying_window/comparisons/sc_sip_fast_closed_loop/README.md)、[速率基准](../nonconvex_timevarying_window/comparisons/sc_sip_motion_rate_benchmark/README.md)、[Gazebo README](../nonconvex_timevarying_window/comparisons/sc_sip_fast_closed_loop/gazebo/README.md)中；Gazebo 为本地未跟踪内容，不能假定所有检出都存在。

## 保留的 GAP-Step / DynaTOGT

较新的生成窗口迷宫使用以下入口，`configs/...` 与 `checkpoints/...` 会由 `gap_step/utils.py` 解析到 **`gap_step/` 下**：

```bash
python -m gap_step.train_window --config configs/train_window_smoke.yaml
python -m gap_step.train_window --config configs/train_window_teacher.yaml
python -m gap_step.evaluate_window --checkpoint checkpoints/window_generated/C5/teacher_final.pt --episodes 200 --stage C5
python -m gap_step.visualize_window --checkpoint checkpoints/window_generated/C5/teacher_final.pt
```

例如该 checkpoint 的实际位置是 `gap_step/checkpoints/window_generated/C5/teacher_final.pt`。旧 `gap_step.train --config configs/train_teacher_smoke.yaml` 仍是旧环境入口，不能与生成窗口迷宫的 checkpoint/结果混用。纯教师实验确认 planner auxiliary 默认关闭；课程表与超参数读本次选用 YAML，不从旧根 README 抄固定值。

```bash
python -m togt_timevarying_window.demo --scenario canonical --mode ordered_dynamic
python -m togt_timevarying_window.export_demo --scenario canonical --mode ordered_dynamic
python -m togt_timevarying_window.export_demo --scenario canonical --mode ordered_dynamic --order G1,G6,G1,G3,G2,G5,G4,G2 --outdir togt_timevarying_window/results/repeated_new_run
python -m togt_timevarying_window.experiments --suite smoke --outdir togt_timevarying_window/results/smoke_new_run
```

外部复现先确认本地 `复现/TOGT-Planner-reproduction/` 存在，再按其说明运行；历史构建证据见 [TOGT_REPRODUCTION_AUDIT](TOGT_REPRODUCTION_AUDIT.md)。
