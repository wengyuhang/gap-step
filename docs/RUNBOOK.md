# 运行与验证手册

## 2026-10-13：当前单窗单重射击MPC

单窗配对入口single_gate_swing.flight默认已切换新MPC；旧paired_final_20261012结果保留。
同参考重飞：python -m convex_timevarying_window.togt_style_mpc.frozen_single_gate --baseline-dir <原单窗结果目录> --outdir <新目录>。
使用togt_style_mpc/README中的acados运行环境变量，当前新MPC自身求解为原生CasADi+L-BFGS-B。
七窗可显式--tracker single-shooting；尚未用此新跟踪实飞七窗，不以单窗结果代替。
测试：python -m pytest -q convex_timevarying_window/togt_style_mpc/tests convex_timevarying_window/comparisons/single_gate_swing/tests。
报告与五条验证：convex_timevarying_window/togt_style_mpc/results/tracking_repair_20261013/REPORT.md。

## 点式 TOGT MINCO 单窗重测（2026-10-09）

独立方法说明、命令及原 W1 2.5Hz/3.5m 已运行结果见
[`togt_point_only_mpc/README.md`](../convex_timevarying_window/togt_point_only_mpc/README.md)
和其 `results/f2p5_a3p5_point_only_v1_20261009/REPORT.md`。运行会控制
`convex_seven_togt_experiment` 容器，不触碰旧回放容器；新结果目录必须不存在。
同结构自由／中心配对结果另见
`convex_timevarying_window/togt_point_only_mpc/results/f2p5_a3p5_point_only_pair_20261009.md`。

## 自由纯跟踪版继续升档（2026-10-08）

`python -u -m convex_timevarying_window.simple_two_gate_soft_only_mpc.raise_difficulty_run --baseline convex_timevarying_window/simple_two_gate_soft_only_mpc/results/pure_tracking_v1_20261008 --outdir <new-directory>`。
固定算法、原初相，频率/幅度/同步0.5步长四轮，每路线首次物理窗框接触停止；未碰撞超时不算边界。
看status.json、boundaries.json与REPORT.md；本批测试上限不等于物理临界值。

## 纯跟踪MPC匹配中心对比（2026-10-08）

`python -u -m convex_timevarying_window.simple_two_gate_soft_only_mpc.center_run --free-results convex_timevarying_window/simple_two_gate_soft_only_mpc/results/pure_tracking_v1_20261008 --outdir <new-directory>`。
仅跑已有七条件的中心版、每组一次；冻结源须匹配自由版，配置只允许free_points不同，世界文件须相同。
新目录REPORT.md按幅度/频率排序，status.json显示进行中/结束；失败及证据缺口保留。

## 无名义准入两窗＋弹性MPC（2026-10-08）

误差恢复自由单方复测：`python -u -m convex_timevarying_window.simple_two_gate_soft_only_mpc.recovery_run --outdir <new-directory>`。
固定v3七条件各一次、只ours；源码快照/summary/status保存于新目录，勿覆盖旧v3配对实验。

完整说明：`convex_timevarying_window/simple_two_gate_soft_only_mpc/README.md`。
conda `wyh`：`python -m pytest -q convex_timevarying_window/simple_two_gate_soft_only_mpc/tests`。
串行新结果目录：`python -u -m convex_timevarying_window.simple_two_gate_soft_only_mpc.run --outdir convex_timevarying_window/simple_two_gate_soft_only_mpc/results/<new-name>`。
基线1Hz/2m重跑，随后频率/幅度/同步0.5递增；中心→自由，每条件一次；仅experiment容器。
主指标完整无碰撞时间与碰撞/未完成；原严格审计保留诊断。`status.json`/`REPORT.md`看当前结果；
被中途停止的旧批次另读`COORDINATOR_STATE.json`，不能把原running状态当仍在执行。
旧硬MPC精确重放与新控制快照验证分别为`mpc_replay.py`、`mpc_recovery_probe.py`，不是实飞。

## 简化两窗：同结构中心完整闭环配对（2026-10-08）

主指标包含返航，不能使用W7退出时间。冻结两次串行飞行；只操作experiment、不操作replay。
协议见simple_two_gate_mpc/README.md。输出目录必须不存在；运行中不编辑方法/依赖源码。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/simple_two_gate_mpc/tests convex_timevarying_window/event_triggered_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.simple_two_gate_mpc.run --outdir convex_timevarying_window/simple_two_gate_mpc/results/new_pair
```

## 第三窗零VA＋竞速初始化（2026-10-08）

最难原赛道1Hz/2m/初相，一版本一次，与冻结历史中心24.428s比较。编译发生在飞行前，
不是离线预求航路；不要更改正在运行批次的源码。仅操作experiment容器，不操作replay。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/third_gate_racing_mpc/tests convex_timevarying_window/third_gate_stop_mpc/tests convex_timevarying_window/event_triggered_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.third_gate_racing_mpc.run --outdir convex_timevarying_window/third_gate_racing_mpc/results/new_run
```

## 两自由窗＋第三窗中心零VA（2026-10-08）

仅最难1Hz/2m/初相，新版本一次；与历史中心24.428s比较，不额外跑中心或上轮估值版本。
协议见`convex_timevarying_window/third_gate_stop_mpc/README.md`；独占experiment，不操作replay。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/third_gate_stop_mpc/tests convex_timevarying_window/event_triggered_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.third_gate_stop_mpc.run --outdir convex_timevarying_window/third_gate_stop_mpc/results/new_run
```

## 两窗＋第三窗时间估计（2026-10-08）

新方案位于`convex_timevarying_window/third_gate_time_mpc/`，与完整赛程续接版本分开。
仅原布局1Hz/2m/初相，各变体一次；独占experiment容器，不操作replay。
构图/编译不求轨迹，耗时与在线正常收敛求解分开记录。输出目录必须不存在。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/third_gate_time_mpc/tests convex_timevarying_window/event_triggered_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.third_gate_time_mpc.run --outdir convex_timevarying_window/third_gate_time_mpc/results/new_hardest_run
```

正常收敛不等于可行，残差合格不等于实时，七窗通过不等于严格返航通过；各项分别报告。

## 自由末态事件版（2026-10-07）

`event_free_terminal_mpc`才是当前两窗/M2/加密/时限/事件触发且非最终局部末态PVAJ自由的入口。
与旧`event_triggered_mpc`（固定停车末端）和`wang_free_terminal_mpc`（另一N2/M4分支）区分。
使用全新结果目录，独占experiment容器；旧结果不可覆盖，replay容器不动。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.event_free_terminal_mpc.run --outdir convex_timevarying_window/event_free_terminal_mpc/results/new_run
```

## 事件触发单场景 MINCO/MPC（2026-10-07）

协议：`convex_timevarying_window/event_triggered_mpc/README.md`。1Hz/2m/初始相位，单次PX4/Gazebo，
只替换experiment专用容器，不操作replay，不并发飞行。旧f1_a2_v1_20261007不可覆盖。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.event_triggered_mpc.run --outdir convex_timevarying_window/event_triggered_mpc/results/new_run
```

## 当前算法与Wang初始相位27次比较（2026-10-07）

协议在`convex_timevarying_window/comparisons/comprehensive_wang/PROTOCOL.md`。
每条件只飞一次、全局初相位0并保留各窗相对相位；18主比较＋9消融。
必须先验证readiness，正式飞行无GUI，不能同时运行其他飞行。
只操作专用experiment容器；未完成既有目录不能用新路径偷偷覆盖失败。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.comparisons.comprehensive_wang.run --readiness 复现/polynomial_dynamic_racing_2023/px4_matched_x500/results/readiness_v3_20261007.json --outdir convex_timevarying_window/comparisons/comprehensive_wang/results/new_run
```

结束自动生成图/报告。`--resume`仅用于完整记录边界的中断且源/配置不变；存在未裁决目录时拒绝覆盖。

## 穿窗邻域加密修复（2026-10-07）

保留旧no-postcheck/软目标入口，使用新目录；重复试验首个失败即停止该版本，
协议在`convex_timevarying_window/comparisons/original_fixed_plane_crossing_resolved/PROTOCOL.md`。
新几何测试会安装进程内场景查询，建议和旧测试分开进程运行。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/crossing_resolved_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.comparisons.original_fixed_plane_crossing_resolved.run --repeats 3 --gui-last --outdir convex_timevarying_window/comparisons/original_fixed_plane_crossing_resolved/results/new_run
```

只替换专用experiment容器，不动replay；--gui-last需DISPLAY可用，原v1_20261007不得覆盖。

## 更保守软目标配对消融（2026-10-07）

仅在专用Gazebo实验容器执行；不会改replay容器。协议在
`convex_timevarying_window/comparisons/original_fixed_plane_deadline_soft_conservative/PROTOCOL.md`。
使用全新输出目录，不覆盖已完成的soft_v1_20261007：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/comparisons/original_fixed_plane_deadline_soft_conservative/tests convex_timevarying_window/comparisons/original_fixed_plane_deadline_no_postcheck/tests convex_timevarying_window/wang_progress_mpc/tests convex_timevarying_window/comparisons/original_fixed_plane_deadline/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.comparisons.original_fixed_plane_deadline_soft_conservative.run --outdir convex_timevarying_window/comparisons/original_fixed_plane_deadline_soft_conservative/results/new_run
```

更新：2026-09-10。命令入口和参数按源码/方法说明核对；默认从仓库根目录运行，MDG 单独注明。

## Wang 2023 独立论文核心（非Gazebo入口）

原文方程与所有未公开参数的选择见[paper_core说明](../复现/polynomial_dynamic_racing_2023/paper_core/README.md)。
单独安装的是公开LBFGS-Lite优化器，不是作者竞速完整源码；旧匹配版本/成绩保留。

```bash
conda run -n wyh python -m 复现.polynomial_dynamic_racing_2023.paper_core.setup_vendor
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q 复现/polynomial_dynamic_racing_2023/paper_core/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 conda run --no-capture-output -n wyh python -m 复现.polynomial_dynamic_racing_2023.paper_core.benchmark --outdir 复现/polynomial_dynamic_racing_2023/paper_core/results/new_core --repeats 5
```

是短局部数值核验，不产生七窗圈时；输出目录需全新，保留所有超预算、未收敛和约束残差。
不得用本轮实时通过数量代替安全飞行成功率。

## 新论文核心：原位置固定平面PX4/Gazebo测试

[协议](../复现/polynomial_dynamic_racing_2023/px4_fixed_plane/PROTOCOL.md)与
[六条件结果](../复现/polynomial_dynamic_racing_2023/px4_fixed_plane/results/original_layout_v2_20261007/REPORT.md)。
这不是下节矩形环线布局，也不调用旧匹配版规划器；当前第一窗前MPC中止，尚无竞速成绩。
会替换专用`convex_seven_togt_experiment`容器，保留旧回放容器。

```bash
./convex_dynamic_seven_window_gazebo/build_plugin.sh
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q 复现/polynomial_dynamic_racing_2023/px4_fixed_plane/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run --no-capture-output -n wyh python -m 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.matrix --outdir 复现/polynomial_dynamic_racing_2023/px4_fixed_plane/results/new_run
```

## 固定平面七窗：只对比当前方法与Wang

2026-10-07删除事后几何/动力学复查的独立仿真消融（事件检查仍保留，不是安全部署入口）：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/comparisons/original_fixed_plane_deadline_no_postcheck/tests convex_timevarying_window/wang_progress_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.comparisons.original_fixed_plane_deadline_no_postcheck.run --outdir convex_timevarying_window/comparisons/original_fixed_plane_deadline_no_postcheck/results/new_run
```

两档结果：低档24.748s通过，高档过5窗后第6窗真实碰撞；旧算法默认未修改。详见新目录报告。

2026-10-07既有共同过窗时刻限制版（原MINCO＋MPC，不是Checked/PVA）同场景入口：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/comparisons/original_fixed_plane_deadline/tests convex_timevarying_window/wang_progress_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.comparisons.original_fixed_plane_deadline.run --outdir convex_timevarying_window/comparisons/original_fixed_plane_deadline/results/new_run
```

原布局0相两档：低档24.876s通过，高档3窗后60s超时；并非完整矩阵。原方法/历史结果不改。
完整证据见该目录`results/deadline_v1_20261007/REPORT.md`。

2026-10-07原始Checked/PVA回归入口（不是MPC，也不是Checked-Soft）：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/comparisons/original_fixed_plane_checked/tests convex_timevarying_window/continuation_racing/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.comparisons.original_fixed_plane_checked.run --outdir convex_timevarying_window/comparisons/original_fixed_plane_checked/results/new_run
```

同原布局0相两档28.284/57.360s均返航，但高频−5.3mm球净空违反，不能称2/2安全通过。
保留旧可停车机制/PVA，仅场景查询适配。详见该目录`results/original_checked_v1_20261007/REPORT.md`。

2026-10-07后续自由局部末端PVAJ版（独立目录，旧版保留）两条件回归：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/wang_free_terminal_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.wang_free_terminal_mpc.run --outdir convex_timevarying_window/wang_free_terminal_mpc/results/new_run
```

非终局末端PVAJ可变，最终任务返航PVA固定；目前两档退出2窗/1窗后失败，无全程圈时。
报告在该目录`results/free_terminal_v1_20261007/REPORT.md`；运行会替换专用实验容器，不操作旧回放。

最新**原不规则位置**、与新paper_core相同六条件的我们现有free_warm入口：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.comparisons.original_fixed_plane_ours.run --outdir convex_timevarying_window/comparisons/original_fixed_plane_ours/results/new_run
```

这六次目前均通过W1后参考耗尽，固定末端自相矛盾，见该目录报告；与下述较早矩形环线结果分开。

最新无中途停车、中心初值三窗版本为独立`wang_center_seed_mpc`；两条件已完成16.432/16.816s，
不宣称全面快于旧版。重跑必须新目录，不覆盖已保存结果或恢复旧矩阵混版：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/wang_center_seed_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.wang_center_seed_mpc.experiment --outdir convex_timevarying_window/wang_center_seed_mpc/results/new_development
```

后续公共过窗时刻修复在独立`wang_progress_mpc`，不向旧暂停矩阵混入新版：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/wang_progress_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.wang_progress_mpc.experiment --outdir convex_timevarying_window/wang_progress_mpc/results/new_development
```

此入口只运行预声明的慢联调与快速回归两条件，不是完整24趟矩阵。

2026-10-07新增独立场景适配与24趟配对协议，详见
[入口说明](../convex_timevarying_window/comparisons/fixed_plane_wang/README.md)。
需要Docker/PX4/Gazebo；只重建专用实验容器，不改旧场景或原方法源码。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/comparisons/fixed_plane_wang/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.comparisons.fixed_plane_wang.run --outdir convex_timevarying_window/comparisons/fixed_plane_wang/results/new_formal
OPENBLAS_NUM_THREADS=1 conda run -n wyh python -m convex_timevarying_window.comparisons.fixed_plane_wang.report convex_timevarying_window/comparisons/fixed_plane_wang/results/new_formal
```

可先以`--pilot`和单独新目录联调；正式输入是Hz和米，不再使用旧运动倍数。
事后单趟用`fixed_plane_wang.audit RUN --method ours`（或`wang`），禁止默认旧赛道审计。

## 环境与输出

```bash
source /home/jack/anaconda3/etc/profile.d/conda.sh
conda activate wyh
```

根 `environment.yml` 来自早期 GAP-Step。SC、SIP、RotSync、FAPP、PhaseGuard 等依赖以各自 README 和 requirements 为准；SIP/Planar-RS/PhaseGuard 的严格检查需要 `python-flint`，缺少时不能当成采样安全通过。

已有结果、checkpoint 和外部复现资料可能被 Git 忽略，检出源码不代表它们存在。新实验使用独立结果目录，保留原运行。GAP-Step 的部分训练配置含 `clean_outputs: true`，会清空所配置输出目录；复用结果前应在新配置中指定新路径或关闭该开关。

## 测试范围

### 2026-10-07：当前Wang自由状态事件v4完整矩阵

仅新增编排，飞行源码须与 `event_v4_20261007/protocol.json` 的63项哈希完全一致。
38条件约需40余分钟；不要同时运行其他飞行或重型历史审计。运行中不修改冻结源码。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -m convex_timevarying_window.comparisons.wang_free_state_mpc.run --outdir convex_timevarying_window/comparisons/wang_free_state_mpc/results/new_frozen
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run --no-capture-output -n wyh python -m convex_timevarying_window.comparisons.wang_free_state_mpc.report convex_timevarying_window/comparisons/wang_free_state_mpc/results/new_frozen
```

仅进程中断恢复使用 `--resume`；不会覆盖已有趟或重飞算法失败。报告器要求status=complete，
历史只读ULog事件复核存入新批次，不改历史原标签。协议在对应comparison目录。

### 2026-10-07：Wang MINCO 自由状态与中心对照

两批必须串行运行（共用 PX4/Gazebo 实例），自由点/中心的区别仅为局部坐标界，均使用同一 MPC。
下列两个运行命令不能并行执行；各自指定全新输出目录，历史批次不覆盖。
当前入口包含事件修复；每趟除原检查外生成 `passage_audit.json`，
主验收必须再通过“七窗正向各一次/整机离面”，不能只读 `gates=7`。
`results/free_v1_20261007` 是修复前冻结结果；`event_v2_20261007` 保留开发结果，
`event_v3_20261007` 是纠正远处下一窗平面误判后的复测，保留高频假反向中止；
`event_v4_20261007` 再将进度检测统一到新鲜测量时间。不混用版本或挑选最快成绩。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/wang_free_state_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -m convex_timevarying_window.wang_free_state_mpc.experiment --outdir convex_timevarying_window/wang_free_state_mpc/results/new_free
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -m convex_timevarying_window.wang_free_state_mpc.experiment --center-only --outdir convex_timevarying_window/wang_free_state_mpc/results/new_center
```

### 2026-10-07：TOGT-MINCO 两条件试测

独立入口，七次 MINCO-s4 + 原优化 MPC，默认跑原始及 r10/t1/a1/120° 两个条件，不是全矩阵。
v1/v2 开发失败和 v3 完成记录分目录保留；使用新目录，不能覆盖已有批次。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/togt_minco_mpc/tests convex_timevarying_window/checked_soft_mpc_isolated/tests convex_timevarying_window/checked_soft_mpc_racing/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -m convex_timevarying_window.togt_minco_mpc.experiment --outdir convex_timevarying_window/togt_minco_mpc/results/new_probe
```

`protocol.json` 冻结配置/源码；在线 100 ms 含检查与 IPC，构图单列。方法 `RESULTS.md`
记录本次结果与历史同条件来源，不能把两趟通过当总体安全保证或速度优势。

### 2026-10-06：优化MPC完整矩阵

使用独立规划进程和等价QP消元；当前只跑优化版，不继续旧未优化MPC批次。
三个开发条件和正式38条件分开保存；正式批次中不改源码/配置。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/checked_soft_mpc_isolated/tests convex_timevarying_window/comparisons/checked_soft_mpc_optimized/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -m convex_timevarying_window.comparisons.checked_soft_mpc_optimized.run --outdir convex_timevarying_window/comparisons/checked_soft_mpc_optimized/results/new_frozen
# 仅在全部计时飞行结束后生成对比图和共同动力学诊断。
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m convex_timevarying_window.comparisons.checked_soft_mpc_optimized.report convex_timevarying_window/comparisons/checked_soft_mpc_optimized/results/new_frozen --historical-dynamics
```

中断续跑加`--resume`并使用原目录；哈希一致才继续，已有原始飞行不重跑。
用户允许轻微名义超限，但碰撞、负净空、未完成七窗返航和100 ms超限仍失败。

### 2026-10-06：MPC替换跟踪的碰撞条件复测

独立入口保留Checked-Soft规划与原赛道设置；每次指定新目录，旧PVA矩阵当前暂停。
只有控制器变更，不自动恢复旧38条件批次。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/checked_soft_mpc_racing/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -m convex_timevarying_window.checked_soft_mpc_racing.px4 --motion-rate 10 --translation-scale 4 --rotation-scale 1 --phase 2.0943951023931953 --outdir convex_timevarying_window/checked_soft_mpc_racing/results/new_hard_case
conda run -n wyh python -m convex_timevarying_window.checked_soft_mpc_racing.report convex_timevarying_window/checked_soft_mpc_racing/results/new_hard_case --outdir convex_timevarying_window/checked_soft_mpc_racing/results/new_comparison
```

MPC为1 s滚动预测、直接总推力/角速度；构图/准备与在线耗时分列，真实净空和接触独立审计。
不能把无接触但未七窗返航的试验计为成功；报告保留所有开发失败和准备期异常。

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

七异形闭环正式轨迹的 Gazebo Harmonic 场景位于 `comparisons/seven_unique_dual_constraint_cem/gazebo/`：

```bash
cd nonconvex_timevarying_window/comparisons/seven_unique_dual_constraint_cem/gazebo
conda run -n wyh python export_world.py
conda run -n wyh python validate_course.py
conda run -n wyh python gazebo_smoke.py
./run_track.sh           # 有 X11 桌面；只加载实体旋转门赛道
./run_track.sh gui       # 流畅展示版
./run_track.sh headless  # 无显示服务器
./setup_manual_control.sh # 首次下载 QGroundControl 与隔离键盘遥控环境
./run_px4_manual.sh      # PX4 SITL + x500 + QGroundControl 鼠标手动飞行
./run_keyboard_teleop.sh # 可选键盘手动飞行
./stop_px4_manual.sh     # 停止 PX4/Gazebo
```

`gazebo_smoke.py` 需要 Docker daemon 权限，使用 Gazebo Harmonic 容器实际加载世界并核对七个关节角速度。赛道资产版仍只定义场地和门；PX4 手动版由 PX4 Gazebo 桥生成官方 x500，使用传感器、飞控、MAVLink 与 QGroundControl，不运行路径规划或轨迹跟踪。离开该目录后返回仓库根目录再执行其他模块命令。

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
## 凸多边形/圆形周期三维窗口

### 在线 Checked-Soft 目标函数变体（2026-10-06）

以下命令从仓库根目录运行；新输出目录必须不存在。批量实验会重启专用PX4实验容器，
不能与其他占用该容器的飞行并行。完成计时飞行后再运行共同ULog动力学审计。

```bash
conda run -n wyh python -m pytest -q convex_timevarying_window/checked_soft_racing/tests convex_timevarying_window/continuation_racing/tests convex_timevarying_window/receding_gate_racing/tests convex_timevarying_window/comparisons/checked_soft_racing/tests
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -m convex_timevarying_window.comparisons.checked_soft_racing.run --outdir convex_timevarying_window/comparisons/checked_soft_racing/results/new_frozen_run
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 conda run -n wyh python -m convex_timevarying_window.comparisons.checked_soft_racing.report convex_timevarying_window/comparisons/checked_soft_racing/results/new_frozen_run --dynamics
```

中断后同一未改源码的批次可给运行器加 `--resume`。协议见
[Checked-Soft](../convex_timevarying_window/comparisons/checked_soft_racing/PROTOCOL.md)。
下面命令属于较早的离线凸TOGT/CEM，不是新在线算法入口。

```bash
pytest -q convex_timevarying_window/tests
conda run -n wyh python -c \
  'from convex_timevarying_window.native_backend import build_native; build_native(force=True)'
conda run --no-capture-output -n wyh python -m convex_timevarying_window.experiment \
  --outdir convex_timevarying_window/togt/results/seven_convex_togt_margin_1p1_body_diameter_20260910
conda run --no-capture-output -n wyh python \
  -m convex_timevarying_window.conditional_dual_constraint_cem.experiment \
  --outdir convex_timevarying_window/conditional_dual_constraint_cem/results/new_run \
  --population 64 --maximum-rounds 50 --repair-max-iterations 600
```
## 2026-10-13 无规划时间上限的单窗升档

环境仍wyh、ACADOS_SOURCE_DIR=/home/jack/wyh/dependencies/acados-togt-v0.5.4及对应lib、单线程BLAS、GZ_SHOW_GUI=0。
入口`python -u -m convex_timevarying_window.comparisons.single_gate_swing.extension --outdir <新目录>`，
默认固定4m从4.5Hz起、固定4Hz从4.5m起，步长.5；任一方法失败后完成配对并停止该路线。
`--start-frequency 5 --start-amplitude 5 --stop-on ours`可接续寻找自由点首次失败，已飞条件不重跑。
本场景默认max_solve_ms=None，在线/诊断无墙钟截止，但保留40s完整任务与数值迭代/见证上限。
新修改验证：`python -m pytest -q convex_timevarying_window/comparisons/single_gate_swing/tests convex_timevarying_window/dynamic_passage_velocity_minco/tests convex_timevarying_window/togt_style_mpc/tests`，32通过。
# 2026-10-13离线横移靠近实验

`comparisons/single_gate_linear_approach/offline_plan.py:compute`先在新目录保存各plan.json，
随后wyh环境、ACADOS_SOURCE_DIR与LD_LIBRARY_PATH按togt_style_mpc配置，线程数1：
`python -u -m convex_timevarying_window.comparisons.single_gate_linear_approach.offline_run --outdir <已有8条plan且尚未实飞的新批目录>`。
不重复运行已有protocol.json目录；只操作convex_seven_togt_experiment。
协议见该方法OFFLINE_PROTOCOL.md。旧在线run入口不再用于当前用户任务。

