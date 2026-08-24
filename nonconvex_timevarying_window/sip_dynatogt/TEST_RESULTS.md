# SIP-DynaTOGT 测试记录

最后更新：2026-08-24。以下数值均来自 `wyh` Conda 环境中的实际命令，不是估计值。

## 新包测试

```text
$ pytest -q nonconvex_timevarying_window/sip_dynatogt/tests
..............                                                           [100%]
14 passed in 6.89s
```

覆盖内容：

- Line、CircularArc、Bézier 和非有理 B-spline 的区间包含性，包括跨 knot span 的并集包含；
- 周期平移、RPY 姿态和均匀尺度运动；
- MINCO 导数、hover 平坦性、角速度、总推力和单桨推力的 Arb 包含；
- 明显安全/碰撞的整机长方体边框距离；
- 端点与中点都安全，但四分之一时刻碰撞的采样漏检；
- 采样点速度为零，但采样间存在速度窄峰；
- 采样点只有 hover 推力，但采样间存在单桨推力窄峰；
- 姿态的 heading/body-z 平坦性分母在采样间为零；
- 区间预算用尽时必须返回 `UNRESOLVED`；
- 静态和动态单窗 `[K,D] -> SLSQP -> certify` 端到端求解；
- 无 pickle 运行产物、SHA-256 校验和 `verify` 重放；
- 废案 Exact-Area Experiment B 的「质心安全、整机提前碰撞」保留反例。

## 正式静态/动态非凸窗口实验

命令：

```bash
python -m nonconvex_timevarying_window.sip_dynatogt.experiments \
  --suite formal \
  --outdir nonconvex_timevarying_window/sip_dynatogt/results/formal \
  --slsqp-iterations 120 \
  --exchange-iterations 12 \
  --max-cells 200000
```

两个案例均使用 canonical 非凸 L 形真实边界、默认整机半尺寸 `(0.26504, 0.26504, 0.05890) m`、`delta=0.015 m` 和全部默认动力学硬界。

| 案例 | 状态 | 总时间 (s) | 交换轮数 | 认证单元 | 最大深度 | 最小安全平方余量 (m²) | 最小动力学残差余量 |
|---|---:|---:|---:|---:|---:|---:|---:|
| `static_1gate` | `CERTIFIED_FEASIBLE` | 1.971079696 | 10 | 153,586 | 19 | 2.103929411e-6 | 5.590625539e-6 |
| `translation_1gate` | `CERTIFIED_FEASIBLE` | 1.979542789 | 10 | 110,948 | 17 | 5.625859337e-5 | 6.124788999e-6 |

「动力学残差余量」只是不同物理量残差的最小浮点诊断，不应当作统一物理单位解读；状态本身由 Arb 上界判定。

## 六窗口宽域压力对比

最终结果和完整审计数据见
[EXPERIMENT_REPORT.md](../comparisons/sc_sip_fast_closed_loop/results/wide_scrambled_certified_final/EXPERIMENT_REPORT.md)。
修订后严格区分两种结论：

- 连续域正安全残差证明的 `15 mm` 净距违规；
- 原始 Bezier 边界点严格位于有向长方体内部所证明的实体相交。

原 SC 候选在 400 次迭代上限处停止，总时间为 `16.222846275 s`，已找到内部深度
`6.991570 mm` 的实体相交点。从该点继续优化 50 次后触发原 SC 收敛准则，
总时间为 `16.227510527 s`，实体相交仍存在，审计内部深度为 `14.301884 mm`。

SIP 轨迹总时间为 `16.167545447 s`，以 128-bit Arb 检查 `1,311,838` 个区间单元后返回
`CERTIFIED_FEASIBLE`，高密度诊断最小几何距离为 `0.021324673 m`。该对比是专门设计的
压力测试，只证明这一 SC 失败案例，不构成一般碰撞率的统计结论。

## 独立重放

精确有理系数和跨 B-spline knot 包含修正后，从保存的 JSON/NPZ 重建模型并重算：

```text
static_1gate:
  stored_status = CERTIFIED_FEASIBLE
  recomputed.status = CERTIFIED_FEASIBLE
  checked_cells = 153586
  status_matches = true

translation_1gate:
  stored_status = CERTIFIED_FEASIBLE
  recomputed.status = CERTIFIED_FEASIBLE
  checked_cells = 110948
  status_matches = true
```

## 回归测试

```text
$ pytest -q
46 passed in 2.59s

$ pytest -q nonconvex_timevarying_window/sc_dynatogt/tests
111 passed in 34.24s

$ pytest -q nonconvex_timevarying_window/msr_dynatogt/tests
8 passed in 40.33s

$ pytest -q nonconvex_timevarying_window/atlas_dynatogt/tests
7 passed in 18.75s
```

仓库根 `pytest.ini` 将默认 `testpaths` 限定为 `gap_step/tests`，因此非凸窗口的三组旧方法回归按目录显式另行执行。
