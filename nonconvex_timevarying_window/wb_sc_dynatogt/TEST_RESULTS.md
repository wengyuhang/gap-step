# WBSC-DynaTOGT 验证记录（旧公式结果已归档）

> 2026-08-01：根据“姿态必须在规划中主动调整”的要求，默认规划已修订为 `[K,D,Y]` 联合优化。MINCO、微分平坦性姿态和长方体约束均在求解器每次迭代中计算；原动力学限制保持软约束，新增机体条件为硬约束。下面更早的结果均不能作为当前主算法结论，formal 需重新运行。

## 2026-08-01 修订版检查

```bash
python -m pytest -q nonconvex_timevarying_window/wb_sc_dynatogt/tests
# 17 passed in 1.14s

python -m pytest -q nonconvex_timevarying_window/sc_dynatogt/tests
# 原算法回归通过

python -m compileall -q nonconvex_timevarying_window/wb_sc_dynatogt
# exit 0
```

新增回归测试直接扰动规划变量中的穿越点和 yaw，确认求解器内部的姿态长方体约束值随之变化；这用于防止实现退化成“先规划、后检查”。

当前姿态联合规划 smoke：[`results/smoke/20260801_061508_smoke/`](results/smoke/20260801_061508_smoke/)。manifest 确认 `optimize_yaw=true`、`hard_collision_constraints=true`、`collision_weight=0`，即原动力学软罚保留，新增机体条件直接作为硬约束。静态窄 L 在 22 次 SLSQP 迭代后收敛并通过完整长方体检查；静态 U/曲线和动态三窗口在 smoke 的 24 次上限内尚未收敛，但当前解的几何检查均通过。该运行验证姿态约束闭环可执行，不作为 formal 性能结论。

目录 [`results/smoke/20260801_051119_smoke/`](results/smoke/20260801_051119_smoke/) 属于随后被否定的 `[K,D]` 事后姿态版本，仅保留作历史记录。

验证日期：2026-07-31。

## 代码验证

```bash
pytest -q nonconvex_timevarying_window/wb_sc_dynatogt/tests
# 13 passed in 1.10s

pytest -q nonconvex_timevarying_window/sc_dynatogt/tests
# 111 passed in 31.73s

pytest -q
# 46 passed in 3.16s

python -m compileall -q nonconvex_timevarying_window/wb_sc_dynatogt
# exit 0
```

13 项新测试包括：长方体尺寸/坐标旋转，yaw-MINCO 连续性，非凸边界间隙梯度，姿态与窗口时间梯度，持久化，以及两个反例：

- 中心点合法，但长方体碰窗；
- `0.315 m` 球和枚举的 361 个纯 yaw 姿态都无法通过窄缝，而加入 pitch 后姿态长方体可以通过。

## smoke

```bash
MPLCONFIGDIR=/tmp/mpl-wbsc \
python -m nonconvex_timevarying_window.wb_sc_dynatogt.experiments --suite smoke
```

完成目录：[`results/smoke/20260731_093238_smoke/`](results/smoke/20260731_093238_smoke/)。共 12 个方法运行（3 个场景 × 4 个方法），6 类窗口各用 2,000 个中心—完整姿态样本。姿态长方体相对球模型的候选恢复比例为 `7.15%`–`13.30%`；WBSC 轨迹的 pitch 峰值为 `1.060`–`1.242 rad`。

## formal

```bash
MPLCONFIGDIR=/tmp/mpl-wbsc \
python -m nonconvex_timevarying_window.wb_sc_dynatogt.experiments --suite formal
```

完成目录：[`results/formal/20260731_094740_formal/`](results/formal/20260731_094740_formal/)。进程退出码为 0；`method_runs.csv` 有 860 条数据行和 860 个唯一 `(family, seed, method)` 组合：

- 静态窄 L：30 种子 × 4 方法；
- 静态 U/曲线：30 种子 × 4 方法；
- 动态 L→U→星形：155 种子 × 4 方法。

6 类窗口各采样 100,000 个中心—roll/pitch/yaw 组合，球模型丢失但姿态长方体可行的观察比例为：

| 窗口 | 恢复比例 | Wilson 95% 区间 |
|---|---:|---:|
| L | 10.688% | [10.498%, 10.881%] |
| U | 9.174% | [8.997%, 9.354%] |
| 五角星 | 14.428% | [14.212%, 14.647%] |
| Limacon | 7.367% | [7.207%, 7.531%] |
| 波浪 | 8.484% | [8.313%, 8.658%] |
| 直线/Bezier 混合 | 10.863% | [10.672%, 11.057%] |

按“优化器收敛且规定穿越时刻全长方体合法”统计的整体观察值为：

| 方法 | 安全成功 | 成功率 | Wilson 95% 区间 |
|---|---:|---:|---:|
| 原 SC 球模型 | 0/215 | 0.000% | [0.000%, 1.755%] |
| 点模型 | 19/215 | 8.837% | [5.730%, 13.389%] |
| 长方体固定 yaw | 1/215 | 0.465% | [0.082%, 2.587%] |
| 完整 WBSC-DynaTOGT | 19/215 | 8.837% | [5.730%, 13.389%] |

这些数值只是当前固定计算预算下的观察值。点模型仍是不安全的乐观上界；原球形内缩为空的任务没有从分母剔除；formal 没有设置“新方法必须胜出”的验收阈值。

formal 使用 20 个 `spawn` 工作进程，避免 `fork` 继承 BLAS/PyTorch 线程池导致反向传播死锁。每个方法使用同一 40 次 L-BFGS-B 迭代上限和每段 6 个优化节点；最终动力学极值使用每段至少 33 个节点重算。
