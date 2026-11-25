# MSR-DynaTOGT formal 实验报告

## 方法差异

SC-DynaTOGT 对一个初值执行一次局部 L-BFGS-B，动力学限制为软惩罚，优化后主要做诊断。MSR-DynaTOGT 系统生成中心、随机扰动、转弯感知和区域分散初值，维护去重候选池；对超限候选进行高密度采样检查、时间修复、二分缩放和一次完整联合再优化。SC 映射、动态窗口、degree-7 MINCO、四旋翼微分平坦性及单旋翼推力/角速度代价保持一致。

> 本报告中的“可行”仅表示高密度采样可行，不是连续时间严格证明。`optimizer_success` 也没有被当作动力学可行。

## 原生 A0–A3 结果

| 方法 | 运行数 | 窗口合法率 | 采样动力学可行率 | 平均总时间 / s | 平均计算时间 / s |
|---|---:|---:|---:|---:|---:|
| A0 | 775 | 100.0% | 0.0% | 5.377927 | 237.436 |
| A1 | 775 | 100.0% | 0.1% | 5.351334 | 1482.787 |
| A2 | 775 | 100.0% | 100.0% | 5.389145 | 303.535 |
| A3 | 775 | 100.0% | 100.0% | 5.366490 | 2385.702 |

## 公平性

输出同时包含 `matched_starts`（四个消融使用完全相同的初值列表和启动次数）与 `matched_time`（按实测单次耗时回放，在共同时间预算内保留可完成的启动，至少接纳第一次启动）。原生 `native` 行用于展示各方法按定义运行时的实际代价。时间预算比较复用真实已测运行，不用估算值替代墙钟时间。

AtlasDynaTOGT 仅作辅助结构比较：它使用三次 Hermite、各向异性缩放及速度/加速度/jerk 指标，没有本实验的 degree-7 MINCO、单旋翼推力和角速度接口，场景定义也不同。因此没有把它的数值与 A0–A3 直接排名，也没有宣称 MSR 对 Atlas 性能优越。

## 改善与计算成本

A3 相对 A0 的高密度采样动力学可行率变化为 +1.000，平均总时间变化 -0.011437 s，平均计算时间倍率 10.05×。
A0 含采样动力学不可行轨迹，因此上述总时间差只是原始数值差，不能解释为可行解之间的性能优越性。

## 分场景表现

- `full_three`（155 个种子）：A0/A3 采样动力学可行率分别为 `0.0%`/`100.0%`；A3 平均总时间相对 A0 更长或相同（1.8275 -> 1.8321 s）；A0 存在不可行运行，时间差仅作数值记录。
- `hard_thrust_full`（155 个种子）：A0/A3 采样动力学可行率分别为 `0.0%`/`100.0%`；A3 平均总时间相对 A0 更短（6.2914 -> 6.2477 s）；A0 存在不可行运行，时间差仅作数值记录。
- `paper_irregular_six`（155 个种子）：A0/A3 采样动力学可行率分别为 `0.0%`/`100.0%`；A3 平均总时间相对 A0 更短（14.9742 -> 14.9508 s）；A0 存在不可行运行，时间差仅作数值记录。
- `static_single`（155 个种子）：A0/A3 采样动力学可行率分别为 `0.0%`/`100.0%`；A3 平均总时间相对 A0 更长或相同（1.9686 -> 1.9696 s）；A0 存在不可行运行，时间差仅作数值记录。
- `translation_three`（155 个种子）：A0/A3 采样动力学可行率分别为 `0.0%`/`100.0%`；A3 平均总时间相对 A0 更长或相同（1.8279 -> 1.8322 s）；A0 存在不可行运行，时间差仅作数值记录。

## 失败种子及原因

- A0 / `full_three` / seeds=0-154 （155 个）：sampled_dynamic_limit_violation | sampled_single_rotor_thrust_upper_limit
- A0 / `hard_thrust_full` / seeds=0-154 （155 个）：sampled_dynamic_limit_violation | sampled_single_rotor_thrust_upper_limit
- A0 / `paper_irregular_six` / seeds=0-154 （155 个）：sampled_dynamic_limit_violation | sampled_single_rotor_thrust_upper_limit
- A0 / `static_single` / seeds=0-154 （155 个）：sampled_dynamic_limit_violation | sampled_single_rotor_thrust_upper_limit
- A0 / `translation_three` / seeds=0-154 （155 个）：sampled_dynamic_limit_violation | sampled_single_rotor_thrust_upper_limit
- A1 / `full_three` / seeds=0-154 （155 个）：sampled_dynamic_limit_violation | sampled_single_rotor_thrust_upper_limit
- A1 / `hard_thrust_full` / seeds=0-154 （155 个）：sampled_dynamic_limit_violation | sampled_single_rotor_thrust_upper_limit
- A1 / `paper_irregular_six` / seeds=0-154 （155 个）：sampled_dynamic_limit_violation | sampled_single_rotor_thrust_upper_limit
- A1 / `static_single` / seeds=0-52,54-154 （154 个）：sampled_dynamic_limit_violation | sampled_single_rotor_thrust_upper_limit
- A1 / `translation_three` / seeds=0-154 （155 个）：sampled_dynamic_limit_violation | sampled_single_rotor_thrust_upper_limit

## 已知限制

- 高密度采样可能漏掉采样点之间的瞬时峰值，不能替代连续时间认证。
- 局部时间修复沿固定的分段缩放方向二分，不保证得到全局最小可行时间。
- 多初值和修复会增加计算成本；L-BFGS-B 仍是非凸局部优化。
- 当前范围仍是无洞、无自交、偏置后单连通的窗口，按指定顺序各穿越一次。
- formal 使用 SC-DynaTOGT 的历史代价停止准则，不设置人为迭代上限。
- Atlas 辅助说明：AtlasDynaTOGT 使用三次 Hermite、各向异性窗口缩放及速度/加速度/jerk指标；MSR/SC 使用 degree-7 MINCO、均匀缩放、四旋翼微分平坦性、单旋翼推力和角速度。动力学模型、场景和接口不同，因此不能直接宣称性能优越。


## 每张结果图的通俗解释

以下解释与独立文件 `FIGURE_EXPLANATIONS.md` 内容一致。


先记住两点：图中的 `sampled=True` 只表示高密度采样节点通过限制，不是连续时间数学证明；只有窗口合法且采样动力学可行的轨迹，才适合继续比较飞行时间。

## `figures/trajectory_comparison_static_single.png`

这张三维图展示 `static_single` 在代表种子 `0` 下的四条轨迹。黑色圆点/叉号是起终点，橙色轮廓是 A3 实际穿越时刻的物理窗口，A0–A3 彩线是四种方法的飞行路线。

通俗地看：先检查图例中的 `sampled`。如果是 `False`，即使曲线更短，也像一辆抄近路但发动机超载的车，不能排在可行轨迹前面；都为 `True` 时，才看 `T`，越小表示飞得越快。

- A0：总时间 `1.968553 s`，窗口合法=`True`，高密度采样动力学可行=`False`。
- A1：总时间 `1.968553 s`，窗口合法=`True`，高密度采样动力学可行=`False`。
- A2：总时间 `1.969634 s`，窗口合法=`True`，高密度采样动力学可行=`True`。
- A3：总时间 `1.969634 s`，窗口合法=`True`，高密度采样动力学可行=`True`。
- 本图的 A0 没有通过采样动力学限制，因此 A0 与 A3 的时间差只能描述数值，不能称为可行轨迹之间的速度优势。

图的限制：它只画一个代表种子，不能替代所有正式种子的统计；窗口轮廓取自 A3 的穿越时刻，不表示四种方法在完全相同时刻看到的窗口姿态。

## `figures/trajectory_comparison_translation_three.png`

这张三维图展示 `translation_three` 在代表种子 `0` 下的四条轨迹。黑色圆点/叉号是起终点，橙色轮廓是 A3 实际穿越时刻的物理窗口，A0–A3 彩线是四种方法的飞行路线。

通俗地看：先检查图例中的 `sampled`。如果是 `False`，即使曲线更短，也像一辆抄近路但发动机超载的车，不能排在可行轨迹前面；都为 `True` 时，才看 `T`，越小表示飞得越快。

- A0：总时间 `1.827946 s`，窗口合法=`True`，高密度采样动力学可行=`False`。
- A1：总时间 `1.827946 s`，窗口合法=`True`，高密度采样动力学可行=`False`。
- A2：总时间 `1.832186 s`，窗口合法=`True`，高密度采样动力学可行=`True`。
- A3：总时间 `1.832186 s`，窗口合法=`True`，高密度采样动力学可行=`True`。
- 本图的 A0 没有通过采样动力学限制，因此 A0 与 A3 的时间差只能描述数值，不能称为可行轨迹之间的速度优势。

图的限制：它只画一个代表种子，不能替代所有正式种子的统计；窗口轮廓取自 A3 的穿越时刻，不表示四种方法在完全相同时刻看到的窗口姿态。

## `figures/trajectory_comparison_full_three.png`

这张三维图展示 `full_three` 在代表种子 `0` 下的四条轨迹。黑色圆点/叉号是起终点，橙色轮廓是 A3 实际穿越时刻的物理窗口，A0–A3 彩线是四种方法的飞行路线。

通俗地看：先检查图例中的 `sampled`。如果是 `False`，即使曲线更短，也像一辆抄近路但发动机超载的车，不能排在可行轨迹前面；都为 `True` 时，才看 `T`，越小表示飞得越快。

- A0：总时间 `1.827531 s`，窗口合法=`True`，高密度采样动力学可行=`False`。
- A1：总时间 `1.827531 s`，窗口合法=`True`，高密度采样动力学可行=`False`。
- A2：总时间 `1.832216 s`，窗口合法=`True`，高密度采样动力学可行=`True`。
- A3：总时间 `1.832104 s`，窗口合法=`True`，高密度采样动力学可行=`True`。
- 本图的 A0 没有通过采样动力学限制，因此 A0 与 A3 的时间差只能描述数值，不能称为可行轨迹之间的速度优势。

图的限制：它只画一个代表种子，不能替代所有正式种子的统计；窗口轮廓取自 A3 的穿越时刻，不表示四种方法在完全相同时刻看到的窗口姿态。

## `figures/trajectory_comparison_paper_irregular_six.png`

这张三维图展示 `paper_irregular_six` 在代表种子 `0` 下的四条轨迹。黑色圆点/叉号是起终点，橙色轮廓是 A3 实际穿越时刻的物理窗口，A0–A3 彩线是四种方法的飞行路线。

通俗地看：先检查图例中的 `sampled`。如果是 `False`，即使曲线更短，也像一辆抄近路但发动机超载的车，不能排在可行轨迹前面；都为 `True` 时，才看 `T`，越小表示飞得越快。

- A0：总时间 `14.974203 s`，窗口合法=`True`，高密度采样动力学可行=`False`。
- A1：总时间 `14.908058 s`，窗口合法=`True`，高密度采样动力学可行=`False`。
- A2：总时间 `15.013377 s`，窗口合法=`True`，高密度采样动力学可行=`True`。
- A3：总时间 `14.950942 s`，窗口合法=`True`，高密度采样动力学可行=`True`。
- 本图的 A0 没有通过采样动力学限制，因此 A0 与 A3 的时间差只能描述数值，不能称为可行轨迹之间的速度优势。

图的限制：它只画一个代表种子，不能替代所有正式种子的统计；窗口轮廓取自 A3 的穿越时刻，不表示四种方法在完全相同时刻看到的窗口姿态。

## `figures/trajectory_comparison_hard_thrust_full.png`

这张三维图展示 `hard_thrust_full` 在代表种子 `0` 下的四条轨迹。黑色圆点/叉号是起终点，橙色轮廓是 A3 实际穿越时刻的物理窗口，A0–A3 彩线是四种方法的飞行路线。

通俗地看：先检查图例中的 `sampled`。如果是 `False`，即使曲线更短，也像一辆抄近路但发动机超载的车，不能排在可行轨迹前面；都为 `True` 时，才看 `T`，越小表示飞得越快。

- A0：总时间 `6.291402 s`，窗口合法=`True`，高密度采样动力学可行=`False`。
- A1：总时间 `6.223433 s`，窗口合法=`True`，高密度采样动力学可行=`False`。
- A2：总时间 `6.298314 s`，窗口合法=`True`，高密度采样动力学可行=`True`。
- A3：总时间 `6.247744 s`，窗口合法=`True`，高密度采样动力学可行=`True`。
- 本图的 A0 没有通过采样动力学限制，因此 A0 与 A3 的时间差只能描述数值，不能称为可行轨迹之间的速度优势。

图的限制：它只画一个代表种子，不能替代所有正式种子的统计；窗口轮廓取自 A3 的穿越时刻，不表示四种方法在完全相同时刻看到的窗口姿态。

## `figures/total_time_comparison.png`

四根柱是 A0–A3 在全部运行上的平均飞行时间：A0=5.377927 s、A1=5.351334 s、A2=5.389145 s、A3=5.366490 s。柱越低表示轨迹平均越快。但必须同时对照动力学可行率图：如果某方法大量不可行，低时间可能只是靠超推力换来的，不能单独宣布它更好。

## `figures/computation_time_comparison.png`

柱高表示求解器实际花费的平均墙钟时间：A0=237.436 s、A1=1482.787 s、A2=303.535 s、A3=2385.702 s。A1/A3 要尝试多个初值，A2/A3 还要修复和再次优化，所以通常更高。它回答的是“为了得到结果多等多久”，不是无人机飞多久。

## `figures/sampled_dynamic_feasibility_rate.png`

纵轴是通过高密度动力学检查的运行比例：A0=0.0%、A1=0.1%、A2=100.0%、A3=100.0%。越接近 100% 越稳定。它直接反映时间修复是否减少推力、角速度或速度超限，但仍然只是采样证据。

## `figures/repair_thrust_before_after.png`

每组红柱是一个“方法×场景”组的修复前平均最大单旋翼推力，绿柱是修复后；默认上限是 5 N。共 1550 个触发修复的 A2/A3 原生运行，最大单旋翼推力的组合平均由 5.0151 N 变为 4.9998 N。绿色降到 5 N 附近或以下，表示放慢轨迹有效削弱峰值。若某次修复由单旋翼下限而非上限触发，仅看最大值不能说明全部原因，应回到 runs.csv 查看最小推力和 failure_reasons。