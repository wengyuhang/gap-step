# MSR-DynaTOGT smoke 实验报告

## 方法差异

SC-DynaTOGT 对一个初值执行一次局部 L-BFGS-B，动力学限制为软惩罚，优化后主要做诊断。MSR-DynaTOGT 系统生成中心、随机扰动、转弯感知和区域分散初值，维护去重候选池；对超限候选进行高密度采样检查、时间修复、二分缩放和一次完整联合再优化。SC 映射、动态窗口、degree-7 MINCO、四旋翼微分平坦性及单旋翼推力/角速度代价保持一致。

> 本报告中的“可行”仅表示高密度采样可行，不是连续时间严格证明。`optimizer_success` 也没有被当作动力学可行。

## 原生 A0–A3 结果

| 方法 | 运行数 | 窗口合法率 | 采样动力学可行率 | 平均总时间 / s | 平均计算时间 / s |
|---|---:|---:|---:|---:|---:|
| A0 | 5 | 100.0% | 0.0% | 6.161245 | 7.161 |
| A1 | 5 | 100.0% | 20.0% | 5.970820 | 31.954 |
| A2 | 5 | 100.0% | 100.0% | 6.226261 | 18.864 |
| A3 | 5 | 100.0% | 100.0% | 6.023721 | 75.148 |

## 公平性

输出同时包含 `matched_starts`（四个消融使用完全相同的初值列表和启动次数）与 `matched_time`（按实测单次耗时回放，在共同时间预算内保留可完成的启动，至少接纳第一次启动）。原生 `native` 行用于展示各方法按定义运行时的实际代价。时间预算比较复用真实已测运行，不用估算值替代墙钟时间。

AtlasDynaTOGT 仅作辅助结构比较：它使用三次 Hermite、各向异性缩放及速度/加速度/jerk 指标，没有本实验的 degree-7 MINCO、单旋翼推力和角速度接口，场景定义也不同。因此没有把它的数值与 A0–A3 直接排名，也没有宣称 MSR 对 Atlas 性能优越。

## 改善与计算成本

A3 相对 A0 的高密度采样动力学可行率变化为 +1.000，平均总时间变化 -0.137524 s，平均计算时间倍率 10.49×。
A0 含采样动力学不可行轨迹，因此上述总时间差只是原始数值差，不能解释为可行解之间的性能优越性。

## 分场景表现

- `full_three`：A0/A3 采样动力学可行分别为 `False`/`True`；A3 总时间相对 A0 更短（2.9815 -> 2.9639 s）；A0 不可行，时间差仅作数值记录。
- `hard_thrust_full`：A0/A3 采样动力学可行分别为 `False`/`True`；A3 总时间相对 A0 更短（6.6492 -> 6.6390 s）；A0 不可行，时间差仅作数值记录。
- `paper_irregular_six`：A0/A3 采样动力学可行分别为 `False`/`True`；A3 总时间相对 A0 更短（16.0790 -> 15.8019 s）；A0 不可行，时间差仅作数值记录。
- `static_single`：A0/A3 采样动力学可行分别为 `False`/`True`；A3 总时间相对 A0 更短（2.0047 -> 1.9817 s）；A0 不可行，时间差仅作数值记录。
- `translation_three`：A0/A3 采样动力学可行分别为 `False`/`True`；A3 总时间相对 A0 更短（3.0918 -> 2.7322 s）；A0 不可行，时间差仅作数值记录。

## 失败种子及原因

- A0 / `static_single` / seed=0：sampled_dynamic_limit_violation | sampled_single_rotor_thrust_upper_limit
- A1 / `static_single` / seed=0：sampled_dynamic_limit_violation | sampled_single_rotor_thrust_upper_limit
- A0 / `translation_three` / seed=0：sampled_dynamic_limit_violation | sampled_single_rotor_thrust_lower_limit
- A1 / `translation_three` / seed=0：sampled_dynamic_limit_violation | sampled_single_rotor_thrust_lower_limit | sampled_single_rotor_thrust_upper_limit
- A0 / `full_three` / seed=0：sampled_dynamic_limit_violation | sampled_single_rotor_thrust_lower_limit | sampled_single_rotor_thrust_upper_limit
- A1 / `full_three` / seed=0：sampled_dynamic_limit_violation | sampled_single_rotor_thrust_lower_limit
- A0 / `paper_irregular_six` / seed=0：sampled_dynamic_limit_violation | sampled_single_rotor_thrust_upper_limit
- A1 / `paper_irregular_six` / seed=0：sampled_dynamic_limit_violation | sampled_single_rotor_thrust_upper_limit
- A0 / `hard_thrust_full` / seed=0：sampled_dynamic_limit_violation | sampled_single_rotor_thrust_upper_limit

## 已知限制

- 高密度采样可能漏掉采样点之间的瞬时峰值，不能替代连续时间认证。
- 局部时间修复沿固定的分段缩放方向二分，不保证得到全局最小可行时间。
- 多初值和修复会增加计算成本；L-BFGS-B 仍是非凸局部优化。
- 当前范围仍是无洞、无自交、偏置后单连通的窗口，按指定顺序各穿越一次。
- smoke 为控制快速验证成本，将每次 L-BFGS-B 上限设为 24 次迭代；`optimizer_success=false` 可能表示达到该烟测上限，必须与独立采样可行性分开解读。
- Atlas 辅助说明：AtlasDynaTOGT 使用三次 Hermite、各向异性窗口缩放及速度/加速度/jerk指标；MSR/SC 使用 degree-7 MINCO、均匀缩放、四旋翼微分平坦性、单旋翼推力和角速度。动力学模型、场景和接口不同，因此不能直接宣称性能优越。
