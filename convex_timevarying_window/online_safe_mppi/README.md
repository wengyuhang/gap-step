# VT-GP-MPPI：动态七窗口在线安全竞速

本目录实现一个只针对当前凸动态七窗口赛道的简洁在线方法：**可行性触发的过窗点 MPPI**（Viability-Triggered Gate-Point MPPI, VT-GP-MPPI）。它保留 TOGT “联合选窗内穿越点与到达时刻”的思想，但不在每个控制周期求整圈轨迹。

一句话版本：上层在窗口局部坐标中选一个小幅切弯点并锁定，同时持续更新动态窗口的预计穿越时刻；下层用 1.2 s GPU MPPI 执行，局部屏障和最终一步验证决定命令能否真正下发。

## 算法

1. **点/时联合候选**：对 7 个候选时刻查询解析窗口位姿，以中心和几何切弯方向之间的 5 个点为候选，用真正执行的“进近—法向穿越—法向离框”折线代价排序。自由点限制在中心 0.10 m 信赖域内，W5 固定用中心。
2. **锁定局部点，动态更新时刻**：过窗点以窗口局部坐标保存，因此会跟随已知窗口运动。普通重规划只更新到达时刻，不重新追逐另一个局部点；只有安全 rollout 比例过低时允许换点，长时间停滞则退回中心。
3. **事件触发重规划**：合法过门、0.8 s 计划年龄、rollout 可行率下降、穿越承诺裕量不足和长时间停滞都会触发。因此不是“发现要撞了才规划”。
4. **局部安全 MPPI**：1024 条 rollout，24 步，每步 50 ms。碰框、越界、姿态代理限制、速度/加速度/jerk/snap 或绕过当前门的 rollout 被硬剔除。穿越时刻只是软相位偏好，不会为了追一个过期时钟而强制降速。
5. **可达集裁剪**：MPPI 只查询 1.2 s 内可能进入无人机可达球的门框。被排除的门同时考虑了当前速度、最大加速度、门框外接半径和窗口最大平移速度，不是按索引随意忽略。
6. **执行前最终验证**：局部门框屏障、顺序平面过滤和高度屏障组合后，对最终命令重新做一步离散安全验证。找不到已验证命令就安全中止，不执行“最不坏但仍不安全”的动作。

在线环不做整圈检查。5 ms 全轨迹球体—门框审计只在每次实验结束后独立运行，不进入控制决策和在线计时。

## 冻结结果

- VT-GP-MPPI：`10/10` 完成，平均 `73.715 s`，最小稠密采样净空 `0.2289 m`。
- 中心点对照：`9/10` 完成；种子 8 在找不到已验证一步命令时中止，没有碰撞。
- 双方共同完成的 9 对中，联合选点 6 胜、2 负、1 平，平均快 `0.0778 s` (`0.105%`)；95% bootstrap 区间 `[-0.667,+0.578] s`，单侧 Wilcoxon `p=0.270`。这是小幅、不显著的时间改善，不能写成已证明普遍更快。
- 主方法选点规划器单次最大 `2.173 ms`，MPPI 单次最大 `94.372 ms`，完整控制周期最大 `99.749 ms`，没有超过 100 ms。

详细结果、负结果和验收口径见 [RESULTS.md](RESULTS.md)。

## 复现

```bash
conda run --no-capture-output -n wyh python -m \
  convex_timevarying_window.online_safe_mppi.experiment \
  --seeds 10 --methods proposed --rollouts 1024 \
  --outdir convex_timevarying_window/online_safe_mppi/results/reproduce_proposed

conda run --no-capture-output -n wyh python -m \
  convex_timevarying_window.online_safe_mppi.experiment \
  --seeds 10 --methods center_periodic --rollouts 1024 \
  --outdir convex_timevarying_window/online_safe_mppi/results/reproduce_center

conda run --no-capture-output -n wyh python -m \
  convex_timevarying_window.online_safe_mppi.analyze_results \
  convex_timevarying_window/online_safe_mppi/results/reproduce_proposed \
  convex_timevarying_window/online_safe_mppi/results/reproduce_center \
  convex_timevarying_window/online_safe_mppi/results/reproduce_comparison

conda run -n wyh pytest -q \
  convex_timevarying_window/online_safe_mppi/tests
```

`center_periodic` 是历史命名；当前实现与主方法使用同一事件触发规则，不是 0.2 s 强制周期规划。

## 证据边界

- 当前证据只覆盖已知解析周期运动、当前一条七窗口赛道、简化加速度接口闭环仿真。
- 机体安全使用半径 `0.3794 m` 的方向无关外接球，不是 x500 姿态相关网格碰撞。
- 最终一步盾牌是离散重验证，5 ms 审计是数值稠密采样；两者都不是连续域不变集证明。
- `99.749 ms` 是 RTX A4000 与普通 Linux/CUDA 上的观测最大值，余量只有 `0.251 ms`，不等于实时操作系统上的最坏情况保证。
- 这不是 PX4/Gazebo 闭环或真机结果。

## 研究来源

设计借鉴了 [TOGT 的时空联合过门参数化](https://arxiv.org/abs/2309.06837)、[reference-free MPPI 无人机竞速](https://arxiv.org/abs/2509.14726)、[Shield-MPPI](https://arxiv.org/abs/2302.11719) 和 [GS-MPPI 动态环境采样控制](https://arxiv.org/abs/2410.02154)。本实现是面向本赛道的独立简化组合，不宣称复现这些论文。
