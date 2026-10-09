# Checked-Soft：保留 checked 准入的软净空竞速

2026-10-06 新建的并列算法。只改变目标函数，不替代 checked 的安全检查，
不改变穿窗事件、规划窗口数、控制接口、动力学界和失败判据。

## 算法

继承 `continuation_racing.guard.CheckedPlanner`；本次 PX4 实验配置为接下来两扇窗和一个可停留终点，
联合优化时间、穿越点、速度和加速度；五次多项式连接；每 0.2 s 滚动重规划；
PX4 PVA 参考 100 Hz，预览 0.05 s。

原目标是时间与积分 jerk²。新增时间积分净空代价：

`J = sum(T) + 0.0006 integral ||jerk||² dt + integral sum(phi(d²)) dt`

`phi(d²) = lambda * max(0, (r_pref²-d²)/(r_pref²-r_hard²))³`

- `d²` 是候选绝对时刻、候选位置到真实运动边框中心线的平方距离；
  多边形取边段距离，圆窗取圆环距离。不罚开口中心或整个窗口平面。
- `r_hard = drone_radius + sleeve_radius + 0.35 m`。
- `r_pref = r_hard + 0.15 m`；`lambda = 1` 固定，不按门号/场景调参。
- 在原硬余量边界，每接近一扇框每秒增加 1 s 等价代价；
  名义机体净空达到 0.50 m 时该项为零。软目标允许达不到 0.50 m，
  但不允许用有限罚分代替 0.35 m 硬余量/准入。
- 在原均匀节点上积分，使用原 NLP 的相邻边框集合；完整局部轨迹检查仍检查所有框。
  三次 hinge 是 C2，最近多边形边段距离本身仍为分段光滑。

它等价于额外软约束 `d² + sigma >= r_pref², sigma >= 0`，并惩罚
`lambda * (sigma/(r_pref²-r_hard²))³`。由于 sigma 的最优值可直接写成
`max(0, r_pref²-d²)`，实现中解析消去松弛变量，**没有增加决策变量或约束数量**；
两窗加终点的最大自由变量数仍为 19。也没有改变每个窗口的硬安全集合。

保留每次候选约 10 ms 间隔的整段名义检查、原自适应补点、完整周期 <100 ms
和已有参考衔接方式。**软约束不是安全证明，也不保证更快**：它可能减少擦边
导致的拒绝/跟踪风险，也可能增加路长、求解耗时或引起不同局部极小值。

## 代码与运行

- 新目标与参数：`planner.py`；共享基类仅新增默认返回零的 `extra_stage_cost` 接口。
- 旧 checked 可原样运行；只有 `--checked-soft` 才启用新目标。
- 参数为零时目标、约束与旧 checked 完全相同，测试覆盖该消融。

```bash
conda run -n wyh python -m pytest -q convex_timevarying_window/checked_soft_racing/tests convex_timevarying_window/continuation_racing/tests
conda run -n wyh python -m convex_timevarying_window.receding_gate_racing.px4_experiment --launch --checked-soft --tracking-preview .05 --outdir /absolute/new/run
```

协议与跨方法报告位于 `../comparisons/checked_soft_racing/`。不覆盖历史结果。
当前历史口径使用中心过平面事件；整机包围球碰撞按独立真值轨迹审计。
动态场景采样通过不等于连续安全认证或真实飞控鲁棒性保证。

原定38条件批次中发现过任务前仿真机体不响应爬升的初始化异常；独立
`comparisons/checked_soft_racing/adjudicate.py` 仅凭规划开始前的ULog证据判定，
原始批次不改写，补充表同时保存原尝试与一次重试。正常飞行后的碰撞/超时不重试。
