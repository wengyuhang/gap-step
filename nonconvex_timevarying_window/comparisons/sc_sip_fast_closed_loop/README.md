# SC 与 SIP 宽域快速闭环对比

该实验在同一条六窗口、宽域、不规则三维闭环上运行 `SC-DynaTOGT` 和
`SIP-DynaTOGT`。起点与终点完全相同，窗口顺序为：

```text
W3(B-spline) -> W4(arc capsule) -> W1(circle) ->
W5(Bezier diamond) -> W0(L polygon) -> W2(Bezier notch) -> start/finish
```

初始窗口中心跨度为 `27 x 26 x 10 m`，穿越顺序与存储编号故意不同。
六扇窗口均具有独立的周期平移、完整 RPY 旋转和均匀缩放；周期为
`2.05–3.10 s`，单轴平移振幅最高 `1.85 m`，单轴旋转振幅最高 `0.88 rad`，
缩放振幅最高 `0.60`。所有窗口的最小尺度均不小于 `0.40`，不会缩放消失，
并且预处理产生了非空的整机可通行区域。

SC 与 SIP 使用同一长方体机体，半尺寸为
`(0.26504, 0.26504, 0.05890) m`，净裕度为 `0.015 m`。SC 的安全多边形使用
长方体外接球半径加净裕度作为固定世界中心内缩：对最小缩放
`s_min`，局部内缩设置为 `d_world / s_min`，从而保证完整周期内的世界
中心距离不小于 `d_world`。最终碰撞检查不使用外接球，而是两种方法共享同一个
姿态相关的长方体—原始边界距离公式。

SC 与 SIP 使用同一个物理边界，但表示保持分离：SC 为拟合映射而对曲线
稠密化和 Chang 重采样；SIP 直接使用直线、完整/半圆弧、Bézier 和非有理 B-spline，共 24 个
连续参数段。稠密采样点不会转成 SIP 的折线约束。

正式比较的输入冻结、结果不反向调参规则见
[EXPERIMENT_PROTOCOL.md](EXPERIMENT_PROTOCOL.md)。当前已保留结果是压力场景的历史性失败案例，而不是按该协议冻结后的一次性无偏基准。

生成一次完整候选运行：

```bash
python -m nonconvex_timevarying_window.comparisons.sc_sip_fast_closed_loop.experiment \
  --outdir nonconvex_timevarying_window/comparisons/sc_sip_fast_closed_loop/results/wide_scrambled_curves_v5 \
  --sip-initialization sc_warm_start
```

该压力场景需要多次续跑、批量见证和更大规划裕量。已完成的最终认证结果位于：

```text
results/wide_scrambled_certified_final/
```

可重放认证：

```bash
python -m nonconvex_timevarying_window.sip_dynatogt.verify \
  --run nonconvex_timevarying_window/comparisons/sc_sip_fast_closed_loop/results/wide_scrambled_certified_final/sip_dynatogt/run
```

修订后的最终结论：SC 首先被连续域检查确认违反 15 mm 净距；随后的独立
点—长方体审计进一步找到了原始 Bezier 边界严格位于机体内部的实体相交点。
将 400 次截停的 SC 候选继续优化至收敛后，实体相交仍然存在，且旋翼推力越界。
SIP 以 128-bit Arb、`1,311,838` 个区间单元和最大深度 27 通过整机净距与全部
动力学硬约束认证。该场景是有意设计的压力测试，只证明存在这一失败案例，不代表 SC 的
一般碰撞率。

默认入口保留独立中心初值；正式宽域闭环结果使用显式的
`--sip-initialization sc_warm_start`，对应已批准方法中“SC/MINCO 给出参数化候选，
SIP 负责连续违规发现与安全修复”的流程。`summary.json` 同时记录 SIP 修复阶段耗时
以及计入 SC 初解的端到端耗时，不能把热启动成本隐藏掉。

输出包括：

- `scenario/scenario.json`：所有窗口边界、初始位姿、运动振幅和周期；
- `sc_dynatogt/`、`sip_dynatogt/`：各自的轨迹、结果、三维总览、穿越局部图、缩放图和 GIF；
- `comparison/`：总时间、整机净距和 `contact_timeline.png` 接触时间带对比；
- `summary.json`、`EXPERIMENT_REPORT.md`：机器可读和人类可读结果。

15 mm 净距结论使用 SIP 的 Arb 整机连续域认证器重放两条轨迹；实体相交
另外通过原始曲线点在机体坐标系中严格位于长方体内部来确认。高密度距离曲线
只用于定位和作图，不替代区间证书或实体相交见证。
