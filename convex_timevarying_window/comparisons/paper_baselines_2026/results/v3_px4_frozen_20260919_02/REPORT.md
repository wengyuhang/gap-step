# V3与两篇旧baseline：同条件完整评估

本次仅运行V3，16条件全部结束；旧两篇结果未改动或重跑。
这是当前独立实现的PX4集成评估，不是作者官方性能比较，也不意味着文档算法已经成功。

V3严格成功 **0/16**；发生门套接触的试飞 **0/16**；失败不填写成功圈时。
2023多项式旧实现严格8/16（另有1次解锁失败）；2026学习方法未通过PX4迁移，不能写成0/16。

|编号|频率|平移幅度|摆角幅度|相位°|V3过门|V3终止原因|V3严格成功|2023严格成功|
|---:|---:|---:|---:|---:|---:|---|---|---|
|0|1|1|1|0|0|timeout|False|True|
|1|1|1|1|120|0|timeout|False|True|
|2|1|2|2|0|0|timeout|False|False|
|3|1|2|2|120|0|timeout|False|True|
|4|1|4|4|0|0|timeout|False|True|
|5|1|4|4|120|0|timeout|False|False|
|6|10|1|1|0|0|timeout|False|True|
|7|10|1|1|120|0|timeout|False|True|
|8|10|2|2|0|0|timeout|False|True|
|9|10|2|2|120|0|timeout|False|False|
|10|10|4|4|0|0|timeout|False|False|
|11|10|4|4|120|0|timeout|False|False|
|12|10|4|1|0|0|timeout|False|False|
|13|10|4|1|120|0|timeout|False|True|
|14|10|1|4|0|0|timeout|False|False|
|15|10|1|4|120|0|timeout|False|False|

## 耗时与证据边界

```json
{
  "cases": 16,
  "strict_successes": 0,
  "contact_cases": 0,
  "failure_counts": {
    "timeout": 16
  },
  "mppi_cycles": 41483,
  "mppi_ms_median": 18.951923004351556,
  "mppi_ms_max": 41.24203196261078,
  "mppi_ms_p99": 26.78203900810331,
  "mppi_over_20ms": 13213,
  "mppi_over_100ms": 0,
  "control_start_interval_ms_median": 24.00000000000091,
  "max_speed_mps": 6.2050241776315795,
  "minimum_truth_sampled_clearance_m": 0.40662080846387916,
  "centre_plane_events": 382,
  "centre_plane_events_outside_aperture": 381,
  "reference_results": 1774,
  "accepted_reference_results": 961,
  "polynomial_strict_successes": 8,
  "rl": "5e8-step independent training completed, PX4 transfer not validated; no formal 16-case score"
}
```

[曲线图](comparison.png)、[门平面穿越位置](crossing_positions.png)。后者是估计中心的插值过平面事件，不等于完整合法出门。
净空来自旧协议的10 ms真值球模型，实际接触单独统计；不是连续安全证书。
40 ms是控制生效前瞻，不是20 ms目标周期；实际启动间隔和超期必须一起读，不能只看中位计算耗时。
PX4直控与旧基线PVA/CTBR的执行层不同，训练信息也不对齐，不能据此作普遍算法优劣结论。
本轮PX4先建立已检查的控制缓存，MPPI采样优先延续缓存；数值开发中的初始TOGT推力种子不是本轮PX4实际基准输入。
中心过平面事件可能包含门平面自身运动；不能视为主动穿窗次数。孔外事件说明存在绕门现象，具体代价/采样原因尚未通过消融验证。
协议见上级V3_PROTOCOL.md，完成核对见本目录AUDIT.md。
没有共同成功的新方法圈速时，不计算‘快了多少’。失败后的位置接管是仿真恢复，不是算法安全后备。
