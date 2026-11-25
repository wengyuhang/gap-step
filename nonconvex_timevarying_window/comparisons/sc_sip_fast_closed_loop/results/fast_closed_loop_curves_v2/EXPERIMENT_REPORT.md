# SC-DynaTOGT vs SIP-DynaTOGT：快速闭环实验

## 赛道

- 起点与终点：`[-8.  -5.5  2.2]`（同一点）
- 窗口顺序：`L_polygon -> circle_arc -> bezier_notch -> bspline_wave`
- 四个窗口均含周期平移、RPY 旋转和均匀缩放，所有周期不超过 4 s。
- SIP 保留 16 个原始连续边界段：`{'Line': 12, 'CircularArc': 1, 'Bezier': 2, 'BSpline': 1}`；SC 稠密点数仅用于映射：`[2048, 2048, 2304, 2347]`。
- SC 安全区采用固定世界裕度修正，要求 `0.315 m`。
- SIP 使用整机长方体和 `0.015 m` 净距离硬约束。

## 结果

| 指标 | SC-DynaTOGT | SIP-DynaTOGT |
|---|---:|---:|
| 总飞行时间 | 5.191251194 s | 5.203511253 s |
| 本阶段求解墙钟时间 | 80.598 s | 324.814 s |
| 计入初值生成的端到端时间 | 80.598 s | 405.412 s |
| 优化器成功 | True | True |
| 全部硬约束状态 | VIOLATED | CERTIFIED_FEASIBLE |
| 整机碰撞状态 | NO_COLLISION_CERTIFIED | NO_COLLISION_CERTIFIED |
| 采样最小整机距离 | 0.080991 m | 0.098326 m |

连续域结论以 Arb 证书为准；采样最小距离只用于定位和绘图。
