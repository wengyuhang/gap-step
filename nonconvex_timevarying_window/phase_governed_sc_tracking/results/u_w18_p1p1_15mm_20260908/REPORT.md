# U 形旋转窗口：离线碰撞与执行时相位调节

本实验使用同一个零厚度、固定中心/平面、绕法向匀速旋转的均衡 U 形窗口。
窗口角速度为 `18 rad/s`，初相位为 `1.1 rad`，旋转周期为 `0.349065850 s`。
Fixed-WP 和 SC-DynaTOGT 共享两段 degree-7 MINCO 与原始 TOGT 目标；它们的离线目标不包含整段整机碰撞约束。

|method|solver stop|T [s]|crossing [s]|collision samples|collision-free|ordered once|sampled dynamics|
|---|---:|---:|---:|---:|---:|---:|---:|
|Fixed-WP|True|2.494083782|1.294671833|165/2687|False|True|False|
|SC-DynaTOGT|True|2.413053918|1.313053918|171/2615|False|True|False|
|Phase-governed SC tracking|None|2.429053918|1.329053918|0/2644|True|True|False|

执行时调节器从 `0` 到一个旋转周期以 `0.0005 s` 候选间隔预览整机碰撞，选择最早可行等待时间。
选中等待为 `0.016000000 s`，空间路径与 SC 离线轨迹完全相同，穿越时间改变 `0.016000000 s`。
最终最大网格步长为 `0.999611 ms`，碰撞样本从 Fixed-WP 的 `165` 和 SC-DynaTOGT 的 `171` 降为 `0`。

该结果是密集采样数值验收，不是 SIP 连续域证书。相位调节只支持当前固定中心/平面、匀速自旋简化模型；若限定时间内没有安全相位，它会明确失败。
等待段为悬停，后续不重定时，因而离线轨迹原有的动力学超限不会被此方法修复；碰撞与动力学结论需要分开。
