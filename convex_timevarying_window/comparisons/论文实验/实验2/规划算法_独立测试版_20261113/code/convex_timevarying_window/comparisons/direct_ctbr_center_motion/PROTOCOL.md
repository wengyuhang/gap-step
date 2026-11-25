# Direct CTBR 与 2026-09-23 中心运动对比

冻结矩阵：动态七窗口，PX4/x500/Gazebo；频率倍率 1/3/6/10，中心平移幅度倍率 1/2/4，旋转幅度 1，相位 0/2π/3，共 24 条件，每条件一次有效运行。飞行上限 60 s，100 ms 单次完整在线周期。PX4 参数采用既有 checked/Wang 比较的同一组数值。

本方法直接输出 CTBR，总推力和机体系角速度。既有两方法使用 PVA 跟踪与 50 ms 预览，这个预览不能等价移植为 CTBR 控制指令延迟；它是控制接口差异，而非赛道或物理参数差异。MPPI 16384 条序列、2 s 时域，算法参数不随条件修改。仅基础设施解锁失败或无结果允许一次重试；算法失败保留。

主验收口径与既有比较一致：顺序七门并返航、零 Gazebo 接触、10 ms 真值采样整机—门套净空非负、工作区通过、每次 MPPI/TOGT 完整在线周期均 <100 ms。另列出更严格的速度、加速度、倾角采样审计，不和主口径混淆。采样审计不等于连续安全证明，24 个预设条件不构成随机成功概率。

旧结果只读：`amplitude_racing/results/center_motion_checked_20260923` 和 `paper_baselines_2026/results/center_motion_wang2023_20260923`。运行 `python -m convex_timevarying_window.comparisons.direct_ctbr_center_motion.run --outdir <新目录>`，中断后可用 `--resume`。每次原始世界、ULog、接触话题、周期日志、真值审计、源码快照/哈希保存在新目录。
