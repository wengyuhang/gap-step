# SC/SIP 时变窗口运动速率基准

这是一个预注册的多实例对比：12 个确定性种子乘以 slow、nominal、fast 三个运动速率等级，共 36 个实例。每个实例含一扇圆弧窗口和一扇四段 Bézier 窗口；每一组种子的三档只改变窗口运动周期，原始曲线、中心、运动幅值、相位和缩放范围保持一致。

SIP 固定使用 SC 热启动，因而 SIP 的端到端耗时包含 SC 初解。两种轨迹均以原始曲线和同一姿态相关长方体复核。

实体碰撞与 15 mm 净距违规是不同指标。实体判定使用连续区间分支定界，只有原始边界点被证明严格处于长方体内部时才记为 PHYSICAL_INTERSECTION_CONFIRMED；无法分离接触时记为 INTERSECTION_UNRESOLVED。

运行完整基准：

    python -m nonconvex_timevarying_window.comparisons.sc_sip_motion_rate_benchmark.experiment \
      --outdir nonconvex_timevarying_window/comparisons/sc_sip_motion_rate_benchmark/results/run_YYYYMMDD

快速冒烟（仅一个种子、低证明预算，不可作为正式结果）：

    python -m nonconvex_timevarying_window.comparisons.sc_sip_motion_rate_benchmark.experiment \
      --outdir /tmp/sc_sip_motion_smoke --instances 1 --max-cells 20000 --max-depth 16

输出目录包含：每个实例冻结的 scenario/manifest、SC/SIP 轨迹和证书、实体相交判定、机器可读 summary.json、metrics.csv 以及 aggregate.png。已存在且非空的输出目录会被拒绝覆盖。
