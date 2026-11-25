# 首窗位置开发探针

保留2026-11-06的六种侧移×三布局、每种中心/自由的全部名义几何记录first_gate_offsets.json。
这些探针使用compact_layout_px4_v1_20261106/source_snapshot中的初始compact_calibration/scenario.py：水平1/4、尚未额外侧移首窗、旧几何margin。
最终选−4m是为了消除非目标阶段重复穿越和接近首窗，不按竞速时间选场景。没有用球形代理替代实际接触或完赛验收。

probe_first_gate.py是当时执行的开发脚本，依赖当时主工作树状态；当前主工作树已换v3，不能直接在当前状态运行并覆盖原JSON。
若需重现，必须在独立输出目录和上述冻结源码环境下执行。当前算法入口是compact_calibration/planner.py及flight.py，不是本脚本。
