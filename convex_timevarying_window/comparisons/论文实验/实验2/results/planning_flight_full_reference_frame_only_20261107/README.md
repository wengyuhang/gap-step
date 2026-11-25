# 既有记录的完整时长重统计

实飞时间列已撤回并更正，见[更正说明](CORRECTION.md)。后续使用分开的规划表与实飞记录表。

入口build_report.py从原50项规划/实飞记录读取，重算49份ULog中的七窗事件以及完整参考T区间的七路窗框接触。只计窗框碰撞，原地面数据另存；原阈值时间退出本轮排序，时间字段为reference_duration_s。

SUMMARY.md为紧凑合表，REPORT.md为规划/实飞分阶段表及T时刻真值，combined_results.csv/JSON保留50项，flight_reassessment.json保留49条逐窗重计结果。原始数据与冻结源码保持。

在仓库根使用wyh、单线程BLAS重新导出。

```bash
python -m convex_timevarying_window.comparisons.论文实验.实验2.results.planning_flight_full_reference_frame_only_20261107.build_report
```

39条既有七窗无窗碰飞行在原任务结束后已切换位置保持。31条ULog可读取T时刻状态，8条覆盖不足。该限制在摘要、完整表、CSV和验证文件中保存。
