# 完整对比实验结果

执行状态：complete；记录 27/27 条件。

均为独立实现/平台适配，不是作者官方性能。全部原始初相位、每条件一次，无相位扫描或重复。失败中止时刻不是成绩。
初相位此前已用于开发；本轮不是留出相位验证，不推断随机成功率或重复稳定性。

| 方法 | 有效设施条件 | 安全过七窗 | 完整任务通过 | 实际接触 | 成功条件中位时间/s |
|---|---:|---:|---:|---:|---:|
| ours | 9 | 9 | 9 | 0 | 22.072 |
| wang | 9 | 0 | 0 | 7 | None |
| center | 3 | 3 | 3 | 0 | 24.432000000000002 |
| no_soft | 3 | 3 | 3 | 0 | 24.492 |
| no_dense | 3 | 3 | 3 | 0 | 24.324 |

双方共同成功配对数：0；配对时间差中位数：None s。
若无共同成功，不报告速度优势。条件成功中位数不能直接用于整体速度排名。

![安全完成比例](success_heatmap.png)
![安全完成随时间变化](safe_completion_vs_time.png)
![实时计算](online_latency.png)
![消融](ablation_success.png)
![预指定速度案例](representative_speed.png)
![预指定局部穿越位置](representative_crossing_points.png)

summary.json/attempts.json保留逐条件与所有设施重试；protocol.json/source_snapshot为冻结版本。
race_passed只审核七窗竞速前缀；main_passed另含返航。零接触不等于正包络余量，4ms采样不是连续认证。
