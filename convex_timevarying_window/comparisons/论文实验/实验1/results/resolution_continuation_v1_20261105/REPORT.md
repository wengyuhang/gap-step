# 最高动态档物理步长复核

固定4m/4Hz；每个步长每方法各一次独立闭环运行，原1ms矩阵不替换。
本复核重新在线规划，因此参考曲线与初态并非逐位相同；比较接触结果，不声称完成时间随步长收敛。

|物理步长ms|方法|结果|完整时间s|碰框/碰地|
|---:|---|---|---:|---|
|0.5|ours|completed_no_contact|7.188|False/False|
|0.5|center|completed_no_contact|9.926|False/False|
|0.25|ours|completed_no_contact|6.537|False/False|
|0.25|center|completed_no_contact|11.163|False/False|

0.25ms自由原尝试解锁失败：任务时间0、无规划、无控制；原始记录保留，唯一设施重试完成。
旧resolution_v1目录保留停止状态；本continuation只续跑未飞条件，含原两条0.5ms结果的引用。
四趟均无框/地接触并独立核对真值；这是有限步长物理仿真观察，不是严格连续闭环证书。
