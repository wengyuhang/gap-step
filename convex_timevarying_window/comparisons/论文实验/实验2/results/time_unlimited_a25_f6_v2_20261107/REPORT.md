# 0.25／6原自由点方法取消墙钟求解上限

状态为complete，已结束4/4个初值。

每初值墙钟预算无上限，算法自身的收敛和迭代终止规则见protocol.json。
时间为七窗后返航完整参考时长，求解耗时单列。

|初值|求解结果|完整参考 s|该初值求解耗时 s|终止状态|
|---:|---|---:|---:|---|
|0|获得合格候选|22.102453|126.399|Optimization terminated successfully|
|1|未找到合格候选|—|31.398|augmented restoration did not reach SQP convergence|
|2|获得合格候选|25.464272|379.197|Optimization terminated successfully|
|3|未找到合格候选|—|311.175|augmented restoration did not reach SQP convergence|

## 完整规划时间比较

|方法|完整参考 s|
|---|---:|
|原300秒/初值自由点|22.102453|
|本轮无墙钟上限自由点|22.102453|
|原动态TOGT自由点|21.676888|

本轮建图2.645秒，四初值求解合计848.176秒。
名义几何结果与节点间动力学诊断分别保存在assessment.json及attitude_audit.json。
