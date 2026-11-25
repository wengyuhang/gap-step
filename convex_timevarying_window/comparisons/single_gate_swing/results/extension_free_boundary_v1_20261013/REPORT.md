# 单窗摆动配对实验

原 W1 2.90×2.30 m；固定平面、水平正弦往复、初相零。双方只差窗口内二维选点是否固定为中心。
两方优化时间及穿越速度，同一硬约束、同一 execution-aligned MPC/PX4/x500。
时间为任务开始到窗后固定终点，含等待及刹车。每条件各一次，不估计碰撞概率。

|振幅 m|频率 Hz|自由结果|自由完整 s|中心结果|中心完整 s|自由节省|
|---:|---:|---|---:|---|---:|---:|
| 4 | 5 | 无碰撞完赛 | 10.092 | 任务截止前未接纳初始解 | — | — |
| 4 | 5.5 | 任务截止前未接纳初始解 | — | 任务截止前未接纳初始解 | — | — |
| 5 | 4 | 无碰撞完赛 | 9.364 | 任务截止前未接纳初始解 | — | — |
| 5.5 | 4 | 无碰撞完赛 | 8.718 | 任务截止前未接纳初始解 | — | — |
| 6 | 4 | 无碰撞完赛 | 9.387 | 任务截止前未接纳初始解 | — | — |
| 6.5 | 4 | 无碰撞完赛 | 15.784 | 任务截止前未接纳初始解 | — | — |
| 7 | 4 | 任务截止前未接纳初始解 | — | 任务截止前未接纳初始解 | — | — |

## 原始记录与计算时间

|振幅|频率|方法|碰框/碰地|接受/请求|规划最大 ms|MPC最大 ms|原始结果|
|---:|---:|---|---|---:|---:|---:|---|
| 4.0 | 5.0 | center | False/False | 0/56 | 1029.11 | 60.47 | [a4_f5_center](a4_f5_center/result.json) |
| 4.0 | 5.0 | ours | False/False | 1/2 | 439.99 | 39.75 | [a4_f5_ours](a4_f5_ours/result.json) |
| 4.0 | 5.5 | center | False/False | 0/55 | 1129.58 | 60.38 | [a4_f5.5_center](a4_f5.5_center/result.json) |
| 4.0 | 5.5 | ours | False/False | 0/42 | 1468.6 | 61.22 | [a4_f5.5_ours](a4_f5.5_ours/result.json) |
| 5.0 | 4.0 | center | False/False | 0/52 | 1786.02 | 61.87 | [a5_f4_center](a5_f4_center/result.json) |
| 5.0 | 4.0 | ours | False/False | 1/3 | 1105.6 | 46.78 | [a5_f4_ours](a5_f4_ours/result.json) |
| 5.5 | 4.0 | center | False/False | 0/55 | 984.2 | 60.33 | [a5.5_f4_center](a5.5_f4_center/result.json) |
| 5.5 | 4.0 | ours | False/False | 1/1 | 178.75 | 39.47 | [a5.5_f4_ours](a5.5_f4_ours/result.json) |
| 6.0 | 4.0 | center | False/False | 0/56 | 1403.96 | 61.22 | [a6_f4_center](a6_f4_center/result.json) |
| 6.0 | 4.0 | ours | False/False | 1/1 | 253.96 | 42.0 | [a6_f4_ours](a6_f4_ours/result.json) |
| 6.5 | 4.0 | center | False/False | 0/56 | 1340.33 | 60.76 | [a6.5_f4_center](a6.5_f4_center/result.json) |
| 6.5 | 4.0 | ours | False/False | 1/8 | 1246.99 | 43.49 | [a6.5_f4_ours](a6.5_f4_ours/result.json) |
| 7.0 | 4.0 | center | False/False | 0/55 | 1076.65 | 61.03 | [a7_f4_center](a7_f4_center/result.json) |
| 7.0 | 4.0 | ours | False/False | 0/44 | 1597.01 | 60.74 | [a7_f4_ours](a7_f4_ours/result.json) |

物理步长统一1 ms；日志按原生频率独立核对。零接触是本次仿真观察，不是闭环连续安全证明。
未完成无有效竞速时间；仅双方无碰撞完成时比较。求解未找到不代表物理无解。
若设施启动失败且未执行任务，最多额外一次启动，保留原记录；已执行条件不重跑、不择优。
