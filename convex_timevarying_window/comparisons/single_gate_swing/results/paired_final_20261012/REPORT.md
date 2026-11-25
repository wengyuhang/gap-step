# 单窗摆动配对实验

原 W1 2.90×2.30 m；固定平面、水平正弦往复、初相零。双方只差窗口内二维选点是否固定为中心。
两方优化时间及穿越速度，同一硬约束、同一 execution-aligned MPC/PX4/x500。
时间为任务开始到窗后固定终点，含等待及刹车。每条件各一次，不估计碰撞概率。

|振幅 m|频率 Hz|自由结果|自由完整 s|中心结果|中心完整 s|自由节省|
|---:|---:|---|---:|---|---:|---:|
| 0 | 静止 | 无碰撞完赛 | 3.383 | 无碰撞完赛 | 3.565 | 5.11% |
| 2 | 0.5 | 漏窗 | — | 未完成 | — | — |
| 2 | 1 | 无碰撞完赛 | 3.861 | 无碰撞完赛 | 3.861 | 0.00% |
| 2 | 1.5 | 碰撞 | — | 无碰撞完赛 | 3.635 | — |
| 2 | 2 | 漏窗 | — | 未完成 | — | — |
| 2 | 2.5 | 漏窗 | — | 无碰撞完赛 | 4.128 | — |
| 2 | 3 | 漏窗 | — | 漏窗 | — | — |
| 2 | 3.5 | 未完成 | — | 未完成 | — | — |
| 2 | 4 | 未完成 | — | 漏窗 | — | — |
| 4 | 0.5 | 未完成 | — | 未完成 | — | — |
| 4 | 1 | 无碰撞完赛 | 4.715 | 无碰撞完赛 | 4.803 | 1.83% |
| 4 | 1.5 | 未完成 | — | 碰撞 | — | — |
| 4 | 2 | 未完成 | — | 无碰撞完赛 | 4.216 | — |
| 4 | 2.5 | 无碰撞完赛 | 4.248 | 漏窗 | — | — |
| 4 | 3 | 无碰撞完赛 | 5.649 | 漏窗 | — | — |
| 4 | 3.5 | 无碰撞完赛 | 5.724 | 漏窗 | — | — |
| 4 | 4 | 碰撞 | — | 漏窗 | — | — |

## 原始记录与计算时间

|振幅|频率|方法|碰框/碰地|接受/请求|规划最大 ms|MPC最大 ms|原始结果|
|---:|---:|---|---|---:|---:|---:|---|
| 0.0 | 1.0 | center | False/False | 1/1 | 20.01 | 21.46 | [a0_f1_center](a0_f1_center/result.json) |
| 0.0 | 1.0 | ours | False/False | 1/1 | 35.15 | 23.16 | [a0_f1_ours](a0_f1_ours/result.json) |
| 2.0 | 0.5 | center | False/False | 1/1 | 26.11 | 22.72 | [a2_f0.5_center](a2_f0.5_center/result.json) |
| 2.0 | 0.5 | ours | False/False | 1/1 | 41.89 | 22.31 | [a2_f0.5_ours](a2_f0.5_ours/result.json) |
| 2.0 | 1.0 | center | False/False | 1/1 | 31.03 | 25.48 | [a2_f1_center](a2_f1_center/result.json) |
| 2.0 | 1.0 | ours | False/False | 1/2 | 92.67 | 22.18 | [a2_f1_ours](a2_f1_ours/result.json) |
| 2.0 | 1.5 | center | False/False | 1/1 | 45.2 | 20.83 | [a2_f1.5_center](a2_f1.5_center/result.json) |
| 2.0 | 1.5 | ours | True/False | 1/1 | 54.4 | 24.7 | [a2_f1.5_ours](a2_f1.5_ours/result.json) |
| 2.0 | 2.0 | center | False/False | 1/1 | 31.94 | 24.35 | [a2_f2_center](a2_f2_center/result.json) |
| 2.0 | 2.0 | ours | False/False | 1/1 | 69.18 | 22.57 | [a2_f2_ours](a2_f2_ours/result.json) |
| 2.0 | 2.5 | center | False/False | 1/1 | 31.46 | 20.74 | [a2_f2.5_center](a2_f2.5_center/result.json) |
| 2.0 | 2.5 | ours | False/False | 1/1 | 47.22 | 21.71 | [a2_f2.5_ours](a2_f2.5_ours/result.json) |
| 2.0 | 3.0 | center | False/False | 1/1 | 27.52 | 23.92 | [a2_f3_center](a2_f3_center/result.json) |
| 2.0 | 3.0 | ours | False/False | 1/1 | 57.5 | 21.6 | [a2_f3_ours](a2_f3_ours/result.json) |
| 2.0 | 3.5 | center | False/False | 1/2 | 91.34 | 19.42 | [a2_f3.5_center](a2_f3.5_center/result.json) |
| 2.0 | 3.5 | ours | False/False | 1/1 | 84.38 | 18.69 | [a2_f3.5_ours_facility_retry](a2_f3.5_ours_facility_retry/result.json) |
| 2.0 | 4.0 | center | False/False | 1/1 | 49.03 | 18.51 | [a2_f4_center](a2_f4_center/result.json) |
| 2.0 | 4.0 | ours | False/False | 1/3 | 91.85 | 19.85 | [a2_f4_ours](a2_f4_ours/result.json) |
| 4.0 | 0.5 | center | False/False | 1/1 | 24.72 | 25.26 | [a4_f0.5_center](a4_f0.5_center/result.json) |
| 4.0 | 0.5 | ours | False/False | 1/1 | 37.93 | 24.22 | [a4_f0.5_ours](a4_f0.5_ours/result.json) |
| 4.0 | 1.0 | center | False/False | 1/1 | 34.97 | 20.87 | [a4_f1_center](a4_f1_center/result.json) |
| 4.0 | 1.0 | ours | False/False | 1/1 | 44.55 | 39.72 | [a4_f1_ours_facility_retry](a4_f1_ours_facility_retry/result.json) |
| 4.0 | 1.5 | center | False/True | 1/1 | 36.73 | 16.74 | [a4_f1.5_center](a4_f1.5_center/result.json) |
| 4.0 | 1.5 | ours | False/False | 1/1 | 43.57 | 39.18 | [a4_f1.5_ours](a4_f1.5_ours/result.json) |
| 4.0 | 2.0 | center | False/False | 1/1 | 21.1 | 18.93 | [a4_f2_center](a4_f2_center/result.json) |
| 4.0 | 2.0 | ours | False/False | 1/1 | 56.62 | 20.59 | [a4_f2_ours](a4_f2_ours/result.json) |
| 4.0 | 2.5 | center | False/False | 1/1 | 34.18 | 18.4 | [a4_f2.5_center](a4_f2.5_center/result.json) |
| 4.0 | 2.5 | ours | False/False | 1/1 | 46.72 | 16.12 | [a4_f2.5_ours](a4_f2.5_ours/result.json) |
| 4.0 | 3.0 | center | False/False | 1/1 | 46.64 | 18.19 | [a4_f3_center](a4_f3_center/result.json) |
| 4.0 | 3.0 | ours | False/False | 1/1 | 73.88 | 17.95 | [a4_f3_ours](a4_f3_ours/result.json) |
| 4.0 | 3.5 | center | False/False | 1/4 | 93.3 | 19.09 | [a4_f3.5_center](a4_f3.5_center/result.json) |
| 4.0 | 3.5 | ours | False/False | 1/1 | 64.34 | 17.31 | [a4_f3.5_ours](a4_f3.5_ours/result.json) |
| 4.0 | 4.0 | center | False/False | 1/15 | 93.57 | 27.4 | [a4_f4_center](a4_f4_center/result.json) |
| 4.0 | 4.0 | ours | False/True | 1/2 | 92.25 | 23.18 | [a4_f4_ours](a4_f4_ours/result.json) |

物理步长统一1 ms；日志按原生频率独立核对。零接触是本次仿真观察，不是闭环连续安全证明。
未完成无有效竞速时间；仅双方无碰撞完成时比较。求解未找到不代表物理无解。
若设施启动失败且未执行任务，最多额外一次启动，保留原记录；已执行条件不重跑、不择优。
