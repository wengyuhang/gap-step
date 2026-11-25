# 单窗摆动配对实验

2026-10-13 完成17条件34趟有效任务。自由点与中心各17/17无碰撞完赛，自由点17组均更快。
动态16组逐组节省比例均值10.97%，范围1.21%–32.14%；不是多次重复统计或理论保证。
双方均零碰撞，本批支持时间优势，但没有观察到碰撞数量上的安全优势。
当前控制器为execution-aligned单重射击/L-BFGS-B MPC，两方完全相同，不是旧acados RTI。

原 W1 2.90×2.30 m；固定平面、水平正弦往复、初相零。双方只差窗口内二维选点是否固定为中心。
两方优化时间及穿越速度，同一硬约束、同一 execution-aligned MPC/PX4/x500。
时间为任务开始到窗后固定终点，含等待及刹车。每条件各一次，不估计碰撞概率。

|振幅 m|频率 Hz|自由结果|自由完整 s|中心结果|中心完整 s|自由节省|
|---:|---:|---|---:|---|---:|---:|
| 0 | 静止 | 无碰撞完赛 | 3.389 | 无碰撞完赛 | 3.555 | 4.67% |
| 2 | 0.5 | 无碰撞完赛 | 3.359 | 无碰撞完赛 | 3.463 | 3.00% |
| 2 | 1 | 无碰撞完赛 | 3.356 | 无碰撞完赛 | 3.852 | 12.88% |
| 2 | 1.5 | 无碰撞完赛 | 3.659 | 无碰撞完赛 | 3.763 | 2.76% |
| 2 | 2 | 无碰撞完赛 | 3.994 | 无碰撞完赛 | 4.076 | 2.01% |
| 2 | 2.5 | 无碰撞完赛 | 4.046 | 无碰撞完赛 | 4.139 | 2.25% |
| 2 | 3 | 无碰撞完赛 | 5.529 | 无碰撞完赛 | 5.597 | 1.21% |
| 2 | 3.5 | 无碰撞完赛 | 4.049 | 无碰撞完赛 | 4.982 | 18.73% |
| 2 | 4 | 无碰撞完赛 | 5.514 | 无碰撞完赛 | 5.697 | 3.21% |
| 4 | 0.5 | 无碰撞完赛 | 3.403 | 无碰撞完赛 | 3.495 | 2.63% |
| 4 | 1 | 无碰撞完赛 | 4.717 | 无碰撞完赛 | 4.803 | 1.79% |
| 4 | 1.5 | 无碰撞完赛 | 4.271 | 无碰撞完赛 | 5.913 | 27.77% |
| 4 | 2 | 无碰撞完赛 | 4.162 | 无碰撞完赛 | 4.220 | 1.37% |
| 4 | 2.5 | 无碰撞完赛 | 4.248 | 无碰撞完赛 | 4.885 | 13.04% |
| 4 | 3 | 无碰撞完赛 | 5.659 | 无碰撞完赛 | 7.058 | 19.82% |
| 4 | 3.5 | 无碰撞完赛 | 6.019 | 无碰撞完赛 | 8.718 | 30.96% |
| 4 | 4 | 无碰撞完赛 | 7.372 | 无碰撞完赛 | 10.864 | 32.14% |

## 原始记录与计算时间

|振幅|频率|方法|碰框/碰地|接受/请求|规划最大 ms|MPC最大 ms|原始结果|
|---:|---:|---|---|---:|---:|---:|---|
| 0.0 | 1.0 | center | False/False | 1/1 | 20.07 | 39.05 | [a0_f1_center](a0_f1_center/result.json) |
| 0.0 | 1.0 | ours | False/False | 1/1 | 34.47 | 43.38 | [a0_f1_ours](a0_f1_ours/result.json) |
| 2.0 | 0.5 | center | False/False | 1/1 | 23.9 | 43.3 | [a2_f0.5_center](a2_f0.5_center/result.json) |
| 2.0 | 0.5 | ours | False/False | 1/1 | 40.87 | 38.96 | [a2_f0.5_ours_facility_retry](a2_f0.5_ours_facility_retry/result.json) |
| 2.0 | 1.0 | center | False/False | 1/1 | 26.61 | 40.99 | [a2_f1_center](a2_f1_center/result.json) |
| 2.0 | 1.0 | ours | False/False | 1/1 | 41.4 | 41.15 | [a2_f1_ours](a2_f1_ours/result.json) |
| 2.0 | 1.5 | center | False/False | 1/2 | 91.75 | 43.61 | [a2_f1.5_center](a2_f1.5_center/result.json) |
| 2.0 | 1.5 | ours | False/False | 1/2 | 91.13 | 40.55 | [a2_f1.5_ours](a2_f1.5_ours/result.json) |
| 2.0 | 2.0 | center | False/False | 1/1 | 27.08 | 39.45 | [a2_f2_center](a2_f2_center/result.json) |
| 2.0 | 2.0 | ours | False/False | 1/1 | 71.07 | 40.77 | [a2_f2_ours](a2_f2_ours/result.json) |
| 2.0 | 2.5 | center | False/False | 1/1 | 26.31 | 39.14 | [a2_f2.5_center](a2_f2.5_center/result.json) |
| 2.0 | 2.5 | ours | False/False | 1/1 | 49.17 | 42.46 | [a2_f2.5_ours](a2_f2.5_ours/result.json) |
| 2.0 | 3.0 | center | False/False | 1/1 | 26.82 | 41.78 | [a2_f3_center](a2_f3_center/result.json) |
| 2.0 | 3.0 | ours | False/False | 1/1 | 55.11 | 44.99 | [a2_f3_ours_facility_retry](a2_f3_ours_facility_retry/result.json) |
| 2.0 | 3.5 | center | False/False | 1/1 | 41.6 | 40.15 | [a2_f3.5_center](a2_f3.5_center/result.json) |
| 2.0 | 3.5 | ours | False/False | 1/1 | 79.47 | 39.22 | [a2_f3.5_ours](a2_f3.5_ours/result.json) |
| 2.0 | 4.0 | center | False/False | 1/1 | 60.06 | 44.04 | [a2_f4_center](a2_f4_center/result.json) |
| 2.0 | 4.0 | ours | False/False | 1/1 | 70.16 | 42.83 | [a2_f4_ours](a2_f4_ours/result.json) |
| 4.0 | 0.5 | center | False/False | 1/1 | 22.04 | 42.26 | [a4_f0.5_center_facility_retry](a4_f0.5_center_facility_retry/result.json) |
| 4.0 | 0.5 | ours | False/False | 1/1 | 49.44 | 49.46 | [a4_f0.5_ours](a4_f0.5_ours/result.json) |
| 4.0 | 1.0 | center | False/False | 1/1 | 30.99 | 37.49 | [a4_f1_center](a4_f1_center/result.json) |
| 4.0 | 1.0 | ours | False/False | 1/1 | 40.76 | 40.28 | [a4_f1_ours](a4_f1_ours/result.json) |
| 4.0 | 1.5 | center | False/False | 1/2 | 92.78 | 63.18 | [a4_f1.5_center](a4_f1.5_center/result.json) |
| 4.0 | 1.5 | ours | False/False | 1/1 | 41.4 | 46.78 | [a4_f1.5_ours](a4_f1.5_ours/result.json) |
| 4.0 | 2.0 | center | False/False | 1/1 | 26.32 | 54.37 | [a4_f2_center](a4_f2_center/result.json) |
| 4.0 | 2.0 | ours | False/False | 1/1 | 38.16 | 41.18 | [a4_f2_ours](a4_f2_ours/result.json) |
| 4.0 | 2.5 | center | False/False | 1/1 | 35.5 | 43.36 | [a4_f2.5_center](a4_f2.5_center/result.json) |
| 4.0 | 2.5 | ours | False/False | 1/1 | 73.54 | 46.49 | [a4_f2.5_ours](a4_f2.5_ours/result.json) |
| 4.0 | 3.0 | center | False/False | 1/1 | 39.56 | 47.36 | [a4_f3_center_facility_retry](a4_f3_center_facility_retry/result.json) |
| 4.0 | 3.0 | ours | False/False | 1/1 | 80.99 | 43.61 | [a4_f3_ours](a4_f3_ours/result.json) |
| 4.0 | 3.5 | center | False/False | 1/3 | 91.6 | 54.19 | [a4_f3.5_center](a4_f3.5_center/result.json) |
| 4.0 | 3.5 | ours | False/False | 1/6 | 142.24 | 42.44 | [a4_f3.5_ours](a4_f3.5_ours/result.json) |
| 4.0 | 4.0 | center | False/False | 1/4 | 99.9 | 46.9 | [a4_f4_center](a4_f4_center/result.json) |
| 4.0 | 4.0 | ours | False/False | 1/4 | 91.42 | 43.16 | [a4_f4_ours](a4_f4_ours/result.json) |

物理步长统一1 ms；日志按原生频率独立核对。零接触是本次仿真观察，不是闭环连续安全证明。
未完成无有效竞速时间；仅双方无碰撞完成时比较。求解未找到不代表物理无解。
若设施启动失败且未执行任务，最多额外一次启动，保留原记录；已执行条件不重跑、不择优。

## 最终审计与限制

17份pair检查全部通过：除窗口选点自由/固定外，规划参数、MPC配置、PX4参数和world完全一致。
所有生产源码哈希与批次开始时相同；原始ULog及双接触话题发布者检查均存在。
34趟均真值正向通过一次、整机退出、到达终点，任务区间窗框/地面接触计数均0。
终点沿用位置误差<0.55m且PX4估计速度<1m/s，不是精确零PVAJ，也不是要求真值速度严格<1m/s。

MPC最大63.185ms，超过100ms次数0；规划最大142.237ms，超过100ms次数2。
两次均为4m/3.5Hz自由点未接纳请求，分别136.329/142.237ms；90ms预算没有严格截断内部求解。
因此不能声称严格满足每次规划100ms。等待/失败请求/终点刹车都已计入完整任务时间。
例如4m/4Hz双方均请求4次，自由轨迹made_at=1.035s、名义时长6.917s，中心为1.034s、10.570s。
完成时刻按原终点容差判定，可以早于名义终点时刻。34趟接纳轨迹后的真值最大位置误差0.372m。

共4条未起飞设施异常，每条件只重启一次，原始记录全部保留，不算穿窗算法失败：

- a2_f0.5_ours、a2_f3_ours：准备阶段电机指令非零但Gazebo机体真值完全静止，任务开始仍在地面，全程位移0，无接受轨迹。执行链根因未完全定位。
- a4_f0.5_center、a4_f3_center：解锁失败，无任务飞行。

各目录facility_retry_reason.json保留诊断依据；有效重启记录目录带facility_retry。
恢复协调器保留在上级results/resume_shooting_20261013.py；没有重跑已正常起飞的条件，也没有择优取值。
状态文件的excluded_unflown_directory是首次诊断记录；全部4条完整清单见analysis.json的excluded_unflown。

允许优化到达时刻意味着可选择低速摆动相位。例如2m/4Hz自由点实际过面窗速2.886m/s，
远低于该运动最大速度50.265m/s。不能把频率/振幅直接当成穿越时实际窗速；原始audit保留窗速和相对横速。
本批每条件原初相一次，不估计碰撞概率，不证明任意初态/相位均成功。

![完整任务时间](paired_task_time.png)

逐趟附加跟踪误差、求解耗时分位数与设施排除依据见[analysis.json](analysis.json)。
原RTI批次paired_final_20261012和固定参考修复验证均保留，不混入本批成绩。
