# 单窗摆动：自由点与强制中心

## 2026-10-13 取消规划墙钟上限并继续升档

用户明确取消规划时间限制。当前本场景Config.max_solve_ms=None，在线请求和飞前名义诊断均无墙钟截止；
不以一个更大的有限预算代替。共享方法原有90ms默认值不变；MPC自身60ms执行预算不变。
优化器仍保留70次迭代/轮、8轮见证交换、4个相同时长种子与首个收敛可行解返回策略，
不把“取消时间上限”写成保证收敛或穷尽全局可行解。失败状态为no_converged_safe_passage。
保留40s完整任务截止、原绝对时间引用/150ms规划提前量，窗口始终按外生时间运动。
因此长求解可能使接纳时参考已经开始，需在失败诊断中区分计算延迟与名义轨迹不可行。
继续升档用extension入口：固定4m从4.5Hz起，固定4Hz从4.5m起，各0.5步长；
每档双方各一次，完成配对后遇到任一方法未完赛即停止该路线，再测试另一条路线。
首次失败只描述本批观察，不证明更高档全失败或物理不可行。设施未起飞最多一次重启，原始记录保留。
旧paired_shooting_v1_20261013为90ms预算结果，保留冻结记录；新结果独立，不混作同版本全矩阵。

升档已完成：extension_unlimited_v1_20261013先找到中心失败档；随后
extension_free_boundary_v1_20261013用--start-frequency 5 --start-amplitude 5 --stop-on ours接续，未重飞已有条件。
共9组18趟，自由7成功/2未接纳、中心9未接纳，全部无接触。中心首次4m/4.5Hz与4.5m/4Hz，
自由首次4m/5.5Hz与7m/4Hz，均40s未接纳而非碰撞；不等于物理不可行。
合并报告results/extension_summary_20261013/REPORT.md；最大规划1786ms，MPC61.9ms。
5m/4Hz自由参考接纳晚0.955s，最大跟踪误差1.552m，虽完赛仍暴露长规划与原固定提前量的时序问题；本轮未修补。

## 2026-10-13 跟踪版本更新

当前flight默认使用execution-aligned单重射击MPC（shooting_controller.py），双方共用。
原paired_final_20261012的34趟采用旧acados RTI，已完成且原始结果保留；不得将新跟踪验证混成旧矩阵。
新MPC五条旧失败参考均无碰撞完成，参考系数/时刻、世界和PX4参数相同，未重新规划。
完整验证见../../togt_style_mpc/results/tracking_repair_20261013/REPORT.md。
下文协议中原execution-aligned MPC特指旧34趟；未来新批次将冻结当前单重射击源码。

2026-10-12 用户确认的17条件34趟完整配对实验。原W1小矩形，固定平面，中心水平正弦往复；
不是摆锤圆弧，不增加旋转。起点(-16,4,3.2)，终点位于窗平均位置出口侧6m，终点VAJ为零。
静止基准一次；振幅2/4m分别频率0.5到4Hz、0.5步长；初相零，每条件双方各一次。

两方复用同一hard_v2七变量两段P/V-MINCO，内部A/J消元。目标仅总时间。
中心只固定两个面内位置变量为0；两方都优化到达时刻和三维穿越速度。
取消相位时间箱，保留每段0.3–15s通用时长界。两方统一几何时长1/.8/1.2/1.5倍种子，
所有初始面内点均为中心。相同排序、SLSQP收敛/局部安全准入、90ms预算与5s独立名义诊断。
诊断不作为在线初值；首个收敛且局部检查通过的候选返回，不宣称全局最优。

同硬安全余量、同17节点/段电机与角速度约束，同execution-aligned CTBR MPC/PX4/x500。
首次求解及失败重试；轨迹接纳后完整执行，不启用误差重规划或出窗替换。
完整任务40s，终点位置<0.55m且估计速度<1m/s；等待/求解/刹车均计入，编译单列。
实际真值独立核对正向穿窗一次、整机出窗、终点到达；接触记录决定碰撞。

统一物理步长1ms，窗框接触与新增地面接触传感器1000Hz。接触仅计仿真时刻
[30,30+last_time_s]，排除起飞前地面接触及任务结束后的清理。保留原pbtxt/ULog。
正式飞行前核对两个原生接触话题均有gz.msgs.Contacts发布端，不能用空文件当作传感器正常证据。
v1错误订阅自定义地面话题，整批排除正式结论，保留已有记录；v2编译期发现原窗框传感器仍250Hz，
未飞即停止。最终paired_final统一两个原生接触话题、两传感器1000Hz后完整重跑。
1ms离散物理接触不是连续闭环安全证明；最大窗口速度100.53m/s，每步最多约10cm，
高频档仍需结合接触几何/穿越轨迹审查，零接触不能升级为严格证书。

逐趟记录完整时间/碰框/碰地/未完成/求解耗时。仅双方无碰撞完成时比较时间。
不同频率失败不停止后续档。设施异常且未执行任务最多一次重启，原记录保留；
已飞条件不重跑、不覆盖。设施重试仍失败或审计缺失则停止并诊断，不能强行称整批完成。
代码/插件/世界模板冻结，逐对检查配置只差free_points且世界/MPC/PX4参数一致。
不同实测初始PVAJ完整保留，不能宣称两次物理飞行逐位相同。

运行（wyh；新目录必须不存在）：

```bash
export ACADOS_SOURCE_DIR=/home/jack/wyh/dependencies/acados-togt-v0.5.4
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 \
 /home/jack/anaconda3/envs/wyh/bin/python -u -m \
 convex_timevarying_window.comparisons.single_gate_swing.run --outdir <new-directory>
```

独占convex_seven_togt_experiment，不操作convex_seven_replay。开发静止配对与正式矩阵分开存档。
