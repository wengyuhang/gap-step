# 自由选点：纯跟踪MPC继续升档

算法/控制器冻结，原初相，每条件一次；完整时间含返航。每条路线首次实测窗框碰撞后停止该路线。
超时但未记录接触不是碰撞，继续下一档；无碰撞不自动等于完成。
本批最多四轮（三路线最多12趟）；到达测试上限未碰撞，记录未观察到，不声称找到了临界值。

|幅度m|频率Hz|路线|完整时间|结果|首次窗框接触s|误差重规划|
|---:|---:|---|---:|---|---:|---:|
| 2.0 | 2.5 | frequency | — | incomplete | — | 0 |
| 3.5 | 1.0 | amplitude | — | collision | 5.800 | 15 |
| 3.5 | 2.5 | joint | — | collision | 42.416 | 312 |

## 路线状态

- joint: {"state": "observed_collision", "frequency": 2.5, "amplitude": 3.5, "directory": "/home/jack/wyh/时变窗口/convex_timevarying_window/simple_two_gate_soft_only_mpc/results/pure_tracking_raise_v1_20261008/raise_joint_ours_f2.5_a3.5_p000"}
- frequency: {"state": "continue", "tested_frequency": 2.5, "tested_amplitude": 2.0, "outcome": "incomplete"}
- amplitude: {"state": "observed_collision", "frequency": 1.0, "amplitude": 3.5, "directory": "/home/jack/wyh/时变窗口/convex_timevarying_window/simple_two_gate_soft_only_mpc/results/pure_tracking_raise_v1_20261008/raise_amplitude_ours_f1_a3.5_p000"}

单次固定初相的首次观察，不是成功率、连续安全证明或单调碰撞阈值。无窗框接触不代表全环境无碰撞。
