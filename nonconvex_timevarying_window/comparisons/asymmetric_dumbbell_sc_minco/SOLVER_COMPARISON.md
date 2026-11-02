# 同一有限非凸问题的局部求解器对照

2026-10-13。用户询问是否是 SLSQP 特有问题；本轮只做离线求解诊断，不切换生产算法。

同 3 Hz / 2 m 左小右大场景、一个左室中心初值、原 SC 坐标、两段 PV-MINCO，
原目标与有限硬约束完全相同。位置、时间和速度共 7 个变量；解析一阶导数。
固定原见证集合，不加入跨区试探、不重启、不补见证、不改变物理限值。
两个独立初值条件：原始左初值；同一次最小二乘恢复后的左初值。

每个求解器上限 500 次迭代、10 s 诊断预算，关闭 JIT。
IPOPT 使用 limited-memory Hessian，trust-constr 使用稀疏约束雅可比和默认拟牛顿近似；
其他具体选项见compare_solvers.py。此轮不是穷尽求解器设置或最佳性能排名。
有限残差 >= -1e-5 才可进入独立局部几何复查，求解器退出标志不替代验收。

| 求解器 | 原始左初值最小残差 | 原始初值求解时间 | 相同恢复后最小残差 | 恢复后求解时间 |
|---|---:|---:|---:|---:|
| SLSQP | -0.083510 | 204.920 ms | -0.058238 | 280.073 ms |
| IPOPT | -0.063099 | 943.188 ms | -0.063099 | 327.249 ms |
| trust-constr | -0.135259 | 6542.555 ms | -0.096696 | 6450.321 ms |

六条均未满足有限约束，无接纳轨迹、飞行时间或碰撞结论。表中残差包含不同约束，不能解释为距离。
恢复后计时不含共同恢复耗时；IPOPT接口建立约12.856/11.148ms，另记原始结果。
SLSQP原始条件报告不等式不相容，恢复后报告线搜索失败；IPOPT两条均报告
Infeasible_Problem_Detected；trust-constr两条均达500次迭代上限，非10s超时。

IPOPT的该状态表示它停在局部不可行点，不能解释为原非凸问题无解：
[官方退出状态说明](https://coin-or.github.io/Ipopt/OUTPUT.html)。
本轮不能证明换求解器永远无效，只能说明这些设置下单纯替换局部求解器没有解决此例。

原始初值的SLSQP函数评估曾到达右室，最后仍回到左室失败；其余五条评估都留在左室。
因此不能声称所有局部求解器都禁止大步跨区，或到过右室就能找到右室可行轨迹。
上一轮非单调试探成功，证明的是那一条具体求解流程有用，不是普适单初值完备性。

原始数据：[solver_comparison_v1_20261013](results/solver_comparison_v1_20261013)。
入口：

```bash
/home/jack/anaconda3/envs/wyh/bin/python -m \
  nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.compare_solvers \
  --outdir nonconvex_timevarying_window/comparisons/asymmetric_dumbbell_sc_minco/results/solver_comparison_v1_20261013
```

IPOPT通过CasADi回调接收原SC解析链式雅可比，新增测试验证该接口正确传递目标和约束导数。
保留全部负结果，不因失败反复调参后择优；没有操作Gazebo或更改共享生产代码。
