# 单左初值、完整动态模型：增广拉格朗日求解诊断

2026-10-14。纠正上一轮纯选点实验没有验证用户完整任务的问题。本目录独立，不改生产算法。

完整复用原 `planner.Planner` 的 SC、2 段 PV-MINCO、自由时间/内部速度、时间目标、
真实动态有限边安全约束、逐桨推力/角速度/工作空间约束、机体/边框/余量、变量边界。
原局部包络复查/补见证保持，起终点 PVAJ 和 3 Hz / 2 m 已知平移保持。
原始版本仍是 17 节点/段动力学，不将“保留完整模型”说成连续时间动力学证书。

仅替换求解策略为标准不等式 PHR 增广拉格朗日，g(x)>=0：

`L = f(x) + (||max(0,lambda-rho*g)||^2-||lambda||^2)/(2*rho)`。

其中 f 为原目标的相同 log1p 标度（最小点不变），lambda 更新为 max(0,lambda-rho*g)。
内层 L-BFGS-B 仍用原解析梯度和变量边界，不加房间目标/固定查询时刻/旧右侧轨迹。
从同一个原始左侧 7 变量初值开始；不先跑无安全或无动力学问题，不分区重启，不圆盘试探。
最终执行原 SLSQP 时间优化抛光与硬残差/原局部几何复查；不因罚函数小就接纳。
中间迭代可不可行，这是约束求解方法，不将硬安全永久替换成固定权重软代价。

默认 rho=1，若互补残差较上一轮未降至1/2则乘10；每轮内层100迭代，至多20外层、30s诊断预算。
此为开发诊断配置，不暗示100ms在线。若失败，保留失败，不因有解而保证本方法必找到。
与原单初值局部法及既有非局部试探法比较，逐行分开，不把最好的多次试验当一次单初值优化。

参考：Birgin/Martínez 的 PHR/Algencan 不等式处理框架，
https://optimization-online.org/wp-content/uploads/2005/03/1085.pdf 。
这是独立简化实现，不称 Algencan 官方实现或沿用其完整理论保证。

## 结果与复现

两种SC坐标×三求解策略共六行均已执行；详见[完整报告](REPORT_20261014.md)。
ALM两种坐标均未可行，生产不切换。原坐标非局部试探通过原有限/几何验收但仍有节点间动力学超限。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /home/jack/anaconda3/envs/wyh/bin/python -m \
 nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.full_constraint_alm.run \
 --outdir <全新结果目录> --normalization original
```

等价左归一化使用 `--normalization left`；不得覆盖旧目录。无Gazebo调用。
