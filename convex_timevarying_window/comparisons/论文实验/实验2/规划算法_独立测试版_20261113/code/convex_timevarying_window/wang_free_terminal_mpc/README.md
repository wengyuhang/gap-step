# 局部终点 PVAJ 自由的 MINCO＋MPC

2026-10-07，按用户本轮要求，保留旧 `wang_paper_aligned_mpc` 和复现基线不变。

## 改动与边界

- N=2 局部段、M=4 五次片段：当前状态→下一动态窗→自由局部终态。
- 窗内位置、过窗 V/A、时间、内部点沿用原优化；非终局尾端 P/V/A 都是新优化变量。
- 最后一段的最后一个内部位置与末端进入侧 J 之间存在可逆仿射映射，以 J 替代该控制点。
  因而局部 PVAJ 十二分量均可独立改变，不是把曲线算出的 jerk 改名为自由变量。
  保留五次 MINCO-s3 和段内 C4；窗边界仍 C2，不冒称跨窗 J 连续，也没有新增头部 J 等式。
- 原全局曲线只给局部末态初值；冷初值位置裁至原空间界内，优化中不绑定旧末态。
  同窗重规划继承已接纳曲线末端 PVAJ、时长和内部轨迹；窗口点仍从中心初始化。
- 唯一固定 PVA 的终点是任务规定的最终返航点，不是局部预测终点；其 J 仍自由。
- 不改时间/jerk 目标、罚项、几何/动力学准入、MPC、过窗事件或在线预算。
  不新增终端区域、下一窗吸引项、最低剩余时域或停车段。
  自由尾段可能因时间目标缩短，这是待测现象，不能偷偷用固定末态补回。
- 非终局决策数 37，最终返航 28；J 用旧 23:26 坐标，窗偏置仍 26:28，新尾 PVA 为28:37。

## 本轮最小验证（飞行前声明）

先测末端12维独立性、J重参数化等价性/C4、热启动、解析梯度和原实现回归。
随后原位置固定平面两条件各首次飞一次：0.6Hz/1.2m/0相、1Hz/2m/0相。
不重跑Wang，不覆盖上批六次失败。与对应旧轨迹核对world/MPC/PX4配置一致。
只计真实PX4/Gazebo七窗整机离面并返航的完成时间；失败保留中止时刻与原因。
接触/4ms真值采样、完整规划与MPC<100ms沿用；不是连续域安全认证。
若局部准入或闭环失败，如实报告，不在冻结飞行中调参。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/wang_free_terminal_mpc/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.wang_free_terminal_mpc.run --outdir convex_timevarying_window/wang_free_terminal_mpc/results/new_run
```
