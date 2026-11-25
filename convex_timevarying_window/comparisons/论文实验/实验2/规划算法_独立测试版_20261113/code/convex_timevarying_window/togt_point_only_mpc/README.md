# TOGT 控制点最简变体

与 `single_gate_translation` 的自由点碰撞档独立比较，不改原方法。轨迹由全局
MINCO-s4（每段七次）给出；节点仅有实测起点、每扇移动窗口内的穿越点、终点。
优化变量仅为各航段时长和每扇窗口内二维选点。起点 PVAJ 来自原在线请求，
终点 PVAJ 固定为零；窗口处的 PVAJ 由全局 C6 曲线导出，而非自由变量。
单窗为两个航段、两个七次段、四个标量决策变量。

保持原 `simple_two_gate_soft_only_mpc` 的 x500、MPC、PX4 参数、时间/动力学/
软安全目标与 20 ms 穿窗事件采样；原先每半航段八个规则节点，改为每完整航段
十六个节点以保持密度。与原版相同，记录残差但不以事后轨迹检查否决。
移动窗口的到达时间存在多个局部极小值，因此对相同目标函数采用固定的、
仅由航段长度缩放的七个初值，取优化目标最低的收敛结果；不按本赛道相位、
窗号或碰撞结果指定初值。这个搜索差异意味着与历史方法不是“只换基函数”
的严格消融。
安全结果只按 Gazebo 实体接触和完整到达判断；采样代理不是安全证明。

2026-10-09 后续变体按用户要求去掉穿越点的额外安全内缩，只以物理开口
限制二维穿越点；随时间变化的窗框软安全代价和 20 ms 事件采样保持不变。
该开口几何条件在当前优化器中仍通过惩罚实施，不是硬可行性证书。
此前所有 `f2p5_a3p5_point_only_*` 与 `contact_sweep_s4_v1_20261009` 结果
均来自保留内缩的旧源码快照，不能与新变体混写为同一算法成绩。

相同点式 MINCO 的自由／强制中心同条件配对结果见
`results/f2p5_a3p5_point_only_pair_20261009.md`；两方各一次，均无碰撞，
自由点 9.064 s，中心 11.764 s。

运行单趟原初相 2.5 Hz／3.5 m：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 \
  conda run --no-capture-output -n wyh python -u -m \
  convex_timevarying_window.togt_point_only_mpc.flight --method ours \
  --frequency 2.5 --amplitude 3.5 --phase 0 \
  --outdir convex_timevarying_window/togt_point_only_mpc/results/NEW_RUN
```
