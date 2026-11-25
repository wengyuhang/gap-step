# Planner-Privileged Safe Teacher

本方法使用当前通过硬验收的 Conditional Dual-Constraint CEM 轨迹规划结果作为训练期特权信息，训练直接输出归一化总推力和三轴机体角速度（CTBR）的在线教师。它不按绝对时间查询参考位置、速度或姿态，也不在执行时调用轨迹跟踪 MPC。

规划结果只被压缩为七组稀疏信息：窗口顺序、窗口局部穿越点、穿越速度和名义段时间。在线观测由当前完整机体状态、当前/下一窗口状态、局部穿越点和穿越阶段组成。窗口位姿仍按当前仿真时间查询，因为任务本身是时变环境。

控制链为：

```text
当前 x500 状态 + 动态窗口状态 + 稀疏规划特权
  -> 神经网络直接产生 CTBR
  -> 规划恢复动作按训练课程逐步减少占比
  -> 0.3 s 短视界整机避障模块
  -> x500 刚体与受限旋翼动力学
```

`safety_filter.py` 对原网络动作、两级投影动作和规划恢复动作逐条做短视界前向模拟。每条模拟都用真实外接球半径检查全部七个物理门框，并记录预测最小净空和旋翼饱和；选择改动最小的合格动作。若候选均不满足预留净空，则选择预测净空最大的动作并明确标记为 best effort。它是采样预测安全层，不是连续时间证书。

第一轮实验采用行为克隆加 DAgger。纯网络控制和“纯网络+避障”尚未完成任务；网络容易进入远离赛道但无碰撞的状态，说明避障不能替代任务导航。保留 70% 稀疏规划恢复动作、30% 网络动作并加入避障后，首条闭环在 94.08 s 完成七窗闭环并停回起点，零碰撞，最小采样门框净空约 0.089 m。该结果来自 20 ms x500 刚体仿真，尚未接入 Gazebo/PX4。

训练与复验：

```bash
python -m convex_timevarying_window.privileged_safe_teacher.train \
  --outdir convex_timevarying_window/privileged_safe_teacher/results/new_run

python -m convex_timevarying_window.privileged_safe_teacher.evaluate \
  --checkpoint convex_timevarying_window/privileged_safe_teacher/results/new_run/teacher.pt \
  --output convex_timevarying_window/privileged_safe_teacher/results/new_run/evaluation.json

pytest -q convex_timevarying_window/privileged_safe_teacher/tests
```

当前下一阶段是把恢复动作占比按 `0.7 -> 0.5 -> 0.3 -> 0` 课程降低，并在每一级要求完整成功和零碰撞；通过后再接 Gazebo/PX4。任何“安全”结论都必须同时给出模型占比、避障介入率、最小净空、碰撞数和动力学饱和情况。
