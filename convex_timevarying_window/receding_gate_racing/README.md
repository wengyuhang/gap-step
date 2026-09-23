# 三窗口时空滚动规划

目标是用一条统一规则替代旧实现的门编号特例和离线参考回放。
当前算法在每次更新时，从实测位置、速度、加速度出发，联合优化前方最多三个窗口的
到达时间、窗内二维穿越点、速度和加速度；相邻结点由 C2 五次 Hermite 曲线连接。
每个窗口 9 个变量，三个窗口共 27 个变量。目标为局部飞行时间加小权重 jerk 积分。
这保留了 TOGT 的自由穿越点和时间联合优化思想，但不是原始 MINCO/TOGT 的逐行实现。

## 统一约束与执行

- 所有门使用同一个优化模板，没有门编号分支，没有固定穿越偏移、等门状态机或历史轨迹输入。
- 按物理开口限制穿越点、按路线方向限制过门速度。动态窗口以绝对时间查询，不随飞机重置。
- 默认改用[实际时刻局部穿越约束](LOCAL_SWEEP.md)：在真实多项式的过门前后短区间查询动态平面，
  同时限制离面方向/距离和门框净空，不再用全周期最坏加速度推导最低穿越速度。
  旧公式保留于 `--legacy-crossing`，仅用于复查历史失败；10倍频率时其上下界直接矛盾。
- 名义速度、加速度、jerk、倾角和总推力留量进入短视界采样约束。它们不是完整四旋翼单桨可行性证明。
- 可飞区域作为环境输入传入规划器，独立于窗口编号。PX4 适配器使用场景房间范围；
  原世界墙面只有 visual，因此必须单独约束和事后检查房间边界，不能仅靠门框接触日志验收。
- 按 Gazebo 实际门套的外移中心线与 85 mm 半径建立保守胶囊几何；过门前后额外检查真实多项式。
- 一阶加速度响应模型 `tau * a_dot + a = u` 对应前馈 `u = a + tau * jerk`。
  `tau=0.20 s` 是当前模型假设，不能称为已完成 PX4 系统辨识。
- 每 0.2 s 重规划，IPOPT 单次 CPU 预算 65 ms，完整规划周期超出 100 ms 的候选不准入。
  临近计划过门时刻的最后 0.25 s 保留已有曲线，实测过门后移动预测视界。
- 约束先按物理尺度归一化；可行轨迹与中间搜索迭代分开缓存。更换预测视界时从可行轨迹初始化，
  新视界尚未收敛时允许继续其搜索迭代，但不把失败候选下发给飞控。
- 只对即将执行的 0.3 s 前缀查询全部门框，完整赛道审计放在实验之后。
- PX4 使用异步规划、100 Hz P/V/A 流，规划起始状态包含 0.1 s 计算/传输预测。
  窗口数量改变只改变滚动任务长度，不改变算法参数。

## PX4 时间对齐

本镜像 `SENS_GPS0_DELAY` 默认 110 ms，而 Gazebo GNSS 按当前仿真时刻生成测量。
ULog 中 GPS 的 `timestamp-timestamp_sample` 确实为 110 ms。原设置产生约
`0.12 s × 速度` 的估计位置超前；在一次同场景诊断中，位置 RMSE 为 0.894 m。
将两 GPS 延迟配置为 0 后，另一完整运行降至 0.068 m。这个修正是传感器时间配置，
不是窗口轨迹特调，也不是把真值反馈给规划器。

规划器仍只用 PX4 的 `LOCAL_POSITION_NED`。Gazebo 真值仅在飞行结束后读 ULog 做独立审计，
比较时计入 x500 模型原点到机体的旋转后 0.24 m 偏移。PX4 通用位置/速度增益与限幅
由适配器完整保存，不按门或赛道变化。

## 验收协议

开发中失败全部保留。冻结批次比较原赛道重复试飞、运动相位变体、布局变体、
中心点消融。每组同一参数；不得为各门或各变体调参。结果见 [RESULTS.md](RESULTS.md)。

分别报告圈速、初次可行计划总等待时间、每次在线规划及发送耗时、过门顺序、
真实 Gazebo 接触和采样净空。粗 TOGT 24.660806 s 是名义对照，离线恢复的
28.822504 s 也是名义对照；二者的历史 PX4 跟踪曾碰撞，不能作为安全闭环圈速。
旧特调方案 40.216 s 作为已完成闭环的性能基线。

简化仿真的理想/一阶滞后结果不能替代 PX4。当前采样约束、经验跟踪余量和
保留旧计划的执行逻辑没有递归可行性证明，也没有连续域或真实飞行硬安全证书。
没有可行候选时仍可能任务失败；超限、碰撞和未完成不得计入成功圈速。

## 运行

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run --no-capture-output -n wyh \
  python -m convex_timevarying_window.receding_gate_racing.experiment \
  --lag 0.20 --outdir convex_timevarying_window/receding_gate_racing/results/new_lag_run

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run --no-capture-output -n wyh \
  python -m convex_timevarying_window.receding_gate_racing.px4_experiment \
  --launch --outdir convex_timevarying_window/receding_gate_racing/results/new_px4_run

conda run -n wyh pytest -q convex_timevarying_window/receding_gate_racing/tests

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run --no-capture-output -n wyh \
  python -m convex_timevarying_window.receding_gate_racing.suite --prefix new_frozen_suite
```

需要 `casadi`、`numpy` 和现有 PX4/Gazebo 适配环境。`--launch` 只重建专用的
`convex_seven_togt_experiment` 容器；不会停止其他场景容器。
`--phase 0.8` / `--layout 1.2` 同时改变规划输入和结果目录中的 Gazebo 世界副本；
原始世界、网格和运动函数不被覆盖。`--center` 是同一求解器的固定中心点消融。
`--translation-scale 4 --rotation-scale 4` 将平移/RPY幅度分别放大4倍，不改变周期或开口尺寸；
幅度参数同步进入规划、Gazebo世界副本和两种事后审计，历史结果默认均为1。
2026-09-19新增[幅度分离实验](../comparisons/amplitude_racing/README.md)，与频率扫描分开记录。
`--motion-rate 3` 将平移、旋转周期同时除以 3，幅度和相位不变；它只改变场景输入，
不改变规划器/飞控参数或仿真实时倍率。规划输入、结果目录的 SDF 以及两种事后审计
使用同一倍率，历史未保存倍率的结果按 1 倍处理。
`--race` 是单独保留的较高速度/加速度/jerk 配置探查，不是默认配置，也不能与默认批次混合统计。
`--nested` 是实验性两阶段变体：中心点求解预算 25 ms、自由点精修预算 35 ms，
只在可行候选中优先选择预测时间更短者。原赛道一次 32.444 s，仍慢于中心点对照，
尚无变体赛道验证，因此不作为默认方法或已证实的性能改进。
