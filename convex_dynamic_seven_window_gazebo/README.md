# 凸动态七窗口 Gazebo Harmonic 赛道

本目录是 `seven_convex_periodic_3d_closed` 当前赛道的独立 Gazebo 资产包。它包含七个凸窗口、物理碰撞门框、与规划场景一致的周期三轴平移和 RPY 旋转，以及旧赛道已经使用的 PX4 SITL 官方 `x500` 手动飞行配置。运行时不读取 `convex_timevarying_window/` 或旧 Gazebo 目录。

## 场景对应关系

- 起终点：`(-16, 4, 3.2)`，闭合路线。
- 窗口顺序：矩形、圆形、五边形、圆形、六边形、圆形、矩形。
- `course_spec.json` 是当前正式实验 `scene.json` 的本地冻结副本；`manifest.json` 保存其 SHA-256。
- 圆形窗口使用 128 段网格表达；多边形使用原始顶点。门框采用与旧赛道一致的深灰软质门套和彩色 LED 内沿：碰撞门套半径 `85 mm`，其中心线向开口外偏移 `65 mm`，所以内表面与半径 `20 mm` 的数学边界核心对齐，不会缩小窗口开口。
- `PeriodicGateMotion` 在 Gazebo `PreUpdate` 中直接读取仿真时间，逐轴执行与 `MotionProfile` 相同的正弦平移相位 `(0, 0.7, 1.4)` 和旋转相位 `(0, 0.9, 1.8)`。因此暂停、重置和改变实时倍率不会造成墙钟漂移。该赛道的缩放本来就是关闭的。
- Gazebo 的 20 mm 有限门框是接触仿真几何；规划验收使用的零厚度边界及外接球语义仍以算法结果为准。

## 生成与验证

```bash
cd /home/jack/wyh/时变窗口/convex_dynamic_seven_window_gazebo
conda run -n wyh python export_world.py
./build_plugin.sh
conda run -n wyh python validate_assets.py
conda run -n wyh python gazebo_smoke.py
```

`build_plugin.sh` 使用本目录的 `Dockerfile.build` 构建插件，不要求宿主机安装 Gazebo 开发包。生成物包括：

- `convex_seven_dynamic_physics.sdf`：1 ms DART 物理赛道；
- `convex_seven_dynamic_px4.sdf`：4 ms PX4 SITL 世界；
- `meshes/`：七个碰撞/显示网格；
- `build/libPeriodicGateMotion.so`：按仿真时钟驱动窗口的 System 插件。

## 启动赛道

```bash
./run_track.sh
```

无桌面服务器时可运行：

```bash
./run_track.sh headless
```

## 启动 PX4 x500

首次使用先准备 QGroundControl 和键盘遥控环境：

```bash
./setup_manual_control.sh
```

然后启动 PX4 SITL、Gazebo GUI 和 QGroundControl：

```bash
./run_px4_manual.sh
```

无人机沿用旧场景配置，在 `(-16, 4, -6)` 地面位置由 PX4 官方桥接器生成 `x500`。键盘遥控另开终端执行 `./run_keyboard_teleop.sh`；停止仿真执行 `./stop_px4_manual.sh`。

该资产完成赛道几何、运动、碰撞和 PX4 仿真接入；下述实验另提供 PX4 Offboard 参考跟踪器。Gazebo 接触结果不解释为连续域安全认证。

## x500 重规划与双轨迹实飞

`x500_togt/` 是独立于原 TOGT 的 x500 解析梯度后端。质量、组合惯量、旋翼位置、力矩系数和单桨最大推力取自 PX4 Gazebo 官方 `x500/model.sdf` 与 `x500_base/model.sdf`。窗口映射的 `margin=0.849078383 m`，即官方旋翼碰撞外包络直径的 1.1 倍；最终安全验收只用真实外包络半径 `0.385944720 m`，不再增加验收余量。

论文 `standard_planning.yaml` 的 `weightVel=0`，所以 x500 物理动力学验收也不把速度作为合格条件。速度峰值仍作为诊断量保存；倾角、机体角速度和旋翼推力参与动力学合格判断。两条冻结轨迹为：

- `trajectories/togt_x500_physical_constraints_100hz.npz`：TOGT baseline，`T=24.660806190 s`，动力学通过、安全失败；
- `trajectories/our_method_x500_accepted_100hz.npz`：Conditional Dual-Constraint CEM 首个硬验收成功候选 364，`T=28.822503627 s`，动力学与名义整机安全均通过。

两条轨迹都以固定 `yaw=0` 在 PX4 SITL 官方 x500 上实际执行。最终对齐运行分别位于 `results/px4_togt/20260910_154055/` 和 `results/px4_togt/20260910_153915/`：两者都在 W2 产生 Gazebo 接触。baseline 的名义安全本来就失败；我们的方法在 W2 有 `0.189239 m` 名义净空，但首次接触时实际位置误差为 `0.342649 m`，所以该接触属于当前 PX4 位置跟踪误差越过规划余量。它不推翻名义轨迹安全验收，也不能作为闭环鲁棒安全结论。

无界面实验入口：

```bash
./run_px4_experiment_headless.sh
conda run --no-capture-output -n wyh python px4_togt_experiment.py \
  --reference trajectories/our_method_x500_accepted_100hz.npz
```

## 100 Hz MPC 跟踪

MPPI 安全跟踪、轨迹过时判据和事件触发 H=3 重规划的实施方案见[设计文档](MPPI_REPLANNING_DESIGN.md)。该文档目前是算法与实验方案，不代表已经完成 MPPI、闭环避碰或连续域安全验证。

`togt_nmpc.py` 实现 TOGT 飞行实验所引用控制器的全状态滚动预测结构：状态为位置、速度、四元数和机体角速度，输入为四桨推力，预测节点 `N=20`、节点间隔 `50 ms`。由于公开 TOGT 包不含 Agilicious 控制器，本实现用 CasADi/IPOPT 做一次实时迭代，并通过 PX4 的总推力/体轴角速度接口执行；速率权重针对 PX4 级联内环从论文的 1 调到 20。它不是原论文 300 Hz INDI 内环的逐行复现。

MPC 求解与 MAVLink 发送分别运行，Gazebo 保持 `real_time_factor=1`。x500 的总推力指令按官方模型逆映射：`omega=sqrt(F/(4k))`、`command=(omega-150)/850`，而不是把物理推力直接除以最大推力。运行命令：

```bash
./run_px4_nmpc_headless.sh
conda run --no-capture-output -n wyh python px4_togt_nmpc_experiment.py \
  --reference trajectories/our_method_x500_accepted_100hz.npz
```

2026-09-11 的正式运行中，新算法轨迹的 MPC/指令频率为 `100.03/100.06 Hz`，平均求解 `5.73 ms`；W1 首次接触前位置 RMS 为 `0.2147 m`，在参考时间 `3.332 s` 接触。TOGT baseline 的对应 RMS 为 `0.2807 m`，在 `3.152 s` 接触；碰撞后 Gazebo 于约 `4.94 s` 停滞，看门狗保存结果并终止。结果分别位于 `results/px4_togt_nmpc/20260911_033329/` 和 `results/px4_togt_nmpc/20260911_033447/`。两条轨迹均未达到 10 cm 跟踪目标，也未通过 Gazebo 碰撞验收；碰撞后的全程 RMS 不作为正常控制精度。

同日新增[在线 C++ 实现](cpp_mpc/README.md)。最终 4 s 运行稳定完成 400 次求解（100 Hz），平均/P95/最大求解时间为 `5.104/7.326/11.577 ms`；W1 接触前位置 RMS 为 `0.2075 m`，参考时间 `3.344 s` 接触。它比 Python 正式运行的调度更稳定，但误差只小幅变化，仍未达到 10 cm。`cpp_mpc/build.sh` 同步生成真实编译参数的 `compile_commands.json`，用于消除编辑器因缺少本地 CasADi/MAVLink include 路径产生的伪报错。

## VT-GP-MPPI 在线 x500 闭环

`online_safe_mppi_px4_experiment.py` 将 `online_safe_mppi/` 的原始门点/时刻规划、1024-rollout 短视界 MPPI、事件重规划和最终一步验证作为外环；每 `50 ms` 从 PX4 `LOCAL_POSITION_NED` 重建状态，再发送一个短的 P/V/A Offboard 指令给 PX4 位置内环。它不是预先生成并回放完整参考轨迹。门状态用 `Gazebo clock - 30 s` 查询，故与世界中门开始运动的时刻严格对齐。

```bash
./run_px4_experiment_headless.sh
conda run --no-capture-output -n wyh python online_safe_mppi_px4_experiment.py \
  --seed 0 --cruise-speed 3.8 --tracking-tube 0.05 \
  --max-flight-time 160 --control-period 0.05
```

2026-09-17 的完整执行结果在 `results/px4_online_safe_mppi/20260917_115103/`：官方 x500 顺序通过 7/7 门、返回终点、七个 Gazebo sleeve-frame contact topic 均为零。实际课时为 `145.268 s`，所以这是一条安全完成基线，**不是竞速圈速结果**。1024 rollout MPPI 平均/P95/最大耗时为 `45.220/51.307/62.908 ms`；完整外环为 `49.411/55.755/69.981 ms`，没有周期超过 `100 ms`。穿越面上的外接球净空为 `0.655–0.850 m`。这是 PX4/Gazebo 实体接触证据，而非连续域安全证书，也不能外推为真机。

## TOGT 同步参考 + 最后窗口相位守门器

`build_robust_togt_racing_reference.py` 对已验收的 `28.822504 s` x500 TOGT 空间路径做时间尺度变换，并用紧支持 C4 核把每个窗口的穿越点同步到新时刻的动态窗口。W1--W6 使用中心；W7 联合保留局部 `y=0.29 m` 穿越偏移，避免“强行过中心”使进近段扫过矩形下边框。冻结竞速参考为 `trajectories/robust_togt_racing_1p25_100hz.npz`，名义时间 `36.028130 s`，速度/加速度峰值 `10.144/6.246 m/s`，x500 解析动力学罚项为零。

PX4 执行时，`px4_togt_experiment.py --final-gate-guard` 仅在 W7 前暂停参考相位：每周期查询一次 W7 位姿，先跟随来流侧 `1.5 m` 的动态对齐点，实测横向误差进入阈值后才以约 `3 m/s` 穿越，离框后恢复 TOGT 返航段。该守门器不在线重算整圈，也不在线做整轨迹安全扫描。

```bash
./run_px4_experiment_headless.sh
conda run --no-capture-output -n wyh python px4_togt_experiment.py \
  --reference trajectories/robust_togt_racing_1p25_100hz.npz \
  --final-gate-guard --guard-lead 2.0
conda run --no-capture-output -n wyh python analyze_guarded_racing_run.py \
  results/px4_togt/20260918_021544
```

正式复现实验 `results/px4_togt/20260918_021544/` 在官方 x500/PX4 SITL 上按顺序合法穿越 7/7 窗口，七个 Gazebo 门框 contact topic 全为零，并以终点误差 `0.549 m`/速度 `0.309 m/s` 满足严格闭环终止条件。实际圈时 `40.216 s`；W7 实际穿越点为 `(-0.064,0.057) m`。在线周期均值/P99/最大值为 `1.125/1.923/2.719 ms`，全部低于 `100 ms`。逐窗外接球采样最小余量中 W4/W6 最紧，分别为 `0.0395/0.0643 m`；这是采样与 Gazebo 接触实证，不是连续域或真机安全证书。

`1.20×` 版实际时间 `37.108 s`，但 W4 有 1 次旋翼接触，按协议失败；`1.25×` 连续两次零接触，其中最后一次使用严格终点计时。因此当前速度边界保留在 `1.20×` 失败与 `1.25×` 成功之间，不把失败圈当成竞速结果。

## 无控制误差的 Gazebo 回放

`px4_kinematic_replay.py` 使用同一官方 x500 SDF 的动态刚体与七扇动态门框，但不启动 PX4 控制器。它在每个 `4 ms` 物理步直接写入冻结轨迹的位姿、线速度和机体角速度；门框 contact sensor 发现首次接触后以 UDP 通知插件永久释放模型，之后由 Gazebo 的重力与刚体碰撞响应决定运动。该实验检验名义几何和 Gazebo 碰撞口径，不能作为 PX4 闭环结论。

```bash
conda run --no-capture-output -n wyh python px4_kinematic_replay.py \
  --reference trajectories/our_method_x500_accepted_100hz.npz
```

2026-09-11 的完整回放中，TOGT x500 baseline (`24.660806 s`) 和 Conditional Dual-Constraint CEM 轨迹 (`28.822504 s`) 都是七门零接触。结果见 `results/px4_kinematic_replay/20260911_045752/` 和 `20260911_045907/`。因此此前两条 PX4 实飞在 W2 的接触不属于名义轨迹误差。规划使用的外接圆只保留为保守搜索代理，最终安全判定统一为这项 Gazebo 实体检查；完整定义见[安全几何口径](GEOMETRY_ALIGNMENT.md)。
