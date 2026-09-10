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
