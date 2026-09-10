# 七异形闭合赛道 Gazebo Harmonic 场景

该目录独立定义 `L → U → star → limacon → wavy → line_bezier → balanced_U` 闭合赛道，并导出为 Gazebo Harmonic SDF 1.9 世界。窗口形状、尺寸、位置、姿态、初相位和角速度全部写在 `course_spec.py`，不读取规划算法、SC 映射、安全内缩区域或算法实验结果。

## 保真范围

- L、U、星形和 balanced-U 由赛道工程尺寸顶点直接定义；利马松使用 `r(θ)=2.1+0.72cos(θ)`，波浪门使用 `r(θ)=2.15+0.35cos(5θ)`，Line/Bézier 门使用三条直线和两段三次 Bézier 控制点。
- 解析曲线只在生成 Gazebo STL 时按 2 mm 最大弦偏差自适应网格化。这是渲染和刚体碰撞引擎所需的几何离散，不是算法预处理，网格密度也不由任何规划方法决定。
- 碰撞核心由半径 10 mm、12 边截面的封闭管段组成。可选算法复放层中，计入门框半径和 2 mm 网格误差后，正式轨迹仍有 1.878932 mm 的保守最小球体净空。
- 每个窗口增加半径 85 mm 的深色软质门套和高亮内沿。门套中心线向开口外侧偏移 75 mm，因此它的内侧表面与 10 mm 碰撞核心对齐，不缩小原始开口。算法复放世界中门套只渲染；物理赛道世界中门套本身就是碰撞体，做到所见即所撞。
- 算法复放世界使用 1 ms 物理步长、DART、接触系统、阴影、场馆灯和比赛地胶材质。七个窗口分别使用独立 LED 颜色；展示地面位于 `z=-6 m`，低于所有门框旋转扫掠体，不向原赛道添加碰撞障碍。
- 算法复放世界包含正式通过的候选 2701 轨迹、起终点信标和按项目整机尺寸建立碰撞包络的四旋翼模型。物理赛道世界不加载轨迹或无人机。
- 算法复放世界保留闭合地面赛道线和竞速场馆。赛道资产世界改为室内实验场风格：浅灰网格地面、三面灰墙、顶棚发光板和深灰门框。为避免画面杂物，关闭 Gazebo 网格叠层并移除灯光标记和门下支撑杆；室内装饰均为 `visual-only`，不会改变冻结赛道的碰撞几何。
- 默认导出只生成 `seven_unique_physics.sdf` 与 `seven_unique_px4_manual.sdf` 两个赛道世界；它们不包含算法轨迹。`--with-replay` 才另外生成 `seven_unique_high_fidelity.sdf` 和 `seven_unique_race_preview.sdf`，算法结果只是可删除的复放图层。
- 门框运动由 `gz::sim::systems::JointController` 的 `initial_velocity` 执行，不依赖低频外部位姿刷新。

## 生成与检查

```bash
cd nonconvex_timevarying_window/comparisons/seven_unique_dual_constraint_cem/gazebo
conda run -n wyh python export_world.py
conda run -n wyh python validate_course.py
```

`course_manifest.json` 和 `course_validation.json` 只记录赛道定义、网格、材质、门运动与 PX4 世界参数，其中 `algorithm_inputs=[]`。需要重建历史算法复放层时显式执行：

```bash
conda run -n wyh python export_world.py --with-replay
conda run -n wyh python validate_world.py
```

复放层的来源和验收余量单独写在 `manifest.json` 与 `validation.json`，不会进入 PX4 赛道启动链路。

## 启动

需要用 PX4 x500 手动飞行时：

```bash
./run_px4_manual.sh
```

该入口会在同一个 Gazebo Harmonic 世界启动 PX4 SITL，由 PX4 在 `(-16, 4, -6)` 地板位置生成官方 `x500` 模型，然后启动 QGroundControl。没有规划轨迹、轨迹跟踪节点或外部位姿脚本。鼠标在 QGroundControl 的虚拟摇杆上控制飞行，鼠标在 Gazebo 视图中控制镜头。首次使用前执行 `./setup_manual_control.sh`，它会下载官方 Linux AppImage、安装隔离的键盘遥控依赖，并默认打开鼠标虚拟摇杆。

QGroundControl 鼠标操作：进入 Fly 页，先解锁；左杆竖向是油门、横向是偏航，右杆竖向是俯仰、横向是滚转。虚拟油门设置为松手回中，配合 Position 模式试飞。

键盘操作另开一个终端：

```bash
./run_keyboard_teleop.sh
```

`P` 切到 Position，`B` 解锁，`R/F` 增减油门，`W/S` 俯仰，`A/D` 滚转，`Q/E` 偏航，`N` 上锁，`X` 退出并上锁。使用键盘时保持该终端获得焦点。

停止仿真：

```bash
./stop_px4_manual.sh
```

有桌面显示时，直接启动赛道资产版：

```bash
./run_track.sh
```

Gazebo 使用内置 DART 物理引擎。场景只负责把七扇门摆到指定位置，通过原生转动关节和 JointController 设置初相位及恒定角速度；以后载入的无人机由 Gazebo 处理重力、惯性和接触响应。场景没有额外的撞击脚本、外力插件或接触演示模型。

赛道世界不写入自定义 `<gui>`，启动时直接使用 Gazebo Harmonic 自带的标准 GUI、工具栏和交互控制。

只看流畅展示版时使用：

```bash
./run_track.sh gui
```

需要检查算法复放世界时使用：

```bash
./run_track.sh exact-gui
```

无显示服务器时：

```bash
./run_track.sh headless
```

本机通过 `gz-harmonic` 的 Gazebo Harmonic 容器运行。可以用下列命令启动服务器并逐个核对七个关节的实际角速度：

```bash
conda run -n wyh python gazebo_smoke.py
```

流畅展示世界可独立实测：

```bash
conda run -n wyh python gazebo_smoke.py --world seven_unique_race_preview.sdf
```

该世界是几何、碰撞和规定窗口运动的 Gazebo 实现；它不把 Gazebo 数值接触结果升级为连续域安全证书。
