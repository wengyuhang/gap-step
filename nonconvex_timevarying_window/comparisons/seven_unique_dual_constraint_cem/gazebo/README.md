# 七异形闭合赛道 Gazebo Harmonic 场景

该目录把正式验收的 `L → U → star → limacon → wavy → line_bezier → balanced_U` 闭合赛道导出为 Gazebo Harmonic SDF 1.9 世界。窗口中心和平面固定，每个门框通过 Gazebo 原生转动关节绕局部法向匀速自旋；初相位和角速度直接读取冻结场景。

## 保真范围

- 每个碰撞网格都沿 `physical_polygon` 原始物理边界生成，没有使用安全内缩多边形、凸包或包围盒替代曲线。
- 曲线窗口保留预处理的全部边界点：利马松 1814 点、波浪 2048 点、Line/Bézier 2304 点。
- 碰撞核心由半径 10 mm、12 边截面的封闭管段组成。正式轨迹相对零厚度边界的最小真实球体余量为 13.878932 mm；计入 Gazebo 门框实体半径后仍余 3.878932 mm。
- 每个窗口增加半径 85 mm 的深色软质门套和高亮内沿。门套中心线向开口外侧偏移 75 mm，因此它的内侧表面与 10 mm 碰撞核心对齐，不缩小原始开口。算法复放世界中门套只渲染；物理赛道世界中门套本身就是碰撞体，做到所见即所撞。
- 算法复放世界使用 1 ms 物理步长、DART、接触系统、阴影、场馆灯和比赛地胶材质。七个窗口分别使用独立 LED 颜色；展示地面位于 `z=-6 m`，低于所有门框旋转扫掠体，不向原赛道添加碰撞障碍。
- 算法复放世界包含正式通过的候选 2701 轨迹、起终点信标和按项目整机尺寸建立碰撞包络的四旋翼模型。物理赛道世界不加载轨迹或无人机。
- 算法复放世界保留闭合地面赛道线和竞速场馆。赛道资产世界改为室内实验场风格：浅灰网格地面、三面灰墙、顶棚发光板和深灰门框。为避免画面杂物，关闭 Gazebo 网格叠层并移除灯光标记和门下支撑杆；室内装饰均为 `visual-only`，不会改变冻结赛道的碰撞几何。
- 导出器生成四个用途明确的世界：`seven_unique_high_fidelity.sdf` 用于算法结果复放，保留 1 ms 和原始 10 mm 碰撞核心；`seven_unique_race_preview.sdf` 是流畅纯展示版；`seven_unique_physics.sdf` 是纯赛道资产版；`seven_unique_px4_manual.sdf` 使用 PX4 默认的 4 ms 步长，并补全仿真 GPS、磁场和大气环境。四个世界的窗口中心、姿态、初相位和角速度相同。
- 门框运动由 `gz::sim::systems::JointController` 的 `initial_velocity` 执行，不依赖低频外部位姿刷新。

## 生成与检查

```bash
cd nonconvex_timevarying_window/comparisons/seven_unique_dual_constraint_cem/gazebo
conda run -n wyh python export_world.py
conda run -n wyh python validate_world.py
```

`manifest.json` 分别记录碰撞核心和视觉门套的网格哈希、地面材质哈希、三角形数量、物理参数和剩余净空；`validation.json` 还检查场馆装饰没有碰撞几何。

## 启动

需要用 PX4 x500 手动飞行时：

```bash
./run_px4_manual.sh
```

该入口会在同一个 Gazebo Harmonic 世界启动 PX4 SITL，由 PX4 在 `(-16, 4, -6)` 地板位置生成官方 `x500` 模型，然后启动 QGroundControl。没有规划轨迹、轨迹跟踪节点或外部位姿脚本。鼠标在 QGroundControl 的虚拟摇杆上控制飞行，鼠标在 Gazebo 视图中控制镜头。

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
