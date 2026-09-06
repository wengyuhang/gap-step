# Gazebo 赛道适配层

此目录将冻结的 `wide_scrambled_fast_closed_loop_6` 可视化为 Gazebo 世界，不修改
`scenario.py`、比较输入或认证语义。它含六扇碰撞启用的 120 mm 管状非凸门框，使用
同一初始中心、姿态和顺序：`W3 → W4 → W1 → W5 → W0 → W2 → finish`。

生成并打开赛道：

```bash
python gazebo/export_world.py
gz-harmonic gazebo/wide_scrambled_fast_closed_loop.sdf
```

启动动态版（另起 Gazebo 服务器、以 20 Hz 驱动六扇门、再打开 GUI）：

```bash
cd gazebo
./run_dynamic_track.sh
```

`motion_bridge.py` 精确复现 `MotionProfile` 的周期平移和完整 RPY 旋转。
Gazebo 8 的标准 `set_pose` 服务没有缩放字段，因此当前适配层不会伪造原赛道的
均匀缩放；要完整复现缩放需要编译一个 Gazebo System 插件来更新每根门框管段的
局部 scale。

这只是高保真可视化与碰撞场景适配；窗口的连续运动和与 PX4 飞控的闭环接入将在
独立运行层完成，不能把此 Gazebo 展示替代原有的 SIP 连续域安全证书。
