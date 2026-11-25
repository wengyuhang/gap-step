# 紧凑交错7窗静态对照

用户要求核对动态中心失败并测试相同赛道静态状态。全部门固定在原名义中心，沿用同一求解器与初始化规则，结果见[REPORT.md](REPORT.md)。

静态中心6/8通过、4.564935443秒，自由7/8通过、4.020986190秒。动态中心已有失败根核对。所有结果为名义规划，当前无Gazebo飞行。

C++生成器static_control_inputs接受原动态slalom输入目录、原classic输入目录和新输出目录。构建在/tmp/slalom_static_control_build_20261116。规划仍复用原fixed_target_run，输入为results/inputs_v1，配置为config，输出为results/paired_v1，原修复版bin目录与fixed_target工具保持同一引用。
