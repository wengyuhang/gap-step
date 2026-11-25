# 真值日志缺失及只读订阅

`r1_t1_a1_p1_attempt1`完整飞行25.588 s、七门返航、零Gazebo接触，最大完整规划周期71.975 ms。
但11.7 MB的ULog缺少`vehicle_local_position_groundtruth`和`vehicle_attitude_groundtruth`，因此不能完成原协议的独立真值审计。
原始summary.json中的analysis_error和不通过标记保留；报告应将其单列为证据缺失，而不是碰撞，也不能用估计状态补作真值。

确认后，在本批次第五个条件前后启动独立只读`observe.py`，持续订阅专用容器的`/model/x500_0/odometry_with_covariance`，保持Gazebo惰性发布器活跃。
不修改任何传感器值、世界状态、控制输入或飞控参数，真值仍不用于在线控制；源码及算法配置的冻结校验未改变。
后续如补充同条件验证，单独保存，不覆盖缺失日志的原试飞。
