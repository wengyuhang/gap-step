# 完成核对（2026-09-19）

- status.json 为 finished；summary.json 16项，全部结果文件存在，所有子进程正常结束并记录任务超时。
- protocol.json 的25个源文件SHA-256与运行后工作区逐一相同；每项启动前亦由runner检查。
- motion_rate、translation_scale、rotation_scale、phase：16项与旧2023多项式矩阵逐项一致。
- 首项result中的PX4参数与旧首项相比，仅新增MAV_PROTO_VER=2通信设置；飞行控制参数无差异。
- 首项及末项ULog：比赛区间[30,30+last_time_s)内vehicle_control_mode的rates及allocation均为false。
  90.468秒出现的重新启用在超时之后，属于仿真恢复，不计作算法后备控制。
- 20项方法回归通过（0.92秒）；git diff --check通过。
- 速度/穿越位置图已人工查看。中心正向过平面事件382个，其中381个孔外；孔内中心事件不等于整机穿越。
- 所有16项0门、60秒超时、无门框接触，工作区审计通过。没有成功圈时，不作速度提升比例。
- 该证据仅支持本次冻结独立实现的集成评估，不证明PDF理论算法不可能成功，也不证明连续安全。
