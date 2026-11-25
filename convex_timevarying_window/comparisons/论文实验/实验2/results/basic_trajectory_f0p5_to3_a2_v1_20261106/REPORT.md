# 实验二基础组：纯规划轨迹对比

状态：进行中，24/28项。

时间为七窗后返航至零PVAJ终态的完整参考时长，不是Gazebo实飞时间。
碰撞模型：x500外接球与原实体STL窗框/地面；无额外评估裕量。每段检查所有窗口，含返航。
已按固定判据自适应覆盖连续时间；普通浮点数值证据，不是跟踪/实体接触或严格区间证书。

|振幅|频率|本文自由点|本文中心|动态TOGT自由点|动态TOGT中心|
|---|---|---:|---:|---:|---:|
|静态|—|13.280 s，名义无碰撞|14.746 s，名义无碰撞|17.066 s，名义无碰撞|18.741 s，名义无碰撞|
|2 m|0.5 Hz|未找到合格规划|15.048 s，名义无碰撞|17.445 s，名义碰撞|17.741 s，名义碰撞|
|2 m|1 Hz|13.665 s，名义无碰撞|未找到合格规划|15.872 s，名义碰撞|18.127 s，名义碰撞|
|2 m|1.5 Hz|14.028 s，名义无碰撞|16.109 s，名义无碰撞|17.291 s，名义碰撞|17.574 s，名义碰撞|
|2 m|2 Hz|16.278 s，名义无碰撞|17.717 s，名义无碰撞|14.757 s，名义碰撞|20.095 s，名义碰撞|
|2 m|2.5 Hz|19.267 s，名义无碰撞|未找到合格规划|18.404 s，名义碰撞|18.628 s，名义碰撞|
|2 m|3 Hz|待完成|待完成|待完成|待完成|

## 计算耗时和原始记录

|条件|方法|建图 s|四初值求解 s|独立评估 s|规划来源|原始评估|
|---|---|---:|---:|---:|---|---|
|静态|本文自由点|2.316|97.849|0.116|复用冻结规划|[记录](assessments/static_ours.json)|
|静态|本文中心|2.255|233.057|0.079|复用冻结规划|[记录](assessments/static_center.json)|
|静态|动态TOGT自由点|0.001|2.498|0.108|复用冻结规划|[记录](assessments/static_dynamic_togt.json)|
|静态|动态TOGT中心|0.000|0.412|0.086|复用冻结规划|[记录](assessments/static_dynamic_togt_center.json)|
|2m/0.5Hz|本文自由点|2.684|251.108|0.000|新规划|[记录](assessments/f0p5_a2_ours.json)|
|2m/0.5Hz|本文中心|2.703|151.872|0.111|新规划|[记录](assessments/f0p5_a2_center.json)|
|2m/0.5Hz|动态TOGT自由点|0.000|7.869|0.159|新规划|[记录](assessments/f0p5_a2_dynamic_togt.json)|
|2m/0.5Hz|动态TOGT中心|0.001|2.111|0.103|新规划|[记录](assessments/f0p5_a2_dynamic_togt_center.json)|
|2m/1Hz|本文自由点|2.481|270.454|0.179|复用冻结规划|[记录](assessments/medium_ours.json)|
|2m/1Hz|本文中心|2.502|122.614|0.000|复用冻结规划|[记录](assessments/medium_center.json)|
|2m/1Hz|动态TOGT自由点|0.001|14.399|0.185|复用冻结规划|[记录](assessments/medium_dynamic_togt.json)|
|2m/1Hz|动态TOGT中心|0.001|0.737|0.120|复用冻结规划|[记录](assessments/medium_dynamic_togt_center.json)|
|2m/1.5Hz|本文自由点|2.573|317.744|0.167|新规划|[记录](assessments/f1p5_a2_ours.json)|
|2m/1.5Hz|本文中心|2.576|201.533|0.154|新规划|[记录](assessments/f1p5_a2_center.json)|
|2m/1.5Hz|动态TOGT自由点|0.001|4.554|0.170|新规划|[记录](assessments/f1p5_a2_dynamic_togt.json)|
|2m/1.5Hz|动态TOGT中心|0.001|1.263|0.132|新规划|[记录](assessments/f1p5_a2_dynamic_togt_center.json)|
|2m/2Hz|本文自由点|2.656|203.092|0.176|复用冻结规划|[记录](assessments/f2_a2_ours.json)|
|2m/2Hz|本文中心|2.708|227.980|0.140|复用冻结规划|[记录](assessments/f2_a2_center.json)|
|2m/2Hz|动态TOGT自由点|0.001|4.154|0.105|复用冻结规划|[记录](assessments/f2_a2_dynamic_togt.json)|
|2m/2Hz|动态TOGT中心|0.001|0.668|0.120|复用冻结规划|[记录](assessments/f2_a2_dynamic_togt_center.json)|
|2m/2.5Hz|本文自由点|2.682|208.146|0.152|新规划|[记录](assessments/f2p5_a2_ours.json)|
|2m/2.5Hz|本文中心|2.579|61.028|0.000|新规划|[记录](assessments/f2p5_a2_center.json)|
|2m/2.5Hz|动态TOGT自由点|0.001|4.580|0.114|新规划|[记录](assessments/f2p5_a2_dynamic_togt.json)|
|2m/2.5Hz|动态TOGT中心|0.001|1.730|0.112|新规划|[记录](assessments/f2p5_a2_dynamic_togt_center.json)|

未找到规划≠碰撞；检查待定≠通过。碰撞轨迹的时长保留展示，但不计有效时间优势。
四方法保持旧目标/约束/选解规则。复用规划原求解耗时照录，不作为本轮新的计时重复实验。
历史实飞数据不覆盖；不启动Gazebo/PX4，不操作容器。协议见protocol.json及源PROTOCOL.md。
