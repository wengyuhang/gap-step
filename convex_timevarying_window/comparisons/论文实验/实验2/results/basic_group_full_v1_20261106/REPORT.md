# 基础组：全局七窗混合动态竞速

状态：完成，12/12条件记录。

固定紧凑基础布局；四个水平平移窗、三个外支点摆动窗，平面固定。各条件原初相一次；所有时间包含起飞执行至七窗整机退出、返航及减速。
赛前一次全局规划，不滚动、不在飞行中重规划。只有无碰撞完赛才填写有效竞速时间；规划未找到与碰撞分别记录。

|运动等级|本文自由点|本文中心|动态TOGT自由点|动态TOGT中心|
|---|---:|---:|---:|---:|
|静态|12.886 s，无碰撞|14.328 s，无碰撞|16.446 s，无碰撞|18.076 s，无碰撞|
|1 Hz／2 m|13.269 s，无碰撞|未找到合格规划|碰撞|碰撞|
|3 Hz／4 m|未找到合格规划|未找到合格规划|碰撞|碰撞|

## 计算与原始证据

|等级|方法|结果|建图 s|四初值求解 s|名义时间 s|MPC最大 ms|来源|
|---|---|---|---:|---:|---:|---:|---|
|static|本文自由点|无碰撞完赛|2.316|97.849|13.280|52.820|[原始记录](original_static_ours/result.json)|
|static|本文中心|无碰撞完赛|2.255|233.057|14.746|39.880|[原始记录](original_static_center/result.json)|
|static|动态TOGT自由点|无碰撞完赛|0.001|2.498|17.066|40.644|[保留原始实飞](../compact_layout_px4_v3_20261106/original_dynamic_togt/result.json)|
|static|动态TOGT中心|无碰撞完赛|0.000|0.412|18.741|41.501|[保留原始实飞](../compact_layout_px4_v3_20261106/original_dynamic_togt_center/result.json)|
|medium|本文自由点|无碰撞完赛|2.481|270.454|13.665|42.113|[原始记录](original_medium_ours/result.json)|
|medium|本文中心|未找到合格规划|2.502|122.614|—|—|[原始记录](plans/original_medium_center/result.json)|
|medium|动态TOGT自由点|碰撞|0.001|14.399|15.872|47.986|[原始记录](original_medium_dynamic_togt/result.json)|
|medium|动态TOGT中心|碰撞|0.001|0.737|18.127|40.509|[原始记录](original_medium_dynamic_togt_center/result.json)|
|high|本文自由点|未找到合格规划|2.483|150.777|—|—|[原始记录](plans/original_high_ours/result.json)|
|high|本文中心|未找到合格规划|2.479|50.959|—|—|[原始记录](plans/original_high_center/result.json)|
|high|动态TOGT自由点|碰撞|0.001|4.949|14.809|37.960|[原始记录](original_high_dynamic_togt/result.json)|
|high|动态TOGT中心|碰撞|0.001|2.182|15.396|61.626|[原始记录](original_high_dynamic_togt_center/result.json)|

静态TOGT两条沿用已验证实飞，核对源差异、场景和证据哈希后保留，不重飞择优。完整冻结设置见protocol.json和PROTOCOL.md；继承来源见reuse_audit.json。
四方法分别沿用原目标/约束与选解规则；本文两方只差选点自由度，TOGT两方只差选点自由度。仅双方成功时比较圈时；一次仿真不估计随机成功概率，不把未找到规划等同物理无解。

![实际闭环轨迹](actual_flights.png)
