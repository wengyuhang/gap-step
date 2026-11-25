# 实验二独立求解完整对比

状态 complete，已结束 40/40 项。

固定原七窗与运动，静态加3×3幅频矩阵，初相0。本文两方使用同一当前独立求解器，各自四初值。
时间包含七窗后返航至零VAJ，单位秒。表中时间属于名义几何无碰撞完成。†表示额外5ms诊断存在姿态/角速/物理电机问题。

|幅度/窗宽|频率倍数|本文自由点|本文强制中心|动态TOGT自由点|动态TOGT中心|
|---|---:|---:|---:|---:|---:|
|静态|1|17.055|17.810|21.475|22.370|
|0.25|1|17.012|17.718|21.469|22.404|
|0.25|3|16.894|17.553|21.395|22.456|
|0.25|6|22.102|23.102†|21.677|名义碰撞|
|0.5|1|16.993|17.652|21.369|22.375|
|0.5|3|18.974|19.067|名义碰撞|名义碰撞|
|0.5|6|24.270†|37.231|名义碰撞|名义碰撞|
|0.75|1|17.004|17.614|21.235|22.259|
|0.75|3|26.587|26.738|名义碰撞|名义碰撞|
|0.75|6|37.152|未找到合格规划|名义碰撞|名义碰撞|

各窗基准频率0.30/0.40/0.50/0.60/0.55/0.45/0.35Hz。幅度为各窗实际水平开口宽度的比例。

## 节点间动力学与全部成本

|条件|方法|几何结果|5ms角速|5ms电机物理界|90%推力预留界|完整规划 s|来源|
|---|---|---|---|---|---|---:|---|
|static|本文自由点|名义无碰撞|在界内|在界内|超限/待定|60.967|new measurement|
|static|本文中心|名义无碰撞|在界内|在界内|超限/待定|65.728|new measurement|
|static|动态TOGT自由点|名义无碰撞|在界内|在界内|超限/待定|3.152|retained original measurement|
|static|动态TOGT中心|名义无碰撞|在界内|在界内|超限/待定|0.468|retained original measurement|
|mixed_base|本文自由点|名义无碰撞|在界内|在界内|超限/待定|111.013|new measurement|
|mixed_base|本文中心|名义无碰撞|在界内|在界内|超限/待定|69.215|new measurement|
|mixed_base|动态TOGT自由点|名义无碰撞|在界内|在界内|超限/待定|2.455|retained original measurement|
|mixed_base|动态TOGT中心|名义无碰撞|在界内|在界内|超限/待定|0.813|retained original measurement|
|a25_f3|本文自由点|名义无碰撞|在界内|在界内|超限/待定|705.800|retained original measurement|
|a25_f3|本文中心|名义无碰撞|在界内|在界内|超限/待定|938.948|new measurement|
|a25_f3|动态TOGT自由点|名义无碰撞|在界内|在界内|超限/待定|7.027|retained original measurement|
|a25_f3|动态TOGT中心|名义无碰撞|在界内|在界内|超限/待定|1.456|retained original measurement|
|a25_f6|本文自由点|名义无碰撞|在界内|在界内|超限/待定|776.358|retained original measurement|
|a25_f6|本文中心|名义无碰撞|超限/待定|在界内|超限/待定|277.916|new measurement|
|a25_f6|动态TOGT自由点|名义无碰撞|在界内|在界内|超限/待定|8.925|retained original measurement|
|a25_f6|动态TOGT中心|名义碰撞|在界内|在界内|超限/待定|2.457|retained original measurement|
|a50_f1|本文自由点|名义无碰撞|在界内|在界内|超限/待定|124.757|new measurement|
|a50_f1|本文中心|名义无碰撞|在界内|在界内|超限/待定|113.378|new measurement|
|a50_f1|动态TOGT自由点|名义无碰撞|在界内|在界内|超限/待定|4.309|retained original measurement|
|a50_f1|动态TOGT中心|名义无碰撞|在界内|在界内|超限/待定|1.294|retained original measurement|
|a50_f3|本文自由点|名义无碰撞|在界内|在界内|超限/待定|272.773|new measurement|
|a50_f3|本文中心|名义无碰撞|在界内|在界内|超限/待定|332.978|new measurement|
|a50_f3|动态TOGT自由点|名义碰撞|在界内|在界内|超限/待定|7.499|retained original measurement|
|a50_f3|动态TOGT中心|名义碰撞|在界内|在界内|超限/待定|3.792|retained original measurement|
|a50_f6|本文自由点|名义无碰撞|超限/待定|在界内|超限/待定|671.132|retained original measurement|
|a50_f6|本文中心|名义无碰撞|在界内|在界内|超限/待定|579.431|new measurement|
|a50_f6|动态TOGT自由点|名义碰撞|在界内|在界内|超限/待定|7.148|retained original measurement|
|a50_f6|动态TOGT中心|名义碰撞|在界内|在界内|超限/待定|0.755|retained original measurement|
|a75_f1|本文自由点|名义无碰撞|在界内|在界内|超限/待定|251.099|retained original measurement|
|a75_f1|本文中心|名义无碰撞|在界内|在界内|超限/待定|365.177|new measurement|
|a75_f1|动态TOGT自由点|名义无碰撞|在界内|在界内|超限/待定|6.169|retained original measurement|
|a75_f1|动态TOGT中心|名义无碰撞|在界内|在界内|超限/待定|1.868|retained original measurement|
|a75_f3|本文自由点|名义无碰撞|在界内|在界内|超限/待定|504.019|retained original measurement|
|a75_f3|本文中心|名义无碰撞|在界内|在界内|超限/待定|526.606|new measurement|
|a75_f3|动态TOGT自由点|名义碰撞|在界内|在界内|超限/待定|5.451|retained original measurement|
|a75_f3|动态TOGT中心|名义碰撞|在界内|在界内|超限/待定|3.789|retained original measurement|
|a75_f6|本文自由点|名义无碰撞|在界内|在界内|超限/待定|647.482|new measurement|
|a75_f6|本文中心|未找到合格规划|—|—|—|623.152|new measurement|
|a75_f6|动态TOGT自由点|名义碰撞|在界内|在界内|超限/待定|4.715|retained original measurement|
|a75_f6|动态TOGT中心|名义碰撞|在界内|在界内|超限/待定|1.409|retained original measurement|

时间优势仅在双方名义几何完成、姿态诊断有效、5ms角速度在原界内且电机在实际模型界内时计算，见analysis.json。
动力学约束仍为固定节点硬约束，5ms结果属于额外数值诊断。名义几何与动力学诊断分开，完整连续动力学与实飞分别评估。
25条冻结结果保留原始文件哈希及原测量成本，15条新算。全部40条重新几何/姿态/动力学评估。
本文每初值300秒，动态TOGT沿用原180秒预算及原目标择解；所有内部失败与全部成本保留。

![赛道布局](course_layout.png)
