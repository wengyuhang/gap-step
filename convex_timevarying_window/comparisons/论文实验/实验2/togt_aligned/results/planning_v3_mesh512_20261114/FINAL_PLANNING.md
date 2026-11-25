# 四方法十档完整规划结果

原TOGT公开七门位置/朝向、QuadA；4个2.4m方门与3个直径2.4m圆门，已知平移/摆动。每侧名义安全内缩0.15m，有限求解数值间隙0.1mm，圆窗解析模型、独立网格512段。只做规划，未执行Gazebo竞速。完整参考从作者起点出发，包含七门及到作者原终点的末段。表中通过指窗框、门序与端点；按用户约定地面相交单列，不算窗框碰撞。

本文自由/中心各8个独立初值、16路并行，无额外中心精修。动态TOGT采用现有C++动态适配、原停止设置、单初值中心后自由流程；不将本变体成绩称为作者原七方门论文成绩。失败与负结果保留。

|幅度比|频率倍率|本文自由点/s|本文中心/s|动态TOGT自由点/s|动态TOGT中心/s|
|---|---|---:|---:|---:|---:|
|0|1|**7.311**|8.068|碰撞|碰撞|
|0.25|1|**7.381**|8.170|碰撞|碰撞|
|0.25|3|**7.269**|8.027|碰撞|碰撞|
|0.25|6|**7.212**|未找到合格规划|碰撞|未完成|
|0.50|1|**7.462**|未找到合格规划|碰撞|碰撞|
|0.50|3|**7.180**|7.964|碰撞|碰撞|
|0.50|6|未找到合格规划|未找到合格规划|碰撞|未完成|
|0.75|1|**7.556**|未找到合格规划|碰撞|碰撞|
|0.75|3|**7.019**|7.887|碰撞|碰撞|
|0.75|6|未找到合格规划|未找到合格规划|碰撞|碰撞|

## 赛道适配对对照结果的影响

作者公开混合七门文件的第7门原是SingleBall，未定义门平面；本轮按方门文件取第7门yaw=200°并设为实体方框，终点沿用作者原位置（与第6门中心重合），所以第7门是折返段。静态动态TOGT在通过第7门后、转向终点时碰框；部分本文候选在折返时重复穿过该开口，被一门仅穿一次的任务检查拒收。因此这些失败不能全部归因于高速窗口运动，也不等同于作者原场景成绩。

通过仅指名义曲线的窗框、门序与端点检查，动力学沿用软惩罚；不是实飞结果或严格区间证书。

## 计算时间

以下为实际求解、建图与独立几何检查墙钟；本文计算八初值并行的等待时间，不累计worker。TOGT自由点包含其中心初始化成本。前两档包括圆网格收敛重查开销，轨迹未重算。共同输入生成没有单独计时，未填零，不作为完整初始化成本已测量的结论。

|条件|方法|计算墙钟/s|合格初值|
|---|---|---:|---|
|static|ours|282.753|8/8|
|static|center|87.128|7/8|
|static|dynamic_togt_center|0.114|0/1|
|static|dynamic_togt|0.220|0/1|
|mixed_base|ours|315.357|8/8|
|mixed_base|center|81.303|2/8|
|mixed_base|dynamic_togt_center|0.118|0/1|
|mixed_base|dynamic_togt|0.249|0/1|
|a25_f3|ours|151.078|8/8|
|a25_f3|center|123.151|8/8|
|a25_f3|dynamic_togt_center|0.121|0/1|
|a25_f3|dynamic_togt|0.262|0/1|
|a25_f6|ours|188.292|3/8|
|a25_f6|center|42.079|0/8|
|a25_f6|dynamic_togt_center|0.140|0/1|
|a25_f6|dynamic_togt|0.300|0/1|
|a50_f1|ours|145.449|8/8|
|a50_f1|center|33.069|0/8|
|a50_f1|dynamic_togt_center|0.117|0/1|
|a50_f1|dynamic_togt|0.227|0/1|
|a50_f3|ours|192.942|6/8|
|a50_f3|center|82.512|8/8|
|a50_f3|dynamic_togt_center|0.120|0/1|
|a50_f3|dynamic_togt|0.285|0/1|
|a50_f6|ours|462.685|0/8|
|a50_f6|center|141.096|0/8|
|a50_f6|dynamic_togt_center|0.117|0/1|
|a50_f6|dynamic_togt|0.238|0/1|
|a75_f1|ours|104.340|8/8|
|a75_f1|center|58.213|0/8|
|a75_f1|dynamic_togt_center|0.114|0/1|
|a75_f1|dynamic_togt|0.218|0/1|
|a75_f3|ours|234.032|7/8|
|a75_f3|center|105.883|6/8|
|a75_f3|dynamic_togt_center|0.140|0/1|
|a75_f3|dynamic_togt|0.246|0/1|
|a75_f6|ours|389.137|0/8|
|a75_f6|center|170.681|0/8|
|a75_f6|dynamic_togt_center|0.116|0/1|
|a75_f6|dynamic_togt|0.230|0/1|

## 未找到合格规划的初值停止原因

|条件|方法|停止原因|初值数|
|---|---|---|---:|
|static|center|no_new_witness / repeat_aperture_crossing|1|
|mixed_base|center|no_new_witness / repeat_aperture_crossing|6|
|a25_f6|ours|no_new_witness / repeat_aperture_crossing|5|
|a25_f6|center|no_new_witness / repeat_aperture_crossing|8|
|a50_f1|center|no_new_witness / repeat_aperture_crossing|8|
|a50_f3|ours|finite_optimizer_not_converged|1|
|a50_f3|ours|no_new_witness / repeat_aperture_crossing|1|
|a50_f6|ours|finite_optimizer_not_converged / repeat_aperture_crossing / unresolved / violated|1|
|a50_f6|ours|no_new_witness / repeat_aperture_crossing|7|
|a50_f6|center|no_new_witness / repeat_aperture_crossing|8|
|a75_f1|center|no_new_witness / repeat_aperture_crossing|8|
|a75_f3|ours|finite_optimizer_not_converged / budget_unresolved / repeat_aperture_crossing / violated|1|
|a75_f3|center|no_new_witness / repeat_aperture_crossing|2|
|a75_f6|ours|no_new_witness / repeat_aperture_crossing|8|
|a75_f6|center|no_new_witness / repeat_aperture_crossing|8|

no_new_witness表示当前候选没有可新增的距离约束见证点；若后面同时标注repeat_aperture_crossing，具体障碍是重复过窗任务检查，不能把它写成IPOPT未收敛或物理无解。逐初值的有限求解状态和失败检查见FINAL_CANDIDATE_AUDIT.json。

名义地面相交记录（不并入窗框碰撞）：static/center；mixed_base/center；a25_f3/center；a50_f3/center；a75_f3/center；

通过条件数：ours 8/10；center 5/10；dynamic_togt 0/10；dynamic_togt_center 0/10；

碰撞为名义曲线与实体窗框的接触；未完成为明确的穿窗顺序/次数或端点任务不合格；证据不足仅用于尚未判定的检查。未找到不等于物理不可行。地面接触另记在assessment中，不并入窗框碰撞。原始SUMMARY/PLANNING保留，分类纠正见REPORT_CLASSIFICATION_CORRECTIONS.json。最终数据FINAL_SUMMARY.json、逐初值审计FINAL_CANDIDATE_AUDIT.json。
