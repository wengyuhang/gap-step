# 全部选中轨迹的Gazebo实飞结果

2026-11-07。十赛道五方法50项完成登记，49条完整参考各实飞一次，1项原规划未找到。全部49趟从原生ULog、窗位姿和八路接触恢复日志后独立复核通过。

## 完整竞速时间

时间单位秒，包含七窗后返航与减速。计时从30s统一空中起点开始，完成要求正向各穿一次、整机出面、返回起点0.55m内且真值速度低于1m/s、原生窗框及地面零接触。

|幅度/窗宽|频率倍数|本文自由点|本文中心|动态TOGT自由点|动态TOGT中心|Wang独立复现|
|---|---:|---:|---:|---:|---:|---:|
|静态|1|16.706|17.452|20.925|21.766|20.831|
|0.25|1|16.668|17.354|20.940|21.786|20.689|
|0.25|3|16.515|17.195|20.844|21.836|21.046|
|0.25|6|21.828|22.791|21.139|22.212|21.690|
|0.5|1|16.661|17.302|20.840|21.754|20.631|
|0.5|3|18.688|18.764|碰撞|21.914|21.874|
|0.5|6|23.985|36.942|碰撞|碰撞|碰撞|
|0.75|1|16.672|17.265|20.726|21.626|20.749|
|0.75|3|26.305|26.431|碰撞|碰撞|碰撞|
|0.75|6|36.843|未找到规划|碰撞|碰撞|碰撞|

七窗各自基准频率0.30/0.40/0.50/0.60/0.55/0.45/0.35Hz，乘表内倍数。幅度为各窗实际水平开口宽度的比例，四平移三摆动，初相0。

## 完成与碰撞

|方法|无碰撞完赛|实际碰撞|原规划未找到|
|---|---:|---:|---:|
|本文自由点|10|0|0|
|本文中心|9|0|1|
|动态TOGT自由点|6|4|0|
|动态TOGT中心|7|3|0|
|Wang独立复现|7|3|0|

本文与中心共同完赛9组，自由点9组均更快，实测节省0.405%～35.074%。
对动态TOGT自由点的6个共同完赛条件，本文5组更快，0.25/6倍一组更慢。对Wang的7个共同完赛条件，本文6组更快，0.25/6倍一组更慢。逐组时间差保存在paired_comparisons.json。

## 实验范围与执行适配

本轮将已有选中的多项式系数和绝对时长原样整理，统一用x500/PX4与33状态单重射击MPC执行。MPC参考采用规划同一north tilt/heading pi/2和解析PVAJS/角速度/角加速度。Wang原局部理想执行前缀已经拼接为完整参考，本轮检验该冻结参考的闭环可执行性。
同条件五方世界字节、PX4覆盖参数、MPC配置和执行配置逐项一致。20项执行/跟踪回归通过，2460项源码/资源冻结核对通过。49趟均为attempt_1。
本批MPC控制计算最大66.155ms，本文自由点十趟真值跟踪最大误差0.146343m。原规划建图/求解成本独立保存在来源结果中。
底层基类保留部分历史method/checked_soft/continuation字段。当前方法身份以experiment_variant/reference_manifest为准，当前执行语义以planning_scheduler、solves和本协议为准，所有实际规划请求为0。
名义包围球判断与Gazebo实体接触分别保存。当前Gazebo x500包含机架/起落架和四片旋转桨叶共9个碰撞体，SDF和SHA保存在vehicle_model；实际跟踪运动也与名义曲线存在小幅偏差。

## 失败与证据

10趟存在原生实体接触，全部按碰撞登记。其后的超时、漏窗中止或MPC数值失败仍保留在原始记录中。首次接触时刻、对象及后续控制错误见collision_summary.json。
全部原生pbtxt/ULog经原文和压缩文件SHA双重校验无损保存，最终逐趟恢复并重算通过。
原生日志原文共12.077GiB，压缩文件1.363GiB。原始轨迹、控制输入、真值路程、窗位姿与接触记录均保留。

## 文件入口

- [完整逐次记录](/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/gazebo_all_selected_v1_20261107/REPORT.md)
- [50项CSV](/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/gazebo_all_selected_v1_20261107/results.csv)
- [轨迹来源清单](/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/gazebo_all_selected_v1_20261107/catalogue.json)
- [逐组时间差](/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/gazebo_all_selected_v1_20261107/paired_comparisons.json)
- [碰撞汇总](/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/gazebo_all_selected_v1_20261107/collision_summary.json)
- [独立复核](/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/gazebo_all_selected_v1_20261107/validation.json)
- [原始文件SHA](/home/jack/wyh/时变窗口/convex_timevarying_window/comparisons/论文实验/实验2/results/gazebo_all_selected_v1_20261107/record_sha256.json)
