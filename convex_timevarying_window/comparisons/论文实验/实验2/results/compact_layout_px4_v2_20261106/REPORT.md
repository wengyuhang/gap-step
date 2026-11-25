# 历史七窗形状：三布局静态校准

状态：运行中，2/6。
这是用户指定的布局校准，不是未调过的泛化评估。窗口形状、尺寸、TOGT内缩规则、原生x500目标、MPC/PX4均固定。
全局七窗后返航，无在线规划；有效时间只填物理无碰撞且真值七窗整机退出、返航完成。

|布局|TOGT自由点|TOGT中心|
|---|---|---|
|original|collision|18.075s，无碰撞|
|slalom|待运行|待运行|
|height|待运行|待运行|

## 原始证据

- original / dynamic_togt_center：[completed_no_contact](original_dynamic_togt_center_facility_retry/result.json)
- original / dynamic_togt：[collision](original_dynamic_togt/result.json)
