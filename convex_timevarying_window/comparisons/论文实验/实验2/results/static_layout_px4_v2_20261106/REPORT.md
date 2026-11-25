# 历史七窗形状：三布局静态校准

状态：完成，6/6。
这是用户指定的布局校准，不是未调过的泛化评估。窗口形状、尺寸、TOGT内缩规则、原生x500目标、MPC/PX4均固定。
全局七窗后返航，无在线规划；有效时间只填物理无碰撞且真值七窗整机退出、返航完成。

|布局|TOGT自由点|TOGT中心|
|---|---|---|
|original|11.800s，无碰撞|12.813s，无碰撞|
|slalom|12.301s，无碰撞|13.237s，无碰撞|
|height|12.692s，无碰撞|13.628s，无碰撞|

## 原始证据

- original / dynamic_togt_center：[completed_no_contact](original_dynamic_togt_center/result.json)
- original / dynamic_togt：[completed_no_contact](original_dynamic_togt/result.json)
- slalom / dynamic_togt_center：[completed_no_contact](slalom_dynamic_togt_center/result.json)
- slalom / dynamic_togt：[completed_no_contact](slalom_dynamic_togt/result.json)
- height / dynamic_togt_center：[completed_no_contact](height_dynamic_togt_center/result.json)
- height / dynamic_togt：[completed_no_contact](height_dynamic_togt/result.json)
