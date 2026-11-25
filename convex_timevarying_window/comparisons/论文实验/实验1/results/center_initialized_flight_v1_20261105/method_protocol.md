# 实验1：中心初始化修改验证

2026-11-05，用户要求将当前自由选点改为先求中心可行解、再继续开放选点。
本目录为新算法的开发验证，不覆盖原34趟或改写旧负结果。

- 新方法：`convex_timevarying_window.center_initialized_minco`；原曲线、时间目标、执行量约束、安全包络与MPC/PX4保持。
- 先用原三条自由点较慢的实际PVAJ/绝对起始时刻做固定请求验证。历史值只作同输入诊断，不充作新飞行。
- 再对2m/2.5Hz、2m/3Hz、4m/1.5Hz三条件，各飞新自由/原中心一次，共6趟，初相0。
- 原小窗、1ms物理/1000Hz框和地接触、同终点和40s截止。时间仍包含首解等待、到终点及减速。
- 自由计入中心+精修两阶段的全部耗时；中心保持原单阶段，明确不是等算力消融。
- 每请求保存本次中心完整轨迹、唯一自由初值、两阶段时间、选择原因；返回名义时间不得超过本次中心。
- 固定输入名义不劣保证不等于两次实飞必快；实际初态/请求时刻不同均保留。
- 碰撞/漏窗/无初解/未完成分别保留，只有双方无碰撞完成才比较完整时间。未飞设施故障最多一次重启。
- 所有源、配置、世界与控制器冻结；已飞条件不择优重飞，不按结果调初值/代价/场景。

```bash
python -m convex_timevarying_window.comparisons.论文实验.实验1.center_initialized.probe --outdir <new-probe-directory>
python -m convex_timevarying_window.comparisons.论文实验.实验1.center_initialized.run --outdir <new-flight-directory>
```
