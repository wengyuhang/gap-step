# 原始 Checked 在原七窗固定平面上的验证

2026-10-07，用户要求“用最原始的check方法看下能不能完成”。

方法明确为`continuation_racing.guard.CheckedPlanner`，入口`--checked --tracking-preview .05`。
不是Checked-Soft、MINCO、MPC、MPPI或本轮自由末端PVAJ版。
保留两未来窗+可停车末段、IPOPT65ms/完整100ms、0.2s更新、自适应补点与整个局部检查。
保留原PX4 PVA 100Hz发送/50ms参考预览，原中心穿平面进度逻辑；不悄悄加入新整机事件状态机。
其历史逻辑存在可停车末段，这是本次测试指定的旧方法，不是将停车段放回新方法。

只新增场景接口：统一符号位姿/距离与向量化距离查询，使其符合已固定平面、面内平移的实际世界。
旧规划/发送源码不改，不按门/频率调参。新旧运动函数差异必须经数值/符号测试。

两预声明条件：0.6Hz/单侧1.2m/0相、1Hz/单侧2m/0相，各首次一次、60s上限。
与刚才自由末端两次完全相同的世界、开口、相位和x500/PX4参数；跟踪器故意恢复原PVA，
故比较的是两套完整方案，不是只改规划器的消融。旧结果不覆盖，算法失败不重试。

独立4ms真值审计七窗正向各一次、机体完整离面、不得反穿、房间边界、返航与门框接触。
即使原程序中心事件计数完成，真值不满足就不算成功；无接触不等于安全完成。
末态目标距离<0.55m、速度<1m/s；完整规划与发送周期<100ms。没有外部MPC耗时。
实际名义动态超限单列；采样不是连续安全证书，也不证明任意赛道可行。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run -n wyh python -m pytest -q convex_timevarying_window/comparisons/original_fixed_plane_checked/tests convex_timevarying_window/continuation_racing/tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 GZ_SHOW_GUI=0 conda run --no-capture-output -n wyh python -u -m convex_timevarying_window.comparisons.original_fixed_plane_checked.run --outdir convex_timevarying_window/comparisons/original_fixed_plane_checked/results/new_run
```
