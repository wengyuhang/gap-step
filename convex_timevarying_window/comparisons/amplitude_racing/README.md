# 动态窗口幅度实验

按2026-09-19用户要求，区分频率、平移幅度和RPY摆角幅度。详见[冻结协议](PROTOCOL.md)。
本实验只改变场景输入，不修改local_sweep算法，不缩小安全余量，不按门或难度调参。

共16个固定条件：

- 1/10倍频率 × 两种幅度同时1/2/4倍 × 0/120°两个相位，共12次；
- 10倍频率下，仅平移4倍、仅旋转4倍，各两个相位，另4次。

每个世界SDF都是结果目录中的副本。规划器、Gazebo和事后审计都读取同一幅度与频率。
`--translation-scale` / `--rotation-scale` 默认1，不改变历史结果含义。
尚未产生完整汇总时，不能因存在实验脚本就标记为完成。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 conda run --no-capture-output -n wyh \
 python -m convex_timevarying_window.comparisons.amplitude_racing.run --outdir NEW_DIRECTORY
conda run -n wyh python -m convex_timevarying_window.comparisons.amplitude_racing.plot NEW_DIRECTORY
conda run -n wyh python -m convex_timevarying_window.comparisons.amplitude_racing.motion NEW_DIRECTORY
```

不得并发启动其他复用专用实验容器的PX4试飞；所有算法失败留存，基础设施失败最多重试一次。
`motion_metrics.json` 给出窗口中心/开口边界点速度与三维角速度的采样峰值，
不同于单轴RPY导数，也不是连续最大值证书。

## 2026-09-19 完整结果

[16次冻结结果与完整表格](results/frozen_20260919/REPORT.md)：13/16通过物理及实时验收，无基础设施重试。
算法、配置、PX4参数和余量不变；Python3.11.15、NumPy1.26.0、SciPy1.15.3。
1倍频率的1/2/4倍双幅度均2/2；10倍频率对应2/2、2/2、1/2；仅平移4倍和仅摆角4倍均1/2。
最大完整规划周期83.092 ms；成功飞行最小10 ms真值采样净空仅0.035314 m。
4倍幅度对应单轴平移±0.40–0.80 m、RPY摆角±20–40°，不是连续整周旋转。
10倍频率、4倍双幅度的采样中心/开口边界峰速为5.701/16.175 m/s。

三次失败均保留：

- 双幅度4倍、10倍频率相位0：七门返航但真值x达到28.280 m，超过配置上限28 m；不是撞墙证据。
- 仅平移4倍、10倍频率相位0：16.716 s与W4接触，随后与W5接触；首次接触跟踪误差0.456 m，超过0.35 m余量。
- 仅摆角4倍、10倍频率相位120°：21.804 s计划过期中止，0.104 s后与W6接触。终止前采样净空为正不代表整个过程安全；[接触时刻真值审计](results/frozen_20260919/contact_audit.json)给出W6净空−0.198 m。

结论：计算预算在本批次满足，但大幅快速运动下跟踪误差及计划过期后的备用动作仍有安全缺口。
每条件只有两个固定相位，不估计总体成功概率，不宣称连续安全或显著提速。
[运动幅度图](results/frozen_20260919/window_motion.png)、[实际速度](results/frozen_20260919/speed_profiles.png)、[穿越位置](results/frozen_20260919/crossing_positions.png)。
