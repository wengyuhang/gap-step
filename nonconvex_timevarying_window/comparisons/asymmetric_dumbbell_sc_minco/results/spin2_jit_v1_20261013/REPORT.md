# 平移叠加自旋：单非凸窗口

平移固定3 Hz / 2 m，初相零；只改变绕原窗口参考原点的法向自旋频率。
当前算法不变：机体内缩分区初值、原PV-MINCO、原约束与局部交换。
**均为离线名义规划，未进行Gazebo飞行，不以本报告宣称实际零碰撞。**

| 自旋/圈每秒 | 转速/rpm | 当前验收 | 名义完整起终点/s | 过窗时刻/s | 求解/ms | 选点区域 |
|---:|---:|---|---:|---:|---:|---|
| 2 | 120 | 通过 | 3.016890 | 1.586070 | 522.793 | right |

## 独立1ms审计（不用于接纳）

| 自旋/圈每秒 | 最小采样净空/m | 单桨上界超限/% | 角速度超限/% |
|---:|---:|---:|---:|
| 2 | 0.002797 | 0.3546 | 0.0000 |

## 每个初值的结果

### 2 圈/秒

离线构建/准备 83301.563 ms；JIT=True。

- 区域0：未接纳，441.434 ms；['Positive directional derivative for linesearch', 'Positive directional derivative for linesearch', 'Positive directional derivative for linesearch', 'Positive directional derivative for linesearch']。
- 区域1：通过，79.275 ms；['Optimization terminated successfully', 'Optimization terminated successfully']。

## 解释范围

- 内缩域在窗口自身坐标系计算，自旋不改变其连通分区；SC始终映射原物理开口。
- 符号姿态、有限边距离、局部复查和独立审计均考虑自旋，不能将姿态冻结。
- 只在法向多项式确定的近窗区间做Bernstein/正弦/余弦乘积包络，仍非Arb证书。
- 动力学只在原节点强制；采样超限如实列出，不称完整连续动力学可行。
- 未接纳不等于碰撞或证明场景不可行；仍可能遗漏时间/速度初值分支。
- 本轮每档一次，不估计成功率；名义时间不是闭环竞速时间。

![初值与最终选点](spin_points.png)
