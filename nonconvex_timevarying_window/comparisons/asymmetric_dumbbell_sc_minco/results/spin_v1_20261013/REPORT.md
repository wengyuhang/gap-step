# 平移叠加自旋：单非凸窗口

平移固定3 Hz / 2 m，初相零；只改变绕原窗口参考原点的法向自旋频率。
当前算法不变：机体内缩分区初值、原PV-MINCO、原约束与局部交换。
**均为离线名义规划，未进行Gazebo飞行，不以本报告宣称实际零碰撞。**

| 自旋/圈每秒 | 转速/rpm | 当前验收 | 名义完整起终点/s | 过窗时刻/s | 求解/ms | 选点区域 |
|---:|---:|---|---:|---:|---:|---|
| 0 | 0 | 通过 | 2.782315 | 1.564290 | 1133.175 | right |
| 0.5 | 30 | 通过 | 2.984428 | 1.596629 | 1314.701 | right |
| 1 | 60 | 通过 | 2.683996 | 1.395787 | 1306.537 | right |
| 2 | 120 | 通过 | 3.016890 | 1.586070 | 1985.733 | right |

## 独立1ms审计（不用于接纳）

| 自旋/圈每秒 | 最小采样净空/m | 单桨上界超限/% | 角速度超限/% |
|---:|---:|---:|---:|
| 0 | 0.004376 | 0.4763 | 1.0038 |
| 0.5 | 0.002526 | 1.9766 | 0.0000 |
| 1 | 0.003776 | 0.4137 | 1.8731 |
| 2 | 0.002797 | 0.3546 | 0.0000 |

## 每个初值的结果

### 0 圈/秒

离线构建/准备 1608.851 ms；JIT=False。

- 区域0：未接纳，1014.253 ms；['Positive directional derivative for linesearch', 'Positive directional derivative for linesearch', 'Positive directional derivative for linesearch', 'Positive directional derivative for linesearch']。
- 区域1：通过，116.870 ms；['Optimization terminated successfully']。

### 0.5 圈/秒

离线构建/准备 1584.361 ms；JIT=False。

- 区域0：未接纳，1072.314 ms；['Positive directional derivative for linesearch', 'Positive directional derivative for linesearch', 'Positive directional derivative for linesearch', 'Positive directional derivative for linesearch']。
- 区域1：通过，236.133 ms；['Optimization terminated successfully']。

### 1 圈/秒

离线构建/准备 1577.356 ms；JIT=False。

- 区域0：未接纳，1040.351 ms；['Positive directional derivative for linesearch', 'Positive directional derivative for linesearch', 'Positive directional derivative for linesearch', 'Positive directional derivative for linesearch']。
- 区域1：通过，260.914 ms；['Optimization terminated successfully']。

### 2 圈/秒

离线构建/准备 1618.515 ms；JIT=False。

- 区域0：未接纳，1631.047 ms；['Positive directional derivative for linesearch', 'Positive directional derivative for linesearch', 'Positive directional derivative for linesearch', 'Positive directional derivative for linesearch']。
- 区域1：通过，349.284 ms；['Optimization terminated successfully', 'Optimization terminated successfully']。

## 解释范围

- 内缩域在窗口自身坐标系计算，自旋不改变其连通分区；SC始终映射原物理开口。
- 符号姿态、有限边距离、局部复查和独立审计均考虑自旋，不能将姿态冻结。
- 只在法向多项式确定的近窗区间做Bernstein/正弦/余弦乘积包络，仍非Arb证书。
- 动力学只在原节点强制；采样超限如实列出，不称完整连续动力学可行。
- 未接纳不等于碰撞或证明场景不可行；仍可能遗漏时间/速度初值分支。
- 本轮每档一次，不估计成功率；名义时间不是闭环竞速时间。

![初值与最终选点](spin_points.png)
