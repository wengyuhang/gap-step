# 实验2：全局七窗混合运动竞速

状态：进行中，5/27条记录。

本文自由点与强制中心均一次联合规划七窗及返航，没有中间停车点、没有滚动视域或飞行中重规划。Wang保留论文局部结构，单独标注为独立复现。
每条赛道含4个平移矩形窗与3个绕外部支点摆动圆窗，初相零；静态/1Hz2m/3Hz4m三档，各一次。
竞速时间为执行开始至七窗后返航及减速。全局赛前建图/求解耗时另表列出。

|赛道|等级|自由结果|自由完整s|中心结果|中心完整s|Wang复现结果|Wang完整s|自由对中心节省|
|---|---|---|---:|---|---:|---|---:|---:|
| original | static | 无碰撞完赛 | 27.706 | 未找到合格规划 | — | 未完成 | — | — |
| original | medium | 待完成 | — | 未找到合格规划 | — | 未完成 | — | — |
| original | high | 待完成 | — | 待完成 | — | 待完成 | — | — |
| slalom | static | 待完成 | — | 待完成 | — | 待完成 | — | — |
| slalom | medium | 待完成 | — | 待完成 | — | 待完成 | — | — |
| slalom | high | 待完成 | — | 待完成 | — | 待完成 | — | — |
| height | static | 待完成 | — | 待完成 | — | 待完成 | — | — |
| height | medium | 待完成 | — | 待完成 | — | 待完成 | — | — |
| height | high | 待完成 | — | 待完成 | — | 待完成 | — | — |

## 规划与执行成本

|赛道/等级|方法|全局四初值求解s|名义轨迹s|在线请求数|在线规划最大ms|MPC最大ms|原始记录|
|---|---|---:|---:|---:|---:|---:|---|
| original/static | ours | 184.777 | 28.226 | 0 | — | 53.981 | [原始结果](original_static_ours/result.json) |
| original/static | center | 180.523 | — | 0 | — | — | [原始结果](plans/original_static_center/result.json) |
| original/static | wang | — | — | 683 | 87.806 | 63.850 | [原始结果](original_static_wang/result.json) |
| original/medium | wang | — | — | 678 | 61.006 | 64.480 | [原始结果](original_medium_wang/result.json) |
| original/medium | center | 59.803 | — | 0 | — | — | [原始结果](plans/original_medium_center/result.json) |

碰撞依据框/地原生接触，未找到规划不等同物理不可行。只在自由/中心双方无碰撞完赛时计算时间差。
每条件各一次不估计随机成功概率；源码、八路接触发布端、真实窗口位姿与ULog独立验收保留。
全局优化不是全局最优证明，名义局部包络和17节点/段动力学不是连续闭环证书。
