# GAP-Step 二维时变窗口迷宫特权教师训练方案

## 1. 任务

训练对象不是单一迷宫，而是一类程序生成的二维时变窗口迷宫。窗口由墙到墙的线/折线/曲线构成，线体是障碍，中间动态开口可通行。

## 2. 教师

```text
纯 PPO 特权教师
纯 PyTorch GNN
连续二维动作
```

当前不使用：

```text
planner / BC / 专家演示 / waypoint executor / 最终兜底策略
```

## 3. 观测

```text
GraphObs(global_features, node_features, node_type, edge_index, edge_features)
```

图中包含：

- cell node；
- aperture window node；
- cell-cell 边；
- window-cell 边；
- self-loop。

窗口节点编码：

- 当前开口；
- 未来若干相位的开口宽度；
- 未来开口中心变化；
- 是否为当前路径上的下一个窗口。

## 4. 课程

```text
C1      静态短路径
C1_5    静态过渡
C2-C3   少量窗口与渐进路径长度
C3S*    长路径静态桥接
C3_5-C4F 动态窗口桥接
C5      高难度正式分布
```

C5 当前定义：

```text
6 dynamic windows
full-length maze
mixed geometry
gap range 0.72-0.96
unseen seeds
```

## 5. 奖励

- 成功奖励；
- 碰撞终止惩罚；
- 连续几何 progress；
- 窗前动态开口对齐奖励；
- 关闭窗口风险惩罚；
- 靠墙风险惩罚。

窗前 shaping 以动态开口为优先，不再让静态路径中线压过窗口几何。

## 6. 验收

```text
id_test         200 episodes >= 70%
ood_window_test report
ood_maze_test   report
```

当前结果：

```text
id_test         71.5%
ood_window_test 54.0%
ood_maze_test   74.5%
```

## 7. 产物

```text
checkpoints/window_generated/C5/teacher_final.pt
results/window_generated/C5/train_metrics.csv
results/window_generated/eval_c5.csv
results/window_generated/gifs/*.gif
```
