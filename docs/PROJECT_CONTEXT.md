# Project Context

## 当前目标

GAP-Step 当前只训练 PPO 特权教师。环境是连续二维时变窗口迷宫，机器人是双积分圆盘，动作是二维连续加速度。

教师观测是完整图状态：cell、gate、拓扑边、窗口动力学和全局状态。教师不执行 A*/MPC/waypoint，也不使用专家标签。

## 当前训练策略

训练改为逐课程方式：

```text
C1 -> C1_5 -> C2A -> C2B -> C3 -> C4 -> C5
```

每个课程单独训练和保存。后一个课程继承前一个课程的模型参数，但重置优化器。

输出结构：

```text
checkpoints/<课程>/teacher_final.pt
results/<课程>/train_metrics.csv
```

不再保存 `teacher_best.pt`。

## 失败原因判断

上一次完整训练一直停在 C1，说明问题不是复杂动态门本身，而是基础连续迷宫导航没有稳定学会。

主要判断：

- 全局特权信息有用，但不是路径答案；目标相对向量在有墙迷宫中会诱导直冲
- 原 GNN 只靠全图 mean/max pool，agent cell 和 goal cell 信息容易被稀释
- 原训练中 entropy 和 std 持续升高，PPO 后期变成高噪声策略
- C1 中高回报基本对应高成功率，因此当前优先排查 PPO 流程和超参数，而不是继续改奖励函数
- PPO 已改为显式旧策略流程：旧策略负责采样，当前策略负责更新，更新后同步
- progress reward 能提供方向，但撞墙前也可能给正收益，容易鼓励“冲墙式进展”

因此当前先做小修：

- 降低初始动作噪声：`log_std_init=-1.0`，`max_log_std=0.0`
- 降低 `entropy_coef` 到 `0.0001`
- 超时奖励改为 `-20.0`，避免“原地等超时”优于探索
- `reward_progress` 提高到 `4.0`，继续使用动态几何 potential，适用于后续时变窗口课程
- 撞墙/撞门时截断正 progress reward
- GNN 读出显式加入 agent cell 和 goal cell 表示
- 先验证 C1 稳定性，再逐级推进

## 观测约定

```text
GraphObs(
  global_features: [16],
  node_features: [num_nodes, 32],
  node_type: [num_nodes],
  edge_index: [2, num_edges],
  edge_features: [num_edges, 20],
)
```

图观测是特权模拟器状态，但环境动力学、碰撞和成功判定仍是连续几何。
