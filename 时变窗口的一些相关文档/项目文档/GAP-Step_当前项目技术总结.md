# GAP-Step 当前项目技术总结

## 1. 当前定位

当前项目已重构为 v4.6 版本：连续二维旋转时变窗口迷宫中的 PPO 特权教师训练系统。

当前阶段只做一件事：

```text
程序化连续二维迷宫
    -> 低维特权射线观测
    -> PPO 训练教师策略
    -> ID / OOD-size / OOD-dynamics 评估
```

旧版本的视觉学生、行为克隆、Aux 辅助预测、启发式教师演示、`trainers/` 和 `scripts/` 主流程已经移出当前实现。代码不再兼容旧 E1/E2/E3 固定窗口实验。

## 2. 代码结构

所有当前可运行代码都放在 `gap_step/` 文件夹内：

```text
gap_step/
  env.py          # 连续迷宫环境、碰撞、射线观测、渲染
  curriculum.py   # C1-C5 在线程序化课程生成
  model.py        # PPO 教师 Actor-Critic
  ppo.py          # rollout、GAE、PPO update
  train.py        # 教师训练入口
  evaluate.py     # ID/OOD 评估入口
  visualize.py    # GIF 可视化入口
  utils.py        # 通用工具
  gif.py          # GIF 保存
```

旧目录 `gap_step/envs/`、`gap_step/models/`、`gap_step/teachers/`、`trainers/`、`scripts/` 不再作为代码入口使用。

## 3. 环境与任务

环境是连续方形区域：

```text
Omega_S = [0, S] x [0, S]
```

智能体是圆盘机器人，状态为位置和速度，动作是二维连续加速度：

```text
a_t = [a_x, a_y]
```

动力学为双积分：

```text
v = clip(v + a * dt, -v_max, v_max)
p = p + v * dt
```

迷宫先由 randomized DFS 生成离散网格拓扑，再转换为连续坐标下的横向/竖向墙段。生成器会额外打开少量边形成环路，使 C4/C5 更接近普通二维迷宫，而不是规则墙体阵列。部分通路边被替换成时变窗口槽位。窗口的当前可通行性由宽度和旋转角共同决定：

```text
safe_width = d(t) >= 2 * robot_radius + safe_margin
safe_angle = abs(wrap(theta(t) - theta_ref)) <= theta_safe
gate_safe = safe_width and safe_angle
```

不可通行窗口按障碍处理；可通行窗口在当前几何中表现为开口。

## 4. 特权教师观测

教师观测是固定 39 维低维向量：

```text
self_features + goal_features + ray_features
4 + 3 + 32 = 39
```

其中：

```text
self_features = [x/S, y/S, vx/v_max, vy/v_max]
goal_features = [(goal_x-x)/S, (goal_y-y)/S, norm(goal-pos)/S]
```

`ray_features` 使用固定 `N_ray = 32` 条射线。射线最大距离不再是固定 6.0，而是：

```text
ray_max_dist = 0.35 * S
```

因此在更大尺寸的 OOD 迷宫中，局部感知半径也同步变大，但网络输入维度保持不变。

教师观测不包含：

```text
完整地图
A* 路径
waypoint
窗口数量列表
窗口宽度原始值
窗口角度原始值
未来窗口状态
图像观测
```

## 5. 课程学习

课程由 `gap_step/curriculum.py` 在线生成，不保存训练集。

```text
C1: 小迷宫，1 个固定可通行窗口
C2: 小迷宫，1-2 个开闭窗口，无旋转
C3: 小/中迷宫，加入旋转窗口
C4: 中型普通迷宫，3-5 个时变窗口
C5: 最终普通迷宫，6-10 个异步时变窗口，含横竖墙和少量环路
```

训练尺寸：

```text
S_train = [15, 19, 23]
```

测试包含：

```text
ID:             [15, 19, 23]
OOD-size:       [17, 21, 25, 31]
OOD-dynamics:   [17, 25, 31] + 训练外窗口动力学参数
```

## 6. PPO 教师

教师网络是 MLP Gaussian Actor-Critic：

```text
obs_dim=39 -> 256 -> 256 -> actor_mean(2)
obs_dim=39 -> 256 -> 256 -> value(1)
log_std 为可学习参数
```

执行动作前裁剪到：

```text
[-a_max, a_max]^2
```

环境默认的 strict v4.6 稀疏奖励为：

```text
reward = +20 if goal
       + -20 if collision
       - 0.01
       - 0.001 * ||action||^2
```

当前训练配置额外启用连续几何动态进展 shaping：

```text
progress_reward = reward_progress * (prev_remaining_time - current_remaining_time)
reward_timeout = -5.0 on timeout
```

`remaining_time` 由连续几何 visibility roadmap 估计，不使用 `cell/open_edges` 作为奖励路径。roadmap 节点来自当前连续位置、目标、窗口两侧 approach point 和膨胀墙体转角点；窗口边会从预计到达时间开始分析未来是否可通行，并把等待时间计入代价。因此奖励会在绕行更便宜时偏向绕行，在必须经过窗口时仍给出等待后通过的训练信号。

没有窗口通过奖励、路径跟踪奖励或 waypoint 奖励。动态进展 shaping 只用于训练奖励，不改变教师观测、碰撞规则或成功判定。

## 7. 输出

训练和评估只保存必要结果：

```text
checkpoints/teacher_final.pt
results/train_metrics.csv
results/eval_metrics.csv
results/typical_success.gif
results/typical_wait.gif
results/typical_collision.gif
```

`data/`、`checkpoints/`、`logs/`、`runs/`、`results/` 都是生成物目录，已从 Git 中忽略。

## 8. 当前验证状态

已通过：

```bash
pytest -q
python -m gap_step.train --config gap_step/configs/train_teacher_smoke.yaml
```

smoke 训练可以生成 `checkpoints/teacher_final.pt` 和 `results/train_metrics.csv`。完整收敛需要运行 `gap_step/configs/train_teacher_full.yaml`。
