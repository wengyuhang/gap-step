# GAP-step 实验实施文档 v4.6

## 0. 实验目标

项目总名称：**GAP-step**。

本实验实现一个连续二维旋转时变窗口迷宫环境，并训练一个 PPO 特权教师。实验只训练教师，不实现学生模型。教师观测保持低维，包含自身状态、目标相对位置和当前几何射线距离。窗口安全状态只参与当前可碰撞几何构造，不再作为显式窗口列表输入。环境默认奖励只包含目标奖励、碰撞惩罚、时间惩罚和动作惩罚；当前训练配置额外启用连续几何动态进展 shaping，用于缓解稀疏奖励下的超时局部最优。

本实验不构建显式训练集或测试集。训练和测试均由程序化迷宫生成器在线生成。课程学习通过改变程序化生成规则控制任务难度。

---

## 1. 项目结构

项目目录固定为：

```text
README.md
environment.yml
gap_step/
  __init__.py
  config.py
  configs/
    train_teacher_smoke.yaml
    train_teacher_full.yaml
  curriculum.py
  env.py
  evaluate.py
  gif.py
  model.py
  ppo.py
  train.py
  utils.py
  visualize.py
  tests/
    test_curriculum.py
    test_env.py
    test_model.py
checkpoints/
  teacher_final.pt
results/
  train_metrics.csv
  eval_metrics.csv
  typical_success.gif
  typical_wait.gif
  typical_collision.gif
```

当前实现要求所有可运行项目代码都直接放在 `gap_step/` 文件夹内，不再使用 `trainers/`、`scripts/`、`gap_step/envs/`、`gap_step/models/`、`gap_step/teachers/` 等旧目录。

---

## 2. 环境实现

### 2.1 连续迷宫

环境是连续方形区域：

```text
Omega_S = [0, S] × [0, S]
```

训练尺寸：

```text
S_train = [15, 19, 23]
```

测试尺寸：

```text
S_test_ID = [15, 19, 23]
S_test_OOD_size = [17, 21, 25, 31]
S_test_OOD_dynamics = [17, 25, 31]
```

### 2.2 墙体

墙体用连续轴对齐矩形表示：

```python
wall = {
    "xmin": float,
    "xmax": float,
    "ymin": float,
    "ymax": float,
}
```

迷宫由随机网格拓扑转换成连续横向/竖向墙段，而不是固定墙体模板。部分通路边被替换为时变窗口槽位。窗口槽位之外的区域全部作为静态墙体；窗口不可通行时，窗口区域按不可穿越区域处理；窗口可通行时，机器人可以通过该窗口区域。

### 2.3 机器人

机器人是圆盘：

```python
state = {
    "pos": np.ndarray(shape=(2,)),
    "vel": np.ndarray(shape=(2,)),
}
```

参数固定：

```python
robot_radius = 0.25
dt = 0.1
v_max = 2.0
a_max = 3.0
max_steps = 500
```

### 2.4 连续动作

动作是二维连续加速度：

```python
action = np.array([ax, ay], dtype=np.float32)
```

动力学：

```python
vel = np.clip(vel + action * dt, -v_max, v_max)
pos = pos + vel * dt
```

---

## 3. 时变窗口实现

每个窗口定义为：

```python
gate = {
    "id": int,
    "wall_id": int,
    "orientation": "vertical" | "horizontal",
    "center": np.ndarray(shape=(2,)),
    "slot_width": float,
    "d_min": float,
    "d_max": float,
    "omega_d": float,
    "phi_d": float,
    "theta0": float,
    "theta_amp": float,
    "omega_theta": float,
    "phi_theta": float,
    "theta_ref": float,
}
```

当前宽度：

```python
d = d_min + 0.5 * (d_max - d_min) * (1 + sin(omega_d * t + phi_d))
```

当前角度：

```python
theta = theta0 + theta_amp * sin(omega_theta * t + phi_theta)
```

当前是否可通行：

```python
safe_width = d >= 2 * robot_radius + safe_margin
safe_angle = abs(wrap(theta - theta_ref)) <= theta_safe
gate_safe = safe_width and safe_angle
```

环境内部使用 `d` 和 `theta` 计算 `gate_safe`。`gate_safe=False` 的窗口区域按障碍处理，`gate_safe=True` 的窗口区域按开口处理。教师观测不接收显式 `gate_safe` 列表。

---

## 4. 特权教师观测实现

函数接口：

```python
def get_privileged_obs(self) -> np.ndarray:
    pass
```

观测拼接顺序固定为：

```text
[ self_features, goal_features, ray_features ]
```

### 4.1 self_features

```python
self_features = [
    x / S,
    y / S,
    vx / v_max,
    vy / v_max,
]
```

### 4.2 goal_features

```python
goal_features = [
    (goal_x - x) / S,
    (goal_y - y) / S,
    norm(goal - pos) / S,
]
```

### 4.3 ray_features

使用 32 条射线：

```python
num_rays = 32
ray_max_dist = 0.35 * S
```

每条射线返回到最近墙体、窗口边框或环境边界的距离：

```python
ray_features[i] = min(hit_distance, ray_max_dist) / ray_max_dist
```

其中 `S` 是当前 episode 实际采样得到的连续迷宫边长。`num_rays` 固定为 32，`ray_max_dist` 随 `S` 缩放，因此测试更大迷宫时感知半径同步扩大，但观测维度仍保持：

```python
obs_dim = 4 + 3 + num_rays
```

### 4.4 不实现 gate_safe_flags 列表

代码中不设置 `max_gates`，不拼接 `gate_safe_flags`。窗口数量不参与观测维度。

实现规则固定为：

```python
# 每个 step 先根据窗口宽度和旋转角计算 gate_safe
# gate_safe=False: 该窗口区域加入当前障碍集合
# gate_safe=True : 该窗口区域从当前障碍集合中移除，表现为可通行开口
# ray_features 只对当前障碍集合做 raycast
obs = np.concatenate([self_features, goal_features, ray_features])
```

因此，测试阶段可以生成不同尺寸、不同窗口数量和不同拓扑复杂度的迷宫。网络输入维度不变，仍为：

```python
obs_dim = 4 + 3 + num_rays
```

### 4.5 禁止输入项

代码中不实现以下观测输入：

```text
static_map_flatten
A_star_path
waypoint
next_waypoint
path_direction
full_grid_map
gate_widths
gate_angles
gate_future_labels
```

---

## 5. 课程学习迷宫生成实现

### 5.1 总规则

训练阶段不预先生成训练集。每个 episode 调用一次：

```python
maze = sample_maze(stage_name: str, split: str, seed: int)
```

其中：

- `stage_name` 取值为 `C1`、`C2`、`C3`、`C4`、`C5`；
- `split` 取值为 `train`、`id_test`、`ood_size_test`、`ood_dynamics_test`；
- `seed` 用于保证测试可复现。

`sample_maze` 返回：

```python
maze = Maze(
    S=float,
    start=np.ndarray(shape=(2,)),
    goal=np.ndarray(shape=(2,)),
    walls=list[dict],
    gates=list[Gate],
    wall_segments=list[WallSegment],
    open_edges=set,
)
```

### 5.2 episode 生成流程

每个 episode 固定执行以下流程：

```python
S = sample_size(stage, split)
rows, cols, gate_range, extra_openings = stage_layout(stage)
open_edges = randomized_dfs_maze(rows, cols)
open_edges |= sample_extra_open_edges(extra_openings)
start_cell, goal_cell = choose_left_bottom_and_right_top_cells()
gate_edges = choose_passage_edges_for_gates(open_edges, gate_range)
wall_segments, gates = convert_grid_edges_to_continuous_geometry(open_edges, gate_edges, S)
walls = build_wall_rectangles(wall_segments, gates)
state = initialize_robot(cell_center(start_cell))
```

### 5.3 起点与目标

起点和目标放在普通迷宫拓扑的左下与右上区域，并使用对应网格单元中心映射到连续坐标：

```python
start_cell = (rows - 1, 0)
goal_cell = (0, cols - 1)
start = cell_center(start_cell)
goal = cell_center(goal_cell)
```

若起点或目标与墙体重叠，则做小幅平移修正。

### 5.4 网格迷宫转连续墙段

先用 randomized DFS 生成连通网格迷宫，再额外打开少量封闭边，形成少量环路。未打开的网格边转换为静态连续墙段；被选为窗口的通路边转换为带窗口槽位的墙段。每个墙段用以下结构表示：

```python
wall_segment = {
    "id": int,
    "orientation": "vertical" | "horizontal",
    "coord": float,
    "span": (float, float),
}
```

墙厚固定：

```python
wall_thickness = 0.12
```

竖向墙段在固定 `x=coord` 上沿 `y` 方向延伸；横向墙段在固定 `y=coord` 上沿 `x` 方向延伸。没有窗口的共线墙段会合并；带窗口的墙段单独保留，方便碰撞判定。

### 5.5 窗口槽位

每个窗口槽位最大几何宽度随所在网格边长度裁剪，上限为：

```python
slot_width = min(1.8, 0.72 * edge_span)
```

窗口中心位于被选中通路边的中心。窗口安全时可通行；窗口关闭或旋转角不安全时，窗口槽位按动态障碍处理。

### 5.6 C1 生成器

```python
C1 = {
    "sizes": [15],
    "grid": "3x4",
    "gate_count": 1,
    "extra_openings": 0,
    "gate_dynamics": "fixed_open",
}
```

C1 的窗口参数固定：

```python
d_min = 1.4
d_max = 1.4
omega_d = 0.0
phi_d = 0.0
theta0 = 0.0
theta_amp = 0.0
omega_theta = 0.0
phi_theta = 0.0
```

### 5.7 C2 生成器

```python
C2 = {
    "sizes": [15, 19],
    "grid": "4x5",
    "gate_count": "1-2",
    "extra_openings": 1,
    "gate_dynamics": "open_close",
}
```

窗口宽度参数：

```python
d_min = uniform(0.30, 0.50)
d_max = uniform(1.20, 1.60)
omega_d = uniform(0.45, 0.85)
phi_d = uniform(0.0, 2 * pi)
theta0 = 0.0
theta_amp = 0.0
omega_theta = 0.0
phi_theta = 0.0
```

### 5.8 C3 生成器

```python
C3 = {
    "sizes": [15, 19],
    "grid": "4x5 or 5x6",
    "gate_count": "2-3",
    "extra_openings": "2-3",
    "gate_dynamics": "open_close_rotate",
}
```

窗口宽度参数与 C2 相同。旋转参数：

```python
theta0 = 0.0
theta_amp = uniform(0.15, 0.40)
omega_theta = uniform(0.35, 0.75)
phi_theta = uniform(0.0, 2 * pi)
theta_safe = 0.25
```

### 5.9 C4 生成器

```python
C4 = {
    "sizes": [15, 19, 23],
    "grid": "5x6",
    "gate_count": "3-5",
    "extra_openings": 4,
    "gate_dynamics": "open_close_rotate",
}
```

窗口从起点到终点路径和其他通路边中采样。每个窗口独立采样 C3 的宽度和旋转参数。

### 5.10 C5 生成器

```python
C5 = {
    "sizes": [15, 19, 23],
    "grid": "6x7",
    "gate_count": "6-10",
    "extra_openings": 8,
    "gate_dynamics": "multi_async_open_close_rotate",
}
```

C5 生成最终普通迷宫式拓扑，包含横向和竖向墙段、少量环路以及 6-10 个异步时变窗口。所有窗口独立采样 C3 的宽度和旋转参数。由于每个窗口的频率和相位独立，C5 中窗口是异步变化的。

### 5.11 课程步数

课程顺序固定：

```python
curriculum_order = ["C1", "C2", "C3", "C4", "C5"]
```

每个课程阶段训练步数：

```python
steps_per_stage = 1_000_000
```

总训练步数：

```python
total_steps = 5_000_000
```

---

## 6. 碰撞判定

每一步更新位置后执行碰撞检查：

```python
collision = check_wall_collision(robot_circle, walls)
collision |= check_gate_frame_collision(robot_circle, gates)
collision |= check_closed_gate_crossing(prev_pos, pos, gates)
collision |= check_boundary_collision(pos, S)
```

其中：

- 撞墙直接失败；
- 撞窗口边框直接失败；
- 窗口 `gate_safe=False` 时穿越该窗口区域直接失败；
- 越界直接失败。

---

## 7. 奖励函数实现

环境默认保留 strict v4.6 稀疏奖励：

```python
reward = 0.0
reward += 20.0 if success else 0.0
reward += -20.0 if collision else 0.0
reward += -0.01
reward += -0.001 * np.sum(action ** 2)
```

当前训练配置启用连续几何动态进展 shaping：

```python
progress_reward = reward_progress * (prev_remaining_time - current_remaining_time)

reward += progress_reward
reward += reward_timeout if truncated else 0.0
```

其中 `current_remaining_time` 来自连续几何 visibility roadmap 的 time-dependent Dijkstra 估计。该 roadmap 使用当前连续位置、目标、窗口两侧 approach point 和膨胀墙体转角点；普通边要求线段不穿过膨胀静态障碍，窗口边单独通过窗口两侧 approach point 连接，并从预计到达时间开始采样未来 `gate.is_safe(t)` 计算等待代价。

该 shaping 不作为教师观测，不提供 waypoint，不提供完整地图展开，也不是窗口穿越奖励。窗口不可通行时实际穿越仍按碰撞处理。

代码中仍不实现以下奖励：

```text
gate_pass_reward
near_gate_reward
path_following_reward
alignment_reward
```

---

## 8. PPO 实现

Actor-Critic 网络结构固定为 MLP：

```text
obs_dim -> 256 -> 256 -> actor_mean(2)
obs_dim -> 256 -> 256 -> value(1)
log_std 为可学习参数
```

PPO 参数固定为：

```python
gamma = 0.99
gae_lambda = 0.95
clip_ratio = 0.2
value_coef = 0.5
entropy_coef = 0.01
learning_rate = 3e-4
rollout_steps = 2048
update_epochs = 10
minibatch_size = 256
max_grad_norm = 0.5
```

---

## 9. 程序化测试生成

### 9.1 ID 测试

ID 测试使用 C5 生成器，尺寸集合固定为：

```python
sizes = [15, 19, 23]
seeds = range(10000, 10200)
dynamics = "train_distribution"
```

### 9.2 未见尺寸测试

未见尺寸测试使用 C5 生成器，尺寸集合固定为：

```python
sizes = [17, 21, 25, 31]
seeds = range(20000, 20200)
dynamics = "train_distribution"
```

### 9.3 未见窗口动力学测试

未见窗口动力学测试使用 C5 生成器，尺寸集合固定为：

```python
sizes = [17, 25, 31]
seeds = range(30000, 30200)
dynamics = "ood_distribution"
```

训练外动力学参数：

```python
omega_d = uniform(0.90, 1.40)
theta_amp = uniform(0.40, 0.70)
omega_theta = uniform(0.80, 1.30)
d_max = uniform(1.00, 1.35)
```

---

## 10. 评估实现

`evaluate.py` 最终只写入一个汇总文件：

```text
results/eval_metrics.csv
```

每类测试运行：

```python
eval_episodes = 200
```

记录指标：

```text
split
success_rate
collision_rate
timeout_rate
average_return
average_steps
closed_gate_collision_rate
wall_collision_rate
boundary_collision_rate
average_action_norm
```

---

## 11. 可视化实现

`visualize.py` 只保存三类典型测试 GIF：

```text
results/typical_success.gif
results/typical_wait.gif
results/typical_collision.gif
```

每帧显示：

- 连续墙体；
- 旋转时变窗口；
- 窗口当前是否可通行；
- 机器人位置；
- 机器人轨迹；
- 起点；
- 目标点；
- 当前 step；
- 当前回合结果。

---

## 12. 输出保存规范

实验只保存模型、指标和典型测试 GIF。其他过程数据不保存。

### 12.1 必须保存的文件

```text
checkpoints/
  teacher_final.pt
results/
  train_metrics.csv
  eval_metrics.csv
  typical_success.gif
  typical_wait.gif
  typical_collision.gif
```

保存规则固定如下：

| 文件 | 内容 |
|---|---|
| `checkpoints/teacher_final.pt` | 最终 PPO 特权教师网络参数 |
| `results/train_metrics.csv` | 训练阶段汇总指标 |
| `results/eval_metrics.csv` | ID、尺寸外推、窗口动力学外推测试指标 |
| `results/typical_success.gif` | 典型成功场景 |
| `results/typical_wait.gif` | 典型等待窗口后通过场景 |
| `results/typical_collision.gif` | 典型碰撞失败场景 |

### 12.2 禁止保存的文件

代码中不保存以下内容：

```text
rollout_buffer.pt
trajectory_dataset.pkl
raw_states.npy
raw_observations.npy
raw_actions.npy
episode_frames/
intermediate_checkpoints/
debug_logs/
env_snapshots/
```

PPO rollout buffer 只在内存中使用。每次策略更新完成后清空。生成 GIF 时可以在内存中保存帧列表，GIF 文件写出后不保留单帧图片。

### 12.3 checkpoint 覆盖规则

训练结束只保存：

```text
teacher_final.pt
```

训练过程中如果实现临时恢复文件，只能保存：

```text
teacher_latest.pt
```

`teacher_latest.pt` 每次覆盖旧文件，不保留历史 checkpoint。

---

## 13. 运行命令

创建环境：

```bash
conda env create -f environment.yml
conda activate wyh
```

训练：

```bash
python -m gap_step.train --config gap_step/configs/train_teacher_full.yaml
```

评估：

```bash
python -m gap_step.evaluate --checkpoint checkpoints/teacher_final.pt
```

可视化：

```bash
python -m gap_step.visualize --checkpoint checkpoints/teacher_final.pt
```

---

## 14. 验收标准

代码运行结束后必须产生：

```text
checkpoints/teacher_final.pt
results/train_metrics.csv
results/eval_metrics.csv
results/typical_success.gif
results/typical_wait.gif
results/typical_collision.gif
```

指标达到：

```text
ID_success_rate >= 0.75
OOD_size_success_rate on 25 >= 0.60
OOD_size_success_rate on 31 >= 0.45
OOD_dynamics_success_rate >= 0.45
closed_gate_collision_rate <= 0.10
```
