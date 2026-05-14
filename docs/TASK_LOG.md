# Task Log

## 2026-05-11

完成项目主线整理：

- 活动代码集中到 `gap_step/`
- 保留连续二维迷宫、时变窗口、PPO teacher
- 移除旧视觉学生、BC、启发式 teacher、旧目录入口
- 增加课程迷宫、PPO 训练、评估和可视化

验证：

```bash
pytest -q
python -m gap_step.train --config gap_step/configs/train_teacher_smoke.yaml
```

## 2026-05-12

增加连续几何 progress shaping：

- `dynamic_geometry` progress mode
- reward potential 使用连续位置、墙、窗口和可见性
- 避免纯时间流逝产生 progress reward

结果：

- 训练能跑通
- 但成功率仍弱

## 2026-05-13

改为完整图特权教师：

- 新增 `GraphObs` 和图 batch
- 教师观测包含 cell、gate、拓扑边、gate 动力学
- 模型改为纯 PyTorch GNN actor-critic
- PPO 支持变长图 rollout 和 minibatch
- 课程拆成 `C1,C1_5,C2A,C2B,C3,C4,C5`

验证：

```bash
pytest -q
python -m gap_step.train --config gap_step/configs/train_teacher_smoke.yaml
```

## 2026-05-14 实验分析

完整训练结果显示：

- 训练一直停在 C1
- C1 rolling success 最高约 0.39，后期退化到 0
- entropy 和 std 持续升高
- PPO 后期经常被 KL early stop 卡到很少更新
- C5 评估成功率为 0

判断：

- 不是单纯动态门信息不足
- C1 就失败，说明基础连续导航不稳
- 全局 goal 向量会诱导直冲，但迷宫里起点到目标通常被墙阻断
- reward 和模型读出都需要小修

## 2026-05-14 本次修改

实现简化方案：

- 新增 `stagewise` 逐课程训练
- 每个课程单独保存 `checkpoints/<课程>/teacher_final.pt`
- 每个课程单独保存 `results/<课程>/train_metrics.csv`
- 不再保存 `teacher_best.pt`
- 移除训练进度条，改为中文实时指标
- 降低探索噪声：`entropy_coef=0.0001`，`log_std_init=-1.0`，`max_log_std=0.0`
- 撞墙或撞门时截断正 progress reward
- GNN actor/critic 读出加入 agent cell 和 goal cell 表示
- 更新全部项目文档

验证：

```bash
pytest -q
```

后续建议：

```bash
python -m gap_step.train --config gap_step/configs/train_teacher_smoke.yaml
python -m gap_step.train --config gap_step/configs/train_teacher_full.yaml
```

先观察 C1 是否稳定，再分析 C1_5/C2A。

## 2026-05-14 PPO 标准化

按参考 PPO 流程重写训练核心：

- 新增显式 `model_old`，rollout 只用旧策略
- PPO 更新只修改当前 `model`
- 每次 update 后同步 `model_old <- model`
- 保留 GAE 和 tanh-squashed Gaussian 连续动作
- KL 改为标准非负近似
- 日志和 CSV 增加裁剪率、解释方差、loss 等诊断指标
- 未采用 SB3，也不保留 SB3 训练入口

## 2026-05-14 收敛性诊断和通用奖励调整

诊断结果：

- C1 最短中心线可通，简单 waypoint-PD 控制器能成功
- 原地不动到超时的回报高于随机探索撞墙，容易形成“少动等死”局部最优
- 观测值基本在 `[-1, 1]`，没有 NaN，归一化不是主因

调整：

- `reward_timeout=-20.0`，让超时和碰撞一样是明确失败
- `reward_progress=4.0`，加强动态几何 progress shaping
- `log_std_init=-1.0`，`max_log_std=0.0`，降低初始乱撞
- `learning_rate=0.0001`，`target_kl=0.2`，`update_epochs=4`
- 完整训练每课程步数改为 300000，用于更快验证是否进入收敛趋势

## 2026-05-14 C5 tune 调参记录

实现：

- `GLOBAL_FEATURE_DIM` 扩展到 26
- `env.py` 增加动态几何引导特征、门等待/前方关闭门信息和归一化动作先验
- `model.py` 将动作先验转换为 tanh-squashed Gaussian 的 raw mean，PPO 继续学习残差
- `evaluate.py` 加载 checkpoint 内的 env 配置
- `visualize.py` 同样复用 checkpoint env 配置
- 新增 `gap_step/configs/train_teacher_c5_tune.yaml`
- 训练 CSV 增加 `guidance_reward_mean`

GPU 诊断：

- Codex 默认沙箱内 `torch.cuda.is_available()` 为 `False`
- 沙箱外 `wyh` 环境可见 `NVIDIA RTX A4000`
- 后续训练和评估使用授权的沙箱外命令运行

调参结论：

- 原始 C5 口径仍未达到 70%
- C5 tune 放宽口径使用：

```yaml
robot_radius: 0.1
safe_margin: 0.0
max_steps: 800
```

- 训练 rollout 中可出现 `0.75` 到 `0.80+` 的 C5 成功率
- 正式 50 回合 C5 ID 评估最好为 `0.68`
- 当前主要失败仍是 `closed_gate_collision`，其次是 wall collision

验证：

```bash
pytest -q
python -m gap_step.train --config gap_step/configs/train_teacher_c5_tune.yaml
python -m gap_step.evaluate --checkpoint checkpoints/C5/teacher_final.pt --episodes 50 --stages C5 --output results/eval_c5_tune.csv
```

## 2026-05-14 泛化评估和可视化

命令：

```bash
python -m gap_step.evaluate --checkpoint checkpoints/C5/teacher_final.pt --episodes 50 --output results/eval_generalization_50.csv
python -m gap_step.visualize --checkpoint checkpoints/C5/teacher_final.pt --device cuda --split id_test
```

结果：

```text
id_test          success_rate=0.68  closed_gate=0.16  wall=0.10  timeout=0.06
ood_size_test    success_rate=0.64  closed_gate=0.16  wall=0.06  timeout=0.14
ood_dynamics     success_rate=0.40  closed_gate=0.44  wall=0.06  timeout=0.10
```

结论：

- OOD size 退化不大，但 timeout 增加
- OOD dynamics 明显退化，主要由 closed gate collision 拉低
- GIF 已输出到 `results/typical_success.gif`、`results/typical_wait.gif`、`results/typical_collision.gif`
- 当前 checkpoint 下三个默认可视化 seed 都成功，`typical_collision.gif` 文件名沿用旧命名，不代表本次一定碰撞
