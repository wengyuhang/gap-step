# FAPP-PPO

FAPP-PPO（Future-Aware Privileged-Preview PPO，未来感知特权预览 PPO）是 `closed_loop_deformable_window/` 下的第一个强化学习算法。它处理未来动态完全已知的连续形变非凸窗口，控制四旋翼按指定顺序各穿越一次，然后恢复到初始位置、速度、姿态和角速度。

该方法的定位是“可运行的首个研究基线”，不是全局时间最优或连续时间安全证明。

## 方法组成

- **固定未来实例**：每次 reset 先生成完整非周期关键帧；之后窗口状态可在任意时刻连续查询；
- **每窗独立外生变化**：开放日程、位姿运动和局部形变分别使用独立随机流，生成器不读取无人机状态、路线长度或预计到达时间；
- **真实非凸安全区**：每帧从真实简单多边形内缩得到安全区域，不用凸包代替；
- **有限预览 actor**：看到 \(v,R,\omega\)、当前/后续窗口的多时刻几何预览和名义 CTBR；
- **特权 critic**：额外看到绝对时间和整条剩余窗口序列的预测状态；
- **残差 CTBR PPO**：物理预览控制器提供保守 CTBR，PPO 学习有界残差，并用 residual prior 防止短训练破坏可行先验；
- **四旋翼约束**：CTBR 经期望力矩和混控矩阵分配到四个旋翼，单旋翼推力逐个裁剪；
- **课程训练**：`static -> moving -> deforming -> full`；
- **时间关键机会**：窗口可连续收缩到安全内缩区为空，actor 读取开放状态和开放/关闭倒计时，策略必须等待并抢占短暂可通行区间；
- **闭环成功条件**：四个窗口全部按序合法穿越，且 \(p,v,R,\omega\) 同时回到初始容差内。

算法公式、奖励和约束判定见 [ALGORITHM.md](ALGORITHM.md)，2026 顶会论文依据见 [PAPER_NOTES.md](PAPER_NOTES.md)。
当前自动化测试、负结果诊断和 60 个未见实例的短训练预实验见 [TEST_RESULTS.md](TEST_RESULTS.md)。
大幅形变、暂时不可通行、配对基线、消融、统计检验和论文图表的预注册方案见 [ACADEMIC_EXPERIMENTS.md](ACADEMIC_EXPERIMENTS.md)。

## 安装依赖

仓库的 `wyh` 环境已经包含所需包。独立安装时：

```bash
pip install -r closed_loop_deformable_window/fapp_ppo/requirements.txt
```

## 快速验证

```bash
pytest -q closed_loop_deformable_window/fapp_ppo/tests
python -m closed_loop_deformable_window.fapp_ppo.experiments --suite smoke
```

烟测只验证完整调用链，不代表训练收敛。

时间关键机会模式的链路验证：

```bash
python -m closed_loop_deformable_window.fapp_ppo.train \
  --config closed_loop_deformable_window/fapp_ppo/configs/train_opportunity_smoke.yaml
```

复现 30-update 短训练预实验：

```bash
python -m closed_loop_deformable_window.fapp_ppo.train \
  --config closed_loop_deformable_window/fapp_ppo/configs/train_validation.yaml
```

## 正式训练与评估

```bash
python -m closed_loop_deformable_window.fapp_ppo.train \
  --config closed_loop_deformable_window/fapp_ppo/configs/train_default.yaml

python -m closed_loop_deformable_window.fapp_ppo.evaluate \
  --checkpoint closed_loop_deformable_window/fapp_ppo/runs/default/checkpoints/fapp_ppo_final.pt \
  --episodes 100 --stage full

python -m closed_loop_deformable_window.fapp_ppo.visualize \
  --checkpoint closed_loop_deformable_window/fapp_ppo/runs/default/checkpoints/fapp_ppo_final.pt \
  --stage full \
  --output closed_loop_deformable_window/fapp_ppo/results/rollout.png

python -m closed_loop_deformable_window.fapp_ppo.video \
  --checkpoint closed_loop_deformable_window/fapp_ppo/runs/default/checkpoints/fapp_ppo_final.pt \
  --stage full \
  --output closed_loop_deformable_window/fapp_ppo/results/rollout.mp4
```

`experiments --suite default` 会顺序执行正式训练、100 回合评估和一条 3D 轨迹图。每次实验使用带时间戳的新目录。

时间关键正式训练和九条件配对评估：

```bash
python -m closed_loop_deformable_window.fapp_ppo.train \
  --config closed_loop_deformable_window/fapp_ppo/configs/train_opportunity_default.yaml

python -m closed_loop_deformable_window.fapp_ppo.academic_experiments \
  --checkpoint closed_loop_deformable_window/fapp_ppo/runs/independent_default/checkpoints/fapp_ppo_final.pt \
  --suite default --episodes 200 --seed-start 50000
```

## 主要输出

```text
<run>/
  config.yaml
  train_metrics.csv
  train_summary.json
  checkpoints/fapp_ppo_final.pt
  evaluation_summary.json
  evaluation_records.json
  rollout.png
  rollout.mp4
```

论文图和 MP4 的可见文字统一使用中文，PPO、CTBR、KL、ID 等通用缩写保留。
验证训练完成并运行一次配对评估后，可生成五张论文图：

```bash
python -m closed_loop_deformable_window.fapp_ppo.figures \
  --config closed_loop_deformable_window/fapp_ppo/configs/train_opportunity_validation.yaml \
  --metrics closed_loop_deformable_window/fapp_ppo/runs/independent_validation_v3/train_metrics.csv \
  --academic-dir <本次配对评估目录> \
  --outdir closed_loop_deformable_window/fapp_ppo/runs/independent_validation_v3/paper_figures \
  --seed 53000
```

五张图依次解释真实训练奖励、算法数据流、独立窗口随机过程、预注册实验协议和先导结果；
逐图的通俗中文说明见 [FIGURE_GUIDE.md](FIGURE_GUIDE.md)。

当前可直接播放的中文 MP4 为
`runs/independent_validation_v3/fapp_ppo_early_checkpoint_independent_seed53017_zh.mp4`。
它使用 update 25 的成功诊断回合：四窗按序穿越并在 16.36 秒恢复全状态。最终检查点的
先导成功率为 0/10，因此这个视频明确标为早期检查点示例，不作为论文主性能结论。

评估至少报告：

- 闭环成功率；
- 仅对成功回合统计的平均闭环时间；
- 平均合法穿越数；
- 碰撞率；
- 乱序率；
- 每回合四类闭环状态误差。

## 已知限制

- 窗口边框碰撞和窗口间隔采用离散仿真/采样检查，不是连续时间证书；
- 安全区用 Shapely 多边形内缩，边界关键帧需具有一致采样拓扑；
- 当前 actor 使用状态和未来几何，不使用相机或 LiDAR；
- 名义控制器使未训练策略具备基本飞行能力，性能提升应通过相同名义控制器的 residual=0 消融来验证；
- PPO 只优化期望回报，不保证找到全局最短闭环时间；
- 正式训练尚需实际运行后才能给出收敛率和统计结论。
