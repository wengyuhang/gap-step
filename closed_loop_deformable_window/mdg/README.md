# MDG：连续形变非凸窗口的移动圆盘图规划器

MDG（Moving-Disc Graph for Continuously Deforming Gates）是
`closed_loop_deformable_window/` 下与 FAPP-PPO 并列的独立确定性离线规划方法。
两者共享上级目录的闭环连续形变窗口[问题定义](../PROBLEM_DEFINITION.md)，但不共享
算法实现。MDG 将真实动态非凸
安全区转换为移动圆盘轨迹，以粗到细分层时空图选择圆盘和穿越时刻，再复用仓库
现有的 degree-7 MINCO、四旋翼平坦性约束和 L-BFGS-B 联合优化最终轨迹。

物理开口可以始终保持有效，而考虑无人机尺寸与安全裕度后的安全区可以暂时为空。
MDG 只在安全区非空且能够生成安全圆盘的开放时刻建立图节点，并在指定顺序和飞行
时间约束下选择可达机会；若任一窗口没有可用开放时刻，或各窗开放机会无法按顺序
连接，则明确返回失败，不从闭合窗口强行穿越。

完整方法见 [docs/METHOD.md](docs/METHOD.md)，实验协议见
[docs/EXPERIMENTS.md](docs/EXPERIMENTS.md)，实施与测试状态见
[docs/IMPLEMENTATION_STATUS.md](docs/IMPLEMENTATION_STATUS.md)，已执行记录见
[docs/TEST_RESULTS.md](docs/TEST_RESULTS.md)。

## 安装与运行

```bash
source /home/jack/anaconda3/etc/profile.d/conda.sh
conda activate wyh
python -m pip install -e closed_loop_deformable_window/mdg
cd closed_loop_deformable_window/mdg
python scripts/run_single.py \
  --config configs/default.yaml \
  --seed 0 \
  --method mdg_free \
  --save-video
```

脚本包含仓库路径引导，不做可编辑安装也可从 MDG 目录直接运行。快速烟测和正式
E1–E6：

```bash
python scripts/run_benchmark.py --config configs/smoke.yaml --suite smoke
python scripts/run_benchmark.py --config configs/default.yaml --suite formal --workers 8
```

正式命令可断点续跑：已有且 `metrics.json` 标记 `run_complete=true` 的实例会跳过。
所有方法读取相同种子生成的完整场景，失败实例也会保存优化尝试和失败原因。

## 方法和结果

支持 `center`、`largest_disc`、`uniform_point`、`mdg_center`、`mdg_free`
和仅用于四窗口近似参考的 `dense_oracle`。Dense Oracle 是更密的离散参考，不是
连续问题的全局最优证书。

每个完成实例包含解析配置、场景控制点、圆盘轨迹、粗细图、选中路径、MINCO
轨迹 CSV、指标 JSON、调试日志、静态图和可选 MP4。`results/` 受仓库忽略规则
保护，可由脚本重建。

## 测试

```bash
pytest -q closed_loop_deformable_window/mdg/tests
python -m compileall -q closed_loop_deformable_window/mdg/src
```
