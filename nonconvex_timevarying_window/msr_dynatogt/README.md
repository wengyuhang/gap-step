# MSR-DynaTOGT

MSR-DynaTOGT（Multi-Start and Repair DynaTOGT）是 `nonconvex_timevarying_window/` 下的独立算法。它复用 SC-DynaTOGT 已验证的非凸窗口内部映射、动态窗口、degree-7 MINCO、四旋翼微分平坦性和联合 L-BFGS-B 接口，在外层增加多初值候选搜索和动力学可行性修复。

实现受到 `../../复现/论文/2607.pdf` 中“先生成多样候选、保留候选池、再精化高质量候选”的工程思路启发，但本方法的任务始终是固定窗口顺序下的时间最优穿越。它不是 DVOP、LNS 或强化学习算法，不改变窗口集合或穿越顺序。

## 与 SC-DynaTOGT 的区别

| 方面 | SC-DynaTOGT | MSR-DynaTOGT |
|---|---|---|
| 局部优化启动 | 一个初值对应一次局部优化 | 系统生成中心、随机、转弯感知和区域分散初值，并保留去重候选池 |
| 候选排序 | 优化器目标和收敛状态 | 先窗口顺序/内部合法，再高密度采样动力学可行，最后才比较总时间 |
| 动力学限制 | 优化中使用软惩罚，优化后主要诊断 | 软惩罚不变，额外做高密度检查、时间修复和完整联合再优化 |
| 时间修复 | 无 | `uniform` 整体放慢，或 `local` 只放慢超限段及邻域；逐步放大后用二分逼近 |
| 共同部分 | 圆盘 SC、时变平移/旋转/均匀缩放、degree-7 MINCO、微分平坦性、单旋翼推力/角速度代价、`x=[K,D]` L-BFGS-B | 完全相同 |

`success` 的含义是窗口合法且通过高密度采样动力学限制。代码另存 `optimizer_success`，绝不把优化器收敛标志当作动力学可行。所有“可行”结论只能称为**高密度采样可行**，不是连续时间严格证明。

## 算法流程

```text
同一 SCWindowTrack
  -> 生成多种可复现 x0=[K,D]
  -> 每个 x0 运行现有 SC-DynaTOGT L-BFGS-B
  -> 高密度采样窗口/速度/推力/角速度
  -> 候选池去重并按 (窗口合法, 动力学可行, 总时间) 排序
  -> 对超限候选做 uniform/local 时间修复和二分
  -> repaired T -> K，再做一次完整 [K,D] L-BFGS-B
  -> 重新高密度验证；若再优化破坏可行性，保留修复后的可行 incumbent
  -> 输出 A0/A1/A2/A3 与两种公平比较
```

详细公式和修复判据见 [ALGORITHM.md](ALGORITHM.md)，实验协议见 [EXPERIMENTS.md](EXPERIMENTS.md)。

## 命令

```bash
python -m compileall -q nonconvex_timevarying_window/msr_dynatogt
pytest -q nonconvex_timevarying_window/msr_dynatogt/tests
python -m nonconvex_timevarying_window.msr_dynatogt.experiments --suite smoke
python -m nonconvex_timevarying_window.msr_dynatogt.experiments --suite formal
python -m nonconvex_timevarying_window.msr_dynatogt.experiments --suite formal --workers 2
python -m nonconvex_timevarying_window.msr_dynatogt.experiments --suite smoke --repair-mode uniform
```

`formal` 默认使用每场景 155 个种子，与 SC-DynaTOGT 动态正式实验的种子数量对齐；计算量很大。默认按进程实际 CPU 亲和性选择并发数（上限 12），每完成一个场景—种子任务即写入检查点。中断后可用 `--resume results/<timestamp>_formal` 继续，配置不一致时会拒绝合并。`smoke` 使用一个种子和每段 65 个高密度采样节点，验证完整调用链。

## 结果

结果只能写入本目录的 `results/`。每次运行创建带微秒时间戳的新目录，不接受覆盖已有运行。每个目录至少包含：

- `config.json`、`runs.csv`、`summary.json`、中文 `REPORT.md`和逐图通俗说明 `FIGURE_EXPLANATIONS.md`；
- 可续跑进度 `status.json` 和运行中的 `runs.partial.csv`；
- 五类场景的 A0–A3 轨迹对比图；
- 总时间、计算时间、采样动力学可行率和修复前后推力图；
- 每个场景/种子的原始与修复候选完整 JSON。

真实执行记录见 [TEST_RESULTS.md](TEST_RESULTS.md)。

2026-07-22 完成的 formal 套件位于 `results/20260721_135308_842525_formal/`：5 个场景各 155 个种子，共 775 个任务和 9,300 行协议记录。A0/A1/A2/A3 的高密度采样动力学可行率分别为 0.0%/0.1%/100%/100%；A3 平均飞行时间为 `5.366490 s`，平均求解墙钟为 `2385.702 s`。A0 均不可行，因此不把 A3/A0 时间差宣称为可行解之间的性能优越性。

## AtlasDynaTOGT 辅助比较

AtlasDynaTOGT 与本方法都处理简单非凸动态窗口，但 Atlas 当前使用三次 Hermite、各向异性缩放和速度/加速度/jerk 指标；MSR/SC 使用 degree-7 MINCO、均匀缩放、单旋翼推力和角速度。动力学模型、场景与接口不同，因此仅作结构性辅助说明，不把数值直接排名，也不宣称 MSR 对 Atlas 性能优越。

## 文件结构

```text
config.py               算法、初值、高密度采样、候选池和修复配置
initializations.py      中心/随机/转弯感知/区域分散初值
candidate_pool.py       去重与合法性优先排序
feasibility_repair.py   高密度检查、uniform/local 放慢、二分与再优化
solver.py               共享局部求解及 A0–A3/公平协议
experiments.py          smoke/formal 五场景入口
comparison.py           对比图和 Atlas 可比性说明
results_manager.py      时间戳目录、CSV/JSON、汇总和中文报告
tests/                  单元与端到端烟测
```

## 已知限制

- 高密度采样可能漏掉节点之间的瞬时峰值；没有连续时间证书。
- 时间修复只沿统一比例或局部分段方向搜索，不保证全局最小可行时间。
- 再优化仍是非凸 L-BFGS-B，多初值提高覆盖但不提供全局最优保证。
- 当前只支持无洞、无自交、偏置后单连通的窗口，按指定顺序各穿越一次。
