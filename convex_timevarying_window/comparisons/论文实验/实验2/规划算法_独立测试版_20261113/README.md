# 规划算法独立修改与测试副本

此副本对应 2026-11-09 第一个正式结果的规划实现，以及 2026-11-12 成功复现这批规划结果所用代码。原始文件和历史实验均保留。复制日期为 2026-11-13。

## 修改位置

- `code/convex_timevarying_window/comparisons/论文实验/实验2/soft_dynamics/fast_ipopt.py` 有限 IPOPT 问题
- 同目录 `kernels.py` 轨迹、动力学与几何表达式
- 同目录 `phase_initialization.py` 初始化与优化坐标
- `code/convex_timevarying_window/global_gate_minco_probe/planner.py` 安全检查与补点循环
- `code/convex_timevarying_window/comparisons/论文实验/实验2/higher_swing/run.py` 本文自由/中心参数、初值和单次求解
- 同目录 `scenario.py` 七窗赛道与运动
- 同目录 `matrix.py` 原完整实验协调器（包含实飞，单纯规划测试请用下方单候选入口）

## 版本口径

这是 Python/CasADi + IPOPT 及 C++ 辅助实现。保持原完整 PVAJ 表达式、MINCO 基准导数偏移和累计到达时刻坐标，不混入后续原生 MINCO 改动。原正式实验为八个自由点与八个中心初值，再用最佳中心结果开放自由点优化。此处的单候选例子仅用于调试，不代表完整多初值成绩。

源码及配置均为独立普通副本，无原源码链接。原协议 2558 个文件逐项 SHA-256 核对通过。保留复现时补齐的依赖和资源，未复制历史实飞数据；旧模块导入所需的一份 summary.json 单独保留。仍依赖本机 wyh 环境及本机数值库。

## 单候选测试

在本目录运行

```bash
./run_candidate.sh --outdir "$PWD/examples/a50_f6" --job ours_0
./run_candidate.sh --outdir "$PWD/examples/a50_f6" --job center_0
```

另附 `examples/a75_f6/jobs/`。输出写入所选 outdir 的 plans 子目录。重复实验时新建输出目录并复制 jobs，不能覆盖旧结果。脚本仅规划，不启动 Gazebo。完整八初值可在 jobs 中设 seed 为 0 至 7，并为每项使用唯一 id。

COPY_MANIFEST.json 保存源码来源及复制校验。本次仅复制和验证入口，不启动正式优化或实飞。
