# SLSQP原始实现

原始来源为2026-11-07正式硬约束矩阵`independent_full_matrix_v2_20261107/source_snapshot`，共2433个冻结文件，逐文件SHA校验通过。`code/`保存其完整源码。

## 实际使用的原始求解类

`code/convex_timevarying_window/comparisons/论文实验/实验2/independent_free_solve/planner.py`中的`Planner`。它是原正式矩阵实际调用的类，也作为随后首个软动力学版本的父类。

该类先进行原归一化SLSQP，必要时执行原同问题可行性恢复，再回到SLSQP。原节点硬动力学、north tilt/heading pi/2姿态模型、90%推力比例、几何复查、300秒/初值预算、2000次SLSQP上限和ftol1e-5均沿用冻结版本。

## 当前接入

- `adapters/higher_swing/scenario.py`来自2026-11-09高起伏赛道的冻结场景文件，用于当前十档赛道，三摆动窗绳长2.20米。
- `assets/cases.tar.gz`是当前原生MINCO包的原输入数据。
- `worker.py`只在冻结求解器外接入当前初值。八个初始段时长与实际VAJ逐值复制，点用原映射表示中心。逐候选检查窗口中心/平面/形状，以及初始轨迹节点PVAJ等价性。
- `run.py`启动双方各8个独立初值，每方法同时8进程，双方依次执行。按合格完整七窗返航最短时间择优，不追加中心结果开放选点。

这份历史实现使用Python/SciPy SLSQP与CasADi。计算墙钟包含八路候选启动、建模、原求解/恢复与独立几何检查。原初值的历史生成成本单列。

## 运行

在本目录使用conda环境wyh。输入包随目录提供，可先解压再运行。

```sh
mkdir -p build/cases
tar -xzf assets/cases.tar.gz -C build/cases
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
/home/jack/anaconda3/envs/wyh/bin/python -u run.py \
  --cases build/cases \
  --out outputs/original_eight_ten_new
```

默认先运行a25_f6、a50_f6、a75_f6，再运行其余七档。原始每初值记录、初值等价验证、求解状态与整体报告持续保存。原始代码内容校验在`provenance/FROZEN_SOURCE_VALIDATION.json`。

## 本次完整结果

十条件、双方各八初值共160次任务已完成，全部进程正常退出。自由点和中心均在8/10条件找到通过原版复查的轨迹。共同成功的八档，自由点完整轨迹时间均更短。0.5/6与0.75/6双方未找到合格解。八路初值的物理一致性、源码SHA、选优和汇总检查全部通过。

完整时间表及并行计算墙钟见[REPORT.md](outputs/original_eight_ten_v1_20261112/run/REPORT.md)，逐初值统计见[CANDIDATES.csv](outputs/original_eight_ten_v1_20261112/run/CANDIDATES.csv)，完整校验见[VALIDATION.json](outputs/original_eight_ten_v1_20261112/run/VALIDATION.json)。全部七窗返航轨迹与原始迭代记录均保存在各seed目录。本次为规划及名义几何评估。

恢复代码采用历史完整PVAJ表示和exp正时间映射。当前八初值的初始段时间、位置与实际VAJ已逐项核实。
