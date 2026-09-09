# Feasibility-Guided CEM SC-DynaTOGT

完整变量定义、两类结构化前端、CEM 更新、硬筛选、整机审计及论文来源见 [ALGORITHM.md](ALGORITHM.md)。

原始 SC-DynaTOGT 局部最优解不满足新增的旋转窗口球体安全与严格动力学验收时，本方法用结构化前端和带记忆的完整协方差 CEM 搜索新的原生 `[K,D]`。这是与 `random_dk_sc_dynatogt/` 并列的新算法；旧方法保留为一次性独立均匀扰动基线。

## 搜索结构

1. 从已经独立通过单窗硬筛选的轨迹提取 `(安全穿越相位, D)`，为每个窗口枚举旋转周期别名，联合形成有明确物理意义的多窗初值。第一版三窗协议限制首窗到达时间为 1–3 s、相邻窗口间隔为 1.1–2.8 s，末段枚举 1.37/1.8/2.2 s。
2. 前端候选重新生成完整四段 MINCO，并执行与随机基线完全相同的球体穿越区间、穿越次数/顺序和原生动力学筛选。选择全三窗几何通过者中峰值速度最低的轨迹作为提议中心；这不是最终候选。
3. CEM 对四个原生 K 和三组 D 的极坐标进行联合高斯采样。K 协方差由航段时间雅可比归一化，并含共同时间分量；D 用角度和对数半径避免大模长附近的失真。每轮 256 个样本、32 个精英、16 个历史记忆，更新完整 10×10 协方差。
4. 找到可行解前，失败类别、已通过窗口数、球体余量和峰值速度只用于更新提议分布。它们不进入结果集合。找到首个可行解后再运行一轮，并仅在全部硬筛选通过者中按总时间排序。
5. 排序后的候选才执行真实姿态长方体最终审计；失败即淘汰并检查下一条。无最终通过者返回 `NO_FEASIBLE_CANDIDATE_FOUND`。

这种“失败证据指导下一次采样”与“结果只接受全约束通过者”是两个独立层次。最终规则为

`x* = argmin T(x), x in {全部中间硬约束通过且最终整机审计通过的候选}`。

设计依据包括：TOGT 的 `[K,D]` 光滑映射与 MINCO/L-BFGS 时空联合优化（<https://arxiv.org/html/2309.06837v3>）；iCEM 的相关采样与历史记忆（<https://proceedings.mlr.press/v155/pinneri21a.html>）；RSS 2020 用可行/不可行评估学习时间分配可行边界的多保真黑盒优化（<https://roboticsproceedings.org/rss16/p032.pdf>）。本实现是面向当前固定平面自旋窗口的原型组合，尚不据单次实验声称新的通用最优性。

## 运行

使用 conda `wyh`，从仓库根目录：

```bash
conda run -n wyh python -m nonconvex_timevarying_window.feasibility_guided_cem_sc_dynatogt.multi_window \
  --outdir nonconvex_timevarying_window/feasibility_guided_cem_sc_dynatogt/results/new_run

conda run -n wyh pytest -q \
  nonconvex_timevarying_window/feasibility_guided_cem_sc_dynatogt/tests \
  nonconvex_timevarying_window/random_dk_sc_dynatogt/tests
```

默认重放冻结的三窗 SC 基线，并使用两条已有单窗硬筛选通过轨迹作为显式前端模板。产物保存协议、模板来源、全部候选、逐轮 CEM 状态、严格筛选结果、最终审计和轨迹系数。所有结论是名义模型采样验证，不是连续域证书。
