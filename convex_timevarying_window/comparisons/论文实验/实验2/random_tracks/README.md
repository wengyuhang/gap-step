# 随机赛道验证

生成器 generate.py 使用三个固定随机种子，沿旧 TOGT 布局扰动位置10%、偏航±10°，4平移/3摆动随机分配，独立相位和±15%幅频扰动。三难度分别幅度比/频率倍率0.25/1、0.5/3、0.75/6。同种子三难度配对筛查，全部通过才接纳。最大重采样5000次；若失败记录失败，不改变规则。

冻结九条位于 results/v2，生成及几何审查已完成，原五方法规划批已暂停，结果目录为 results/comparison_v2_20261113。规则不得依据方法成绩修改。当前仅名义固定平面模型，运动基底与窗框姿态分开；接入优化器时需要同步数值和符号运动接口。

运行采用本机已完成逐值复现的冻结算法目录 `/tmp/first_formal_soft_reproduction_code_20261112`，协议保存2558个原文件SHA。新增scenario.py仅适配冻结随机运动，bootstrap.py负责隔离加载。正式源与赛道哈希在启动后保持。

五方法四全局各8冷初值，本文自由点追加最佳中心释放分支，Wang自身局部多时间初值保持。REPORT.md随45项最终方法结果更新，candidate_summary.json保留全部候选。status.json需结合实际PID和progress.jsonl判断。未启动Gazebo。

当前修复状态（2026-11-13）：repair_verified_v2_20261113汇总64条已验收轨迹、161次正常有限求解，覆盖第一布局低中难及另两布局中难。高难八起点批repair_phase_eight_v2_20261113有8条合格结果，7个搜索结束；最后一个因相同有限问题反复恢复失败而停止，原日志保留。finite_recovery.py已接入同问题有界恢复；对照见finite_recovery_development_v1_20261113，完整入口复查见finite_recovery_full_validation_v1_20261113。后续周期继续优化单列repair_exact_phase_continuation_v1_20261113，不能混入独立八起点成绩。正式九赛道五方法对比保持暂停。

本轮继续优化已结束：原始初值2最终25.582053秒，额外1301.299秒；见repair_exact_phase_continuation_v1_20261113/REPORT.md，不覆盖原八初值表。正常路径两条真实回归逐值一致。
