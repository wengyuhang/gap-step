# 实验二存储清理与原始证据恢复

2026-11-06，按用户要求清理主工作树中的实验二；不修改算法、实验配置、碰撞判据或结果分类，不启动新飞行。

已完成：结果文件实际占用从18.86GiB降至5.31GiB，释放13.55GiB（14,546,755,584字节）。174份文本由14.83GiB压缩到2.15GiB；28,653份相同快照文件、59对相同遥测文件完成保留路径去重，删除一份37,186字节可再生成缓存。
全部174份档案解压SHA-256与33,401个其他保留文件的清理前指纹一致。统计不含维护清单自身数MiB开销；实验二全目录`du -sh`约5.4G。

## 空间来源与处理

主要空间来自1000Hz接触/位姿文本。碰撞后无人机留在地面直到180s超时，持续产生大量重复接触记录。多轮布局校准还各自保存了完整源码快照。

- 大于等于1MiB的`.pbtxt`日志改为`.pbtxt.gz`。先完成压缩，再独立解压核对原字节数与SHA-256；通过后才移除未压缩副本。所有失败记录仍保留。
- 相同内容、相同权限的归档`source_snapshot`文件使用硬链接去重；路径、字节和冻结源哈希不变。只连接归档快照之间，不与正在开发的源码连接。快照不可原地编辑。
- 完全相同的已结束飞行`telemetry_live.csv`与`telemetry.csv`保留双路径、使用硬链接；移除可再生成的`.pyc`缓存。
- 不删除ULog、轨迹、配置、世界、结果、报告、求解候选或失败条件；不改另一个工作树的原始记录。

清理前后容量见[summary.json](results/storage_cleanup_20261106/summary.json)，逐日志大小和SHA-256见[manifest.jsonl](results/storage_cleanup_20261106/manifest.jsonl)，全部保留文件与压缩包的复核见[verification.json](results/storage_cleanup_20261106/verification.json)。

## 读取与复查

已生成的结果表和验收JSON可直接读取。原始文本可用`gzip -cd <文件.pbtxt.gz>`流式读取。

**旧审计脚本并不自动读取`.gz`。运行前必须恢复其所需批次的原始日志；不能把压缩后原路径不存在解释为零接触。** 恢复不会改动冻结源，也不能绕过原有源码一致性检查。

在主仓根目录恢复指定批次，例如：

```bash
/home/jack/anaconda3/envs/wyh/bin/python \
  convex_timevarying_window/comparisons/论文实验/实验2/results/storage_cleanup_20261106/maintain.py \
  restore --select compact_layout_px4_v3_20261106/
```

`--select`也可指定单个日志的相对路径。工具先核对剩余空间，逐份解压并核对原始SHA-256；压缩包始终保留。不带`--select`会请求恢复所有日志，可能重新占用约16GB，不建议日常使用。

只读验证压缩档案及所有被保护文件：

```bash
/home/jack/anaconda3/envs/wyh/bin/python \
  convex_timevarying_window/comparisons/论文实验/实验2/results/storage_cleanup_20261106/maintain.py verify
```

维护工具位于结果目录，不加入实验算法源码。`plan.json`是执行前清单，`protected_sha256.json`保存清理前其他全部文件的内容指纹。不要重复启动`clean`；已有清单时工具会拒绝再次执行。
