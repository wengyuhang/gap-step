# 实验二可再生成产物清理

2026-11-13 用户要求清理。仅删除旧 cpp_runtime/results/graphs_v1_20261110 下 .casadi、compiled_wang_v1_20261110 下自动生成 .c/.so 和运行源码目录 Python 字节码缓存。未删除受 Git 跟踪文件。

源码、导出工具、manifest/配置/初值、规划与实飞结果、接触记录、ULog、冻结源码和三个独立实现包保留。删除清单及 SHA-256 见 DELETED_FILES.json。

旧运行图下次执行前需要重新导出，编译库需要重新生成编译。相关工具在 ../cpp_runtime/tools，构建说明在 ../cpp_runtime/README.md。本次不启动求解或重建。
