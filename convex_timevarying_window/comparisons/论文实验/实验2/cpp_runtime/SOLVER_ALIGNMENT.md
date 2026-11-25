# 当前有限求解器同步

2026-11-17：IPOPT和SLSQP使用同一`finite_nlp::Model`。完整同步项、缺口修复及8组逐值验证见[同步报告](../classic_dynamic_tracks_20261115/solver_alignment_20261117/REPORT.md)。

模型只在当前`src/sip_planner.cpp`、`include/native_minco_pvaj.hpp`、`include/restored_kernels.hpp`和当前几何模块维护；`include/finite_nlp.hpp`将同一个有限问题交给两个后端。`include/native_slsqp.hpp`只有通用求解器适配，不定义无人机或窗口模型。底层内核在`vendor/slsqp`，旧SLSQP规划包保留为历史归档。
