# 0.20米规划安全球五方法重跑

2026-11-16全部完成，12条件60方法项，实体碰撞球0.15米。[完整规划和计算时间](PLANNING.md)、[与原0.30米批次对照](COMPARISON.md)、[最终审核](FINAL_AUDIT.json)。

C++ prepare.cpp同步五方法模型配置，SIP点域内缩0.2001米、有限/复查中心线距离0.2851/0.285米，窗口几何与运动保持。Wang规划代理0.20米，TOGT同有限半径内缩。576项输入、Wang图和二进制求解前冻结。实际优化、MINCO、补点和碰撞复查为原C++入口，report.py仅只读整理JSON。

全部本文合格候选经SIP_RESULT_CHECKS.log和独立实体/门序/终点审核，运动和Wang图检查在MOTION_AUDIT.json、WANG_AUDIT.json。旧结果保持。当前只规划，动力学仍为软惩罚，未新Gazebo实飞。
