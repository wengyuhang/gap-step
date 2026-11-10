# 单组取消墙钟求解上限

测试原半径自由点方法在a25_f6条件下的四个独立解析初值。直接调用原independent_free_solve.Planner.solve_all，seconds_per_seed使用正无穷。原求解器的收敛容差、迭代和恢复轮数仍作为算法终止规则记录。

结果分别保存规划时间、实际求解耗时、各初值终止原因和原模型验收。协议见PROTOCOL.md，入口run.py。结果为全局七窗加返航的名义规划。
