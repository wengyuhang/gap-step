# 五方法无倾角惩罚实验

在仓库主工作树、wyh环境中运行，保留原十条件及四独立冷初值。协议见[PROTOCOL.md](PROTOCOL.md)。

先运行模块no_tilt_comparison.build，原子重建native x500与任意多项式软积分库，再运行no_tilt_comparison.run。完整模块前缀为convex_timevarying_window.comparisons.论文实验.实验2。

输出四十份新规划、对应实飞及原生记录，复用原本文硬版二十项和Wang十项数据。report模块分别导出两套五方法规划与实飞表，同时保留来源、完整参考执行时间及历史实测时间的字段区别。
