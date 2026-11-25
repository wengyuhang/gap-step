# 原生Kraft SLSQP数值内核

原样复制仓库既有SciPy 1.14.1/Kraft实现及BSD许可，仅提供数值优化例程；不含旧SLSQP分支的轨迹、硬动力学、初始化或安全逻辑。

来源：https://github.com/scipy/scipy/blob/v1.14.1/scipy/optimize/slsqp/slsqp_optmz.f

2026-11-17从同仓`实验2/SLSQP的c++实现/vendor/slsqp`复制，Fortran源码SHA-256：a1dae36fff99abc781931e1afbd886cdd700f12bb2f80418c8fc95a734efae69。内核未改；由C++当前有限问题适配器调用。
