# 五方法现有轨迹实飞

本目录整理实验二当前十赛道五方法的50项记录，选中49条完整轨迹，统一执行并保留原始证据。

reference.py只适配原多项式PVAJS和北图参考姿态，复用现有单重射击MPC及PX4执行模型。flight.py接通统一物理赛道、PX4、MPC和原生记录。audit.py从ULog与实际窗口/接触消息独立核对完整竞速。archive.py无损保存本批原生日志。

运行使用wyh环境、单线程BLAS、GZ_SHOW_GUI=0、PYTHONDONTWRITEBYTECODE=1、NUMBA_CACHE_DIR=/tmp/convex_trajectory_only_numba。原始参考来自wang2023_full_v1_20261107的comparison_50.json。每次run使用新的结果目录。

完整跟踪回归还需ACADOS_SOURCE_DIR=/home/jack/wyh/dependencies/acados-togt-v0.5.4及LD_LIBRARY_PATH包含该目录的lib。单重射击MPC复用已编译的PX4物理预测器。

```bash
python -m convex_timevarying_window.comparisons.论文实验.实验2.gazebo_matrix.run run --outdir <新目录>
python -m convex_timevarying_window.comparisons.论文实验.实验2.gazebo_matrix.run validate --outdir <已完成目录>
```

目录中的catalogue.json记录每条原轨迹来源和SHA，references保存全部选中系数，flights保存逐次实飞和压缩日志，REPORT.md列完整返航时间与接触结果。冻结协议见PROTOCOL.md。
