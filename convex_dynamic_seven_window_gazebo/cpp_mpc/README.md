# C++ MPC 跟踪器

`controller.cpp` 在在线控制循环中直接用 C++ 完成 MAVLink 收发、参考插值、CasADi 求解、热启动和体轴角速度/总推力指令发送。`export_assets.py` 只在运行前离线导出参考轨迹和序列化 CasADi 函数。

依赖位于本方法忽略的 `.runtime/`：CasADi Python wheel 同时提供 C++ 头文件与动态库，MAVLink C 头文件来自 `c_library_v2`。CasADi wheel 使用旧 libstdc++ ABI，因此真实编译参数必须包含 `-D_GLIBCXX_USE_CXX11_ABI=0`。

```bash
./cpp_mpc/build.sh
./cpp_mpc/run_experiment.sh trajectories/our_method_x500_accepted_100hz.npz 100 4
```

`build.sh` 每次生成 `cpp_mpc/compile_commands.json`，内容与实际成功编译的参数一致。仓库根目录的本机 `.vscode/settings.json` 已将 VS Code C/C++ 和 clangd 指向该数据库；如果编辑器仍显示旧诊断，执行一次 **Developer: Reload Window** 或重启语言服务器。

2026-09-11 的最终 4 s 实测位于 `results/px4_togt_cpp_mpc/20260911_035610/`：400 次求解达到 100 Hz，平均/P95/最大求解时间为 `5.104/7.326/11.577 ms`。W1 首次接触发生在参考时间 `3.344 s`；接触前位置误差 RMS/均值/P95/最大值为 `0.2075/0.1644/0.4118/0.4952 m`。因此 C++ 已解决 Python 调度与代码索引问题，但没有把闭环误差降到 10 cm。
