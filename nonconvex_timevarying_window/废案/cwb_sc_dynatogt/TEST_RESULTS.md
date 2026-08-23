# CWB-SC-DynaTOGT 测试记录

更新时间：2026-08-02。

## 已执行

```text
pytest -q nonconvex_timevarying_window/cwb_sc_dynatogt/tests
12 passed in 0.78s

pytest -q nonconvex_timevarying_window/sc_dynatogt/tests
111 passed in 31.17s

python -m compileall -q nonconvex_timevarying_window/cwb_sc_dynatogt
passed

pytest -q
46 passed in 3.04s

python -m nonconvex_timevarying_window.cwb_sc_dynatogt.experiments \
  --suite smoke --outdir /tmp/cwb_sc_smoke
completed; explicit UNSAFE result after outer budget, no false safe claim
```

烟雾实例当前用于验证完整调用链和失败语义，不作为性能结果。该实例在 3 轮外层预算后仍有
2 个活动 witness，输出 `whole_body_status=unsafe`、`certified=false`；这验证了 L-BFGS 状态不会
替代整机验证状态。

原 SC-DynaTOGT 的 111 个测试全部通过；现有算法目录没有源码改动。
