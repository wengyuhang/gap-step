# Closed-loop Passage Commitment Implementation Plan

> 用户已明确要求不调用技能、直接实现及排错；本文件保留原拟议验证清单，不是技能调用指令。实际完成情况以 COMMITMENT_RESULTS.md 为准，不追补未经执行的测试过程。

**Goal:** 原七窗60秒协议下验证完整穿窗与返航，消除旧电机缓存盲目续行，保留真实失败记录。

**Architecture:** 独立的CTBR预测/执行路径，GPU MPPI选择带反馈穿越机动，PX4负责角速度内环。四阶段门序管理控制入窗准入和完整退出；TOGT只提供正常模式参考。

**Tech Stack:** conda wyh、NumPy、C++、CuPy CUDA、MAVLink、PX4 x500、Gazebo。

**Spec:** [PASSAGE_COMMITMENT_DESIGN.md](PASSAGE_COMMITMENT_DESIGN.md)，用户2026-09-20批准直接实施。实际实现复用同一C++源的独立CTBR编译变体，并扩展共享拒绝原因ABI；旧电机模型回归保留。角速度一阶模型暂为经验近似，未完成拟议的严格辨识验收。

## Global Constraints

- 保留动态三窗TOGT、GPU MPPI、实际整机过门计数、当前实际PVAJ重规划和100 ms安全计算窗口。
- 不修改原七窗位置或运动来制造成功；其他门框仍保留碰撞审计。
- 不缩小现有机体模型或0.35 m误差裕量，不在失稳后放宽42度限制以获得“成功”。
- 100 ms为每次在线规划/控制的墙钟上限，所有筛选、回退与数据传输计入；预热单列。
- 20 ms更新目标也报告超期，禁止仅报告CUDA核时间。
- 原直接电机入口、54项旧测试、论文基线、历史结果不覆盖。新模块放本方法目录，不修改复现包。
- 所有命令从仓库根运行，解释器为`/home/jack/anaconda3/envs/wyh/bin/python`。
- 工作区有大量用户未提交文件；不暂存整目录、不重置、不自动提交无关内容。优先同目录新增模块隔离。

## Review Focus

1. 位置仍接近预测但姿态/角速度偏离：不得继续盲目电机开环（任务2、3）。
2. 窗口旋转扫回已越面机身：中心过面不计完成，出门后仍检查当前框（任务4）。
3. MPPI耗时/丢帧：反馈持续运行，但过期或超出适用范围不可冒充安全（任务3、5）。
4. 入窗无见证、退出时间超视界：禁止把未完成穿越的候选作为入窗依据（任务4）。
5. 角速度接口或推力归一化出错：先无门验证，禁止通过放宽误差界进入全赛道（任务1、2）。

## Task 1: 可诊断的执行接口与无门响应实验

**Files:** Create `ctbr.py`, `ctbr_probe.py`, `tests/test_ctbr.py` in this method directory.
Read `convex_dynamic_seven_window_gazebo/px4_togt_nmpc_experiment.py::send_bodyrate_thrust` and
`convex_dynamic_seven_window_gazebo/togt_nmpc.py::collective_thrust_to_px4` completely before reuse.

**Interfaces:** `encode_ctbr(total_thrust_n: float, rates_flu: ndarray[3]) -> (rates_frd, normalized_thrust)`;
`ctbr_probe` records measured rates, motor outputs, command timestamp and attitude in a new result directory.

- [ ] Write failing conversion tests, including invalid and nonfinite inputs:

```python
def test_flu_to_frd_preserves_roll_and_flips_pitch_yaw():
    rates, thrust = encode_ctbr(20., np.array([.1,.2,.3]))
    np.testing.assert_allclose(rates, [.1,-.2,-.3])
    assert 0 <= thrust <= 1

def test_nonfinite_command_rejected():
    with pytest.raises(ValueError):
        encode_ctbr(float('nan'), np.zeros(3))
```

- [ ] Run `python -m pytest -q convex_timevarying_window/dynamic_gate_mppi_v3/tests/test_ctbr.py`; confirm missing implementation fails.
- [ ] Implement conversion with existing installed thrust mapping, explicit finite/range validation and no silent unit conversion. Read actual PX4 parameters into probe artifacts; do not retune PX4.
- [ ] Repeat tests, then run no-gate hover and symmetric small roll/pitch/yaw rate pulses (0.2 rad/s, 0.2 seconds, separated by recovery). Keep independent position recovery only outside measurement segments and label it.
- [ ] Verify ULog angular-rate loop enabled and requested rate signs correct. Fit response/delay on one trace and validate on a separate trace; save observed error bounds, no claim of rigorous bounds.

## Task 2: 与新执行接口一致的预测模型

**Files:** Create `native/ctbr_rollout.cpp`, `ctbr_rollout.py`, `tests/test_ctbr_rollout.py`.
Reuse read-only geometry interfaces from `rollout.py` and `geometry.py`; do not change old dynamics ABI.

**Interfaces:** `CTBRRollout(track, config, response_model)`; `integrate(state, command, dt)`;
`evaluate(state, commands, prepared, first, phase) -> prediction`, with prediction fields `states`, `clearances`, `valid`, `indices`, `phases`, `rejection_bits`.
Command order `[total_thrust_N, wx_FLU, wy_FLU, wz_FLU]`; response-model file records fitted parameters and validation provenance.

- [ ] Write CPU/GPU parity and hover tests using identical finite response-model coefficients:

```python
def test_cpu_gpu_prediction_agrees(cpu, gpu, hover, commands, prepared):
    a = cpu.evaluate(hover, commands, prepared, 0, 0)
    b = gpu.evaluate(hover, commands, prepared, 0, 0)
    np.testing.assert_allclose(a.states, b.states, atol=5e-4, rtol=5e-5)
    np.testing.assert_array_equal(a.valid, b.valid)
    np.testing.assert_array_equal(a.rejection_bits, b.rejection_bits)
```

- [ ] Run the new tests before implementation; confirm failure.
- [ ] Implement fitted angular-rate response, thrust/motor lag, rigid-body translation and saturation in one CPU/CUDA source. Do not project old rotor candidates into CTBR after safety validation.
- [ ] Add multi-label bit flags: COLLISION=1, TASK=2, SPEED=4, ACCELERATION=8, TILT=16, BODY_RATE=32, ACTUATION=64, TERMINAL=128, NONFINITE=256. Preserve all violations, not just first failure.
- [ ] Compare against held-out task1 traces. If model error cannot fit retained safety assumptions, stop CTBR race integration and report mismatch; do not inflate admissible execution deviation.

## Task 3: 带反馈的穿越缓存

**Files:** Create `feedback_maneuver.py`, `tests/test_feedback_maneuver.py`.

**Interfaces:** `FeedbackManeuver` stores absolute times, desired P/V/quaternion/body rate, CTBR feedforward, gate, exit time, feedback gains, calibrated validity limits;
`command(actual_state, now) -> CTBR[4]`; `within_domain(actual_state, now) -> bool`;
`revalidate(actual_state, now, rollout, prepared) -> prediction`.
Feedback executes on fresh coherent telemetry; PX4 rate inner loop is not a replacement for outer attitude/position feedback.

- [ ] Write a failing test demonstrating orientation-only deviation with unchanged position changes the requested body rate:

```python
def test_same_position_rotated_body_changes_feedback(maneuver, x, rotate_body):
    perturbed = rotate_body(x, roll=.08)
    assert not np.allclose(maneuver.command(x, 0.)[1:],
                           maneuver.command(perturbed, 0.)[1:])
```

- [ ] Add expiry and NaN tests; outside-domain `command` raises an explicit exception rather than repeating final inputs.
- [ ] Implement quaternion shortest-arc attitude feedback and translational correction with bounded rates/thrust. Run the same law inside predictions; no unmodeled execution-only correction.
- [ ] Test withheld optimizer updates for 20/60/100 ms in the model and task1 probe. Record prediction error, tilt, rate and position separately.
- [ ] Confirm expired control, stale telemetry, and domain violation have distinct recorded outcomes. Ending a simulation is a failed run, not a safe recovery proof.

## Task 4: 四阶段门序与完整穿越准入

**Files:** Create `passage_commitment.py`, `tests/test_passage_commitment.py`.

**Interfaces:** `Phase` enum with APPROACH, ENTERED_PRE_CENTER, ENTERED_POST_CENTER, CLEARED;
`admissible_transition(phase, witnessed_exit, witnessed_retreat, continuation_safe) -> bool`;
`PassageCommitment.update(actual_state, absolute_time, current_window)` maintains actual gate order;
`PassageCommitment.admit(prediction, maneuver)` authorizes a candidate, not an unchecked state transition.

- [ ] Write these failing invariants:

```python
def test_entry_requires_complete_exit():
    assert not admissible_transition(Phase.APPROACH, False, False, True)

def test_post_center_cannot_substitute_retreat_for_exit():
    assert not admissible_transition(Phase.ENTERED_POST_CENTER, False, True, True)

def test_exit_requires_safe_continuation():
    assert not admissible_transition(Phase.ENTERED_POST_CENTER, True, False, False)
```

- [ ] Add moving-plane sweep-back, reverse center-crossing and finite-body tail tests using `tests/test_passage.py::single_gate` with motion variations.
- [ ] Implement phase decisions from moving-frame signed distances and body envelope; preserve actual historical entry and exit events.
- [ ] Check complete exit/retreat plus continuation only near current gate; retain other frames in actual collision audit and model hard screening until noninteraction has been established.
- [ ] Reject partial-horizon entries. Test approach stays on entry side without a witness; a hover point swept by the moving gate is not admissible.

## Task 5: GPU候选、代价与PX4接入

**Files:** Create `commitment_gpu.py`, `commitment_diagnostic.py`, `commitment_px4.py`, `tests/test_commitment_controller.py`.
Reuse `reference.py` and `scheduler.py` without altering actual-PVAJ and 100 ms conditions.

**Interfaces:** `CommitmentController.select(state, now, progress, reference) -> FeedbackManeuver | None`;
`current_maneuver` remains usable only under task3 revalidation; adapter maintains separate solve and activation clocks.

- [ ] Write failing tests that autonomous mode never reads a forbidden reference and center-after-plane mode never switches to next-gate tracking:

```python
class ForbiddenReference:
    def __getattribute__(self, name):
        raise AssertionError(name)

def test_autonomous_ignores_reference(controller, state, inside_progress):
    controller.select(state, 0., inside_progress, ForbiddenReference())
```

- [ ] Implement 16384 GPU candidates around complete passage/retreat proposals in CTBR coordinates. Cache feedback continuations compete with new candidates; no random-candidate-only fallback.
- [ ] Normal mode minimizes TOGT position/velocity error subject to admission. Autonomous score is exit duration plus `max(distance/vmax, norm(delta_v)/amax)` as an explicitly approximate next-gate travel cost. Do not present it as optimal time-to-go.
- [ ] CPU FP64 checks selected maneuver with identical feedback/model; complete call including second attempts must remain under100 ms, record all missed deadlines.
- [ ] Implement independent coherent-state receiver and feedback sender; activate time-stamped plans at declared times and log lateness. No direct-motor ROS sender can remain active during CTBR runs.
- [ ] Log remaining-domain checks, rejected constraints, candidate counts, mode causes, actual phase, replan generation, result acceptance, predicted exit and actual exit. Logging must distinguish no candidate from stale/exhausted/unsafe cached policy.
- [ ] Run static and moving single-gate tests, including terminal-state reconstruction from `gpu_passage_px4_20260920_05_extended` with timestamp alignment and documented reconstruction uncertainty.

## Task 6: 冻结验证与结果记录

**Files:** Create `commitment_evaluate.py`, `COMMITMENT_RESULTS.md`; update `README.md`, `docs/PROJECT_CONTEXT.md`, `docs/ROADMAP.md`, `docs/DECISIONS.md`, `docs/TASK_LOG.md` only with measured facts.

- [ ] Run all old and new method tests explicitly, preserve original 54-test compatibility; run no unrelated training.
- [ ] Freeze source hashes and configuration before experiments. Driver must refuse overwrite and detect mid-run source changes:

```python
assert not output.exists()
output.mkdir(parents=True)
assert current_hashes == frozen_hashes
```

- [ ] Run static/translation/rotation single-gate tests at four phases with bounded P/V/attitude disturbances. Report observed recovery, not universal guarantees.
- [ ] Run original phase0.4, frequency/translation/rotation scales1, seeds0/1/2, horizon2.5, samples16384, duration60 in nominal mode.
- [ ] Only after no-gate and single-gate PX4 validation passes, run three original-course PX4 trials under identical parameters and60-second race limit. Stop/mark failure on contacts, actuator instability, workspace violation or stale feedback; retain post-failure recovery outside scoring.
- [ ] Run admission-disabled and feedback-disabled ablations in nominal mode only first; do not deliberately fly a known unsafe configuration into Gazebo gates just for a comparison.
- [ ] Report actual full-body exits, return criterion (<0.55 m and speed<1 m/s, matching prior online protocol), frame clearance, dynamics violations, full cycle times, timeouts and every failed trial. Note online protocol is weaker than exact PVAJ return in the parent planning problem.
- [ ] Self-review source and results. Three empirical successes qualify only the tested conditions; no claim of general recursive feasibility or globally infeasible course from a failed solve.

## Handoff

推荐由当前助手在本会话逐项执行，避免多个代理同时改预测模型与执行接口。用户未授权子代理，不派发。
设计已获批准；本计划需用户审阅确认后进入实现。此文档不是已完成代码或试飞结果。
