"""Read-only flight diagnosis; outputs never modify the racing controller.

Stage labels are the latest optimization-cycle labels, not measured events.
Speed and distance use the executed state estimate; safety uses the separate
Gazebo ground-truth audit. Do not interpret stage occupancy as avoidable delay.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import butter, sosfiltfilt

from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.receding_gate_racing.scenario import vary_track
from .engine import Engine


STAGES = ('TRANSIT', 'PREPARE', 'CROSS')
COLORS = dict(TRANSIT='#3478af', PREPARE='#e6a238', CROSS='#329773')


def read_run(path):
    result = json.loads((path / 'result.json').read_text())
    rows = np.load(path / 'execution.npz')['rows']
    # A repeated telemetry timestamp does not describe another flight interval.
    rows = rows[np.r_[np.diff(rows[:, 0]) > 0, True]]
    rows = rows[(rows[:, 0] >= 0) & (rows[:, 0] <= result['last_time_s'])]
    if len(rows) < 2:
        raise ValueError('not enough execution telemetry')
    cycles = result['solves']
    times = np.array([c['time'] for c in cycles])
    index = np.maximum(0, np.searchsorted(times, rows[:-1, 0], side='right') - 1)
    stages = np.array([cycles[i]['stage'] for i in index])
    speed = np.linalg.norm(rows[:, 4:7], axis=1)
    dt = np.diff(rows[:, 0])
    mid_speed = (speed[:-1] + speed[1:]) / 2
    return result, rows, stages, speed, dt, mid_speed


def diagnose(path):
    r, rows, stages, speed, dt, mid_speed = read_run(path)
    cycles = r['solves']
    ends = [c['time'] for c in r['crossings']]
    def group(mask):
        seconds = float(dt[mask].sum())
        return dict(duration_s=seconds,
                    mean_estimated_speed_mps=float(np.sum(dt[mask] * mid_speed[mask]) / seconds) if seconds else None,
                    below_2_mps_s=float(dt[mask & (mid_speed < 2)].sum()))
    phase_stats = {s: group(stages == s) for s in STAGES}
    segments = []
    for i, (a, b) in enumerate(zip([0., *ends], [*ends, r['last_time_s']])):
        mask = (rows[:-1, 0] >= a) & (rows[:-1, 0] < b)
        segments.append(dict(name=f'W{i + 1}' if i < len(ends) else 'Return',
                             start_s=a, end_s=b, event_duration_s=b-a,
                             **group(mask),
                             stages_s={s: float(dt[mask & (stages == s)].sum()) for s in STAGES}))
    # Inspect the existing seed's instantaneous normal-speed preference using
    # exactly its inputs/formula. This is a proposal, not the selected command.
    track, _ = build_track()
    vary_track(track, r['phase'], r['layout'], r['motion_rate'], r['translation_scale'], r['rotation_scale'])
    engine = Engine(track, gpu=False)
    prepare = []
    for j, c in enumerate(cycles):
        if r.get('config', {}).get('control_revision') or c['stage'] != 'PREPARE' or c['gate'] >= len(track.order):
            continue
        now = c['time']
        x = np.array([np.interp(now, rows[:, 0], rows[:, k]) for k in range(1, 18)])
        w = track.windows[track.order[c['gate']]]
        center, rot, *_ = w.state_at(now)
        next_center, *_ = w.state_at(now + .02)
        q = rot.T @ (x[:3] - center)
        relative = rot.T @ (x[3:6] - (next_center-center)/.02)
        lateral = np.linalg.norm(q[:2])
        closing = -np.dot(q[:2], relative[:2]) / max(.01, lateral)
        cfg = engine.config
        slab = cfg.radius + cfg.error_margin + cfg.frame_thickness
        inner = max(.05, float(engine.geometry.frames[track.order[c['gate']]].boundary_distance(np.zeros(2))) - slab - .1)
        remaining = max(0., lateral-inner)
        reach = np.clip((np.sqrt(closing**2 + 5*remaining)-closing)/2.5, .35, 3.5)
        entry = max(0., -slab-engine.signs[c['gate']]*q[2])
        forward = np.clip(entry/(reach+.35), .5, 9.)
        weight = max(0., min(cycles[j+1]['time'] if j+1 < len(cycles) else r['last_time_s'], r['last_time_s'])-now)
        prepare.append((now, entry, lateral, inner, forward, weight))
    prep = np.array(prepare)
    updates = r['reference_updates']
    ms = np.array([c['cycle_ms'] for c in cycles])
    # Evaluate actual telemetry as well as command increments. Normalize
    # fluctuation by duration, so a shorter lap is not automatically smoother.
    uniform_t = np.arange(rows[0, 0], rows[-1, 0], .01)
    rates = np.column_stack([np.interp(uniform_t, rows[:, 0], rows[:, j]) for j in range(11, 14)])
    high = sosfiltfilt(butter(3, 3., fs=100., btype='highpass', output='sos'), rates, axis=0)
    rate_step = np.linalg.norm(np.diff(rows[:, 19:22], axis=0), axis=1)
    smoothness = dict(
        command_bodyrate_step_p95_radps=float(np.quantile(rate_step, .95)),
        command_bodyrate_step_max_radps=float(rate_step.max()),
        command_bodyrate_jumps_gt1_count=int(np.sum(rate_step > 1.)),
        command_bodyrate_jumps_gt1_per_s=float(np.sum(rate_step > 1.)/dt.sum()),
        command_bodyrate_total_variation_per_s=float(rate_step.sum()/dt.sum()),
        measured_roll_pitch_highpass_3hz_rms_radps=float(np.sqrt(np.mean(np.sum(high[:, :2]**2, axis=1)))),
        measured_yaw_highpass_3hz_rms_radps=float(np.sqrt(np.mean(high[:, 2]**2))),
        measured_bodyrate_rms_radps=np.sqrt(np.mean(rates*rates, axis=0)).tolist(),
        definition='CTBR rate-step Euclidean norm; measured rates linearly resampled at 100 Hz, zero-phase third-order 3 Hz Butterworth highpass. Descriptive, not a comfort standard.')
    output = dict(
        run=str(path.resolve()), flight_time_s=r['flight_time_s'], smoothness=smoothness,
        last_gate_exit_s=ends[-1] if ends else None,
        return_time_s=r['last_time_s']-ends[-1] if ends else None,
        telemetry_interval_s=[float(rows[0, 0]), float(rows[-1, 0])],
        estimated_path_length_m=float(np.linalg.norm(np.diff(rows[:, 1:4], axis=0), axis=1).sum()),
        mean_estimated_speed_mps=float(np.sum(dt*mid_speed)/dt.sum()),
        estimated_exit_speeds_mps=[float(np.interp(t, rows[:, 0], speed)) for t in ends],
        phases=phase_stats, segments=segments,
        time_below_speed_s={str(v): float(dt[mid_speed < v].sum()) for v in (1, 2, 4, 6, 8, 10)},
        mppi_cycle_ms=dict(median=float(np.median(ms)), p95=float(np.quantile(ms, .95)), maximum=float(ms.max())),
        mean_optimization_frequency_hz=len(cycles)/(cycles[-1]['time']-cycles[0]['time']),
        reference_reasons=dict(Counter(x['reason'] for x in updates)),
        accepted_references=sum(x['accepted'] for x in updates),
        future_warnings=sum(c['future_warning'] for c in cycles),
        minimum_first_step_safe_samples=min(c['first_step_safe_samples'] for c in cycles),
        sender_interval_ms=dict(median=float(np.median(dt)*1000), p99=float(np.quantile(dt, .99)*1000), maximum=float(dt.max()*1000)),
        legacy_prepare_seed_below_3_mps_s=float(prep[prep[:, 4] < 3, 5].sum()) if len(prep) else None,
        provenance={name: hashlib.sha256((path/name).read_bytes()).hexdigest() for name in ('result.json', 'execution.npz', 'audit.json')},
        evidence='Execution state estimate, latest optimizer stage; ground-truth safety in audit.json. No causal ablation.')
    return output, (r, rows, stages, speed, dt)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    parser.add_argument('--previous', type=Path)
    args = parser.parse_args()
    out, data = diagnose(args.run)
    r, rows, stages, speed, dt = data
    audit = json.loads((args.run/'audit.json').read_text())
    evaluation = json.loads((args.run/'RACING_EVALUATION.json').read_text())
    if args.previous:
        previous, _ = diagnose(args.previous)
        out['previous'] = {k: previous[k] for k in ('run', 'flight_time_s', 'phases', 'segments', 'mean_estimated_speed_mps', 'estimated_path_length_m', 'smoothness')}
        out['flight_time_difference_s'] = out['flight_time_s'] - previous['flight_time_s']
    (args.run/'SPEED_DIAGNOSIS.json').write_text(json.dumps(out, indent=2, ensure_ascii=False)+'\n')
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), constrained_layout=True)
    ax = axes[0]
    boundaries = np.r_[0, np.flatnonzero(stages[1:] != stages[:-1])+1, len(stages)]
    used = set()
    for a, b in zip(boundaries[:-1], boundaries[1:]):
        stage = stages[a]
        ax.axvspan(rows[a, 0], rows[b, 0], color=COLORS[stage], alpha=.19, lw=0,
                   label=stage.title() if stage not in used else None)
        used.add(stage)
    ax.plot(rows[:, 0], speed, color='#163348', lw=1.2, label='Executed estimated speed')
    ax.axhline(12, color='#c44', ls='--', lw=.9, label='12 m/s bound')
    for crossing in r['crossings']:
        ax.axvline(crossing['time'], color='.4', lw=.65)
        ax.text(crossing['time'], 12.4, f"W{crossing['gate']+1}", ha='center', fontsize=9)
    ax.set(xlabel='Flight time (s)', ylabel='Speed (m/s)', ylim=(0, 13.1), xlim=(0, r['last_time_s']))
    fig.suptitle(f"Actual flight: 7 gates + return in {r['last_time_s']:.3f} s")
    ax.legend(loc='lower left', bbox_to_anchor=(0, 1.01), ncol=5, fontsize=9)
    ax = axes[1]
    names = [x['name'] for x in out['segments']]
    bottom = np.zeros(len(names))
    for stage in STAGES:
        height = np.array([s['stages_s'][stage] for s in out['segments']])
        ax.bar(names, height, bottom=bottom, color=COLORS[stage], label=stage.title())
        bottom += height
    for i, segment in enumerate(out['segments']):
        ax.text(i, bottom[i]+.06, f"{segment['event_duration_s']:.2f} s", ha='center', fontsize=9)
    ax.set(ylabel='Time in segment (s)', ylim=(0, max(bottom)+.65),
           title='Segments end after the whole drone exits the gate; stage time is not wasted time')
    ax.legend(ncol=3, loc='upper left', fontsize=9)
    fig.savefig(args.run/'speed_breakdown.png', dpi=170, bbox_inches='tight')
    plt.close(fig)
    lines = ['# 七窗试飞与速度诊断', '',
             f"窗口条件：频率{r['motion_rate']:g}×、平移幅度{r['translation_scale']:g}×、旋转幅度{r['rotation_scale']:g}×、相位{r['phase']:.6g} rad。",
             f"本次七窗返航 {r['flight_time_s']:.3f} s；严格采样/实时验收 {evaluation['realtime_success']}，零接触 {not r['collision_detected']}。",
             f"最小真值采样机体净空 {audit['min_sampled_body_clearance_m']:.3f} m；最高真值速度 {audit['max_sampled_body_speed_mps']:.3f} m/s。",
             f"执行状态估计平均速度 {out['mean_estimated_speed_mps']:.3f} m/s，轨迹长 {out['estimated_path_length_m']:.2f} m。",
             f"第7门整机退出 {out['last_gate_exit_s']:.3f} s；之后返航与收敛 {out['return_time_s']:.3f} s。", '',
             '| 阶段 | 占用时间（s） | 平均估计速度（m/s） |', '|---|---:|---:|']
    for s in STAGES:
        stats = out['phases'][s]
        lines.append(f"| {s} | {stats['duration_s']:.3f} | {stats['mean_estimated_speed_mps']:.3f} |")
    lines += ['', '阶段来自最近一次优化周期，交接存在约一个优化周期的时间误差；首尾未覆盖的少量遥测时间不补造。',
              'PREPARE包含实际接近门的路程，不能把阶段占用全部称为停车、等待或可节省时间。', '']
    if r.get('config', {}).get('control_revision'):
        lines += ['## 本版机制与控制平顺性', '',
                  '准备候选按当前和预计到达时刻的开口对准程度释放近门减速；预测中的gate/phase决定准备或穿窗目标。',
                  '出门后的下一门/返航方向使用过门前后共同的末端价值，避免过门后突然新增代价导致停滞。',
                  '采样扰动按100 ms节点线性插值；角速度输入变化和实际角加速度进入代价，逐次输入变化预算可由安全需要放宽。',
                  '未放宽真实门框、机体和动力学硬限制；较新修订纠正了远场无限平面的进度误判，具体以源码快照为准。',
                  '各组件尚无独立消融，整体改动的单次结果不能拆分因果贡献。', '']
    else:
        lines += ['## 证据支持的慢因与改进方向', '',
              '1. 准备种子即使已对准开口仍按入口距离减速：reach最少0.35 s，再加0.35 s，',
              '   因此对准且横向闭合时 forward=clip(d/0.7,0.5,9)。距入口2 m/1 m分别只有2.86/1.43 m/s。',
              '   这是候选引导偏好，不是最终动作硬限速。应在预测可安全通过时保留穿越速度，仅在横向调整确有需要时减速。',
              '2. PREPARE/CROSS通过当前门后不再评价后续位置/速度进度，仍有安全、平顺和yaw代价。',
              '   因此“过门后向哪飞、保留多少速度”缺少竞速目标。应加接续TOGT/下一门的出口状态价值，仍由MPPI和输出过滤决定动作。',
              '3. 一次rollout的mode固定为当前真实阶段；预测中进入CROSS后，PREPARE候选仍按准备代价计分。',
              '   应根据预测的gate/ph推进代价阶段，使准备、穿越、过门后接续在同一候选内连贯。', '']
    lines += [f"本次共接入 {out['accepted_references']} 条TOGT，触发原因 {out['reference_reasons']}。异步求解不能简单累加成飞行等待时间。",
              '   重复避让/跟踪失配可能造成参考和yaw切线频繁变化；需要记录参考误差与接入连续性再作独立对照。',
              f"MPPI中位/最大 {out['mppi_cycle_ms']['median']:.2f}/{out['mppi_cycle_ms']['maximum']:.2f} ms，平均约 {out['mean_optimization_frequency_hz']:.1f} Hz。",
              f"最少首拍合格样本 {out['minimum_first_step_safe_samples']} / {r['config']['samples']}，发送端候选搜索回退 {evaluation['sender_fallbacks']} 次。",
              f"角速度指令相邻跳变95分位 {out['smoothness']['command_bodyrate_step_p95_radps']:.3f} rad/s；>1 rad/s跳变 {out['smoothness']['command_bodyrate_jumps_gt1_count']} 次。",
              f"最大单次指令跳变 {out['smoothness']['command_bodyrate_step_max_radps']:.3f} rad/s；典型波动减少不代表极端跳变也减少。",
              f"实测roll/pitch角速度3 Hz高通合成RMS {out['smoothness']['measured_roll_pitch_highpass_3hz_rms_radps']:.4f} rad/s。", '',
              '## 对照和边界', '']
    if args.previous:
        lines += [f"对照运行 {out['previous']['flight_time_s']:.3f} s；本次差值 {out['flight_time_difference_s']:+.3f} s。",
                  f"估计轨迹长度 {out['previous']['estimated_path_length_m']:.2f} → {out['estimated_path_length_m']:.2f} m，平均速度 {out['previous']['mean_estimated_speed_mps']:.3f} → {out['mean_estimated_speed_mps']:.3f} m/s。",
                  f"对照指令跳变95分位 {out['previous']['smoothness']['command_bodyrate_step_p95_radps']:.3f} rad/s；对照实测roll/pitch高频RMS {out['previous']['smoothness']['measured_roll_pitch_highpass_3hz_rms_radps']:.4f} rad/s。",
                  '两个版本各一次，随机采样与运行时序不同，不据此证明总体优势或单一组件收益。']
    lines += ['弯道减速部分来自原物理限值；不能直接把路程除以12 m/s当可实现圈速。',
              '本次是固定条件有限采样证据，不是连续安全保证或全局最优证明。',
              '未保存每周期所采用的完整参考及yaw目标，不能从本日志计算真实yaw相对规划切线的误差；实际飞行方向不代替规划切线。',
              '控制器版本以每次结果的source_snapshot和哈希为准。', '',
              '原始证据：result.json、execution.npz、flight.ulg、audit.json、RACING_EVALUATION.json；哈希见SPEED_DIAGNOSIS.json。', '']
    (args.run/'SPEED_DIAGNOSIS.md').write_text('\n'.join(lines))
    print(json.dumps(out, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
