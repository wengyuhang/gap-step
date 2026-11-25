"""Read-only historical baseline comparison; write only into the new batch."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
BASE = ROOT/'convex_timevarying_window'
SOURCES = [
    ('center24', 'Checked', 'comparisons/amplitude_racing/results/center_motion_checked_20260923/summary.json'),
    ('center24', 'Wang 2023 independent', 'comparisons/paper_baselines_2026/results/center_motion_wang2023_20260923/summary.json'),
    ('center24', 'Direct-CTBR MPPI', 'comparisons/direct_ctbr_center_motion/results/direct_ctbr_20260924/summary.json'),
    ('amplitude16', 'Checked', 'continuation_racing/results/frozen_checked_20260919/summary.json'),
    ('amplitude_partial', 'Continuation unchecked', 'continuation_racing/results/frozen_v4_20260919/summary.json'),
    ('amplitude16', 'Local-sweep', 'comparisons/amplitude_racing/results/frozen_20260919/summary.json'),
    ('amplitude16', 'Wang 2023 independent', 'comparisons/paper_baselines_2026/results/two_papers_20260919/polynomial_matrix/summary.json'),
    ('amplitude16', 'Document V3 MPPI', 'comparisons/paper_baselines_2026/results/v3_px4_frozen_20260919_02/summary.json'),
    ('rate12', 'Local-sweep', 'comparisons/motion_rate_racing/results/local_sweep_frozen_20260918/summary.json'),
    ('rate12', 'Worst-case crossing', 'comparisons/motion_rate_racing/results/frozen_20260918/summary.json'),
    ('rate_partial', 'GP-MPPI independent', 'comparisons/motion_rate_racing/results/gp_frozen_20260918/summary.json'),
]


def read(path):
    return json.loads(Path(path).read_text())


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def key(row):
    if 'phase' not in row or ('rate' not in row and 'motion_rate' not in row):
        run = Path(row['run'])
        result = read((run if run.is_absolute() else ROOT/run)/'result.json')
        row = dict(result, **row)
    return tuple(round(float(v), 9) for v in (row.get('rate', row.get('motion_rate')),
                 row.get('translation_scale', 1.), row.get('rotation_scale', 1.), row['phase'], row.get('layout', 0.)))


def selected(rows):
    """Drop infrastructure-only attempts, never select by performance."""
    groups = {}
    for row in rows:
        groups.setdefault(key(row), []).append(row)
    result = []
    for attempts in groups.values():
        valid = [r for r in attempts if not r.get('infrastructure_failure', False)]
        if len(valid) > 1:
            raise ValueError('Multiple algorithm attempts for one frozen condition')
        result.append(valid[0] if valid else attempts[-1])
    return result


def maximum_cycle(row):
    if row.get('cycle_ms'):
        return row['cycle_ms'].get('max')
    values = [row[k] for k in ('mppi_max_ms', 'togt_max_ms') if row.get(k) is not None]
    return max(values) if values else None


def failure_reasons(row):
    reasons = []
    if row.get('infrastructure_failure'):
        reasons.append('基础设施失败')
    if row.get('analysis_error'):
        reasons.append('验收证据缺失')
    if row.get('flight_time_s') is None:
        reasons.append('未完成七窗返航')
    if row.get('collision'):
        reasons.append('Gazebo接触')
    if row.get('room_passed') is False:
        reasons.append('工作区超界')
    if row.get('min_clearance_m') is not None and row['min_clearance_m'] < 0:
        reasons.append('真值采样整机净空为负')
    if maximum_cycle(row) is not None and maximum_cycle(row) >= 100:
        reasons.append('完整周期超100ms')
    if not row.get('realtime_success') and not reasons:
        reasons.append('见原始顺序/终点审计')
    return reasons


def stats(rows):
    passed = [r for r in rows if r.get('realtime_success', False)]
    timings = [maximum_cycle(r) for r in rows if maximum_cycle(r) is not None]
    clearances = [r['min_clearance_m'] for r in passed if r.get('min_clearance_m') is not None]
    return dict(n=len(rows), physical_success=sum(bool(r.get('physical_success')) for r in rows),
                realtime_success=len(passed), incomplete_evidence=sum('analysis_error' in r for r in rows),
                infrastructure_failures=sum(bool(r.get('infrastructure_failure')) for r in rows),
                median_success_time_s=float(np.median([r['flight_time_s'] for r in passed])) if passed else None,
                median_success_clearance_m=float(np.median(clearances)) if clearances else None,
                worst_cycle_ms=max(timings) if timings else None)


def paired(new, old):
    before = {key(r): r for r in old}
    pairs = [(r, before[key(r)]) for r in new if key(r) in before]
    successful = [(a, b) for a, b in pairs if a.get('realtime_success') and b.get('realtime_success')]
    delta = [a['flight_time_s']-b['flight_time_s'] for a, b in successful]
    clearance_delta = [a['min_clearance_m']-b['min_clearance_m'] for a, b in successful
                       if a.get('min_clearance_m') is not None and b.get('min_clearance_m') is not None]
    return dict(matched_conditions=len(pairs), common_successes=len(successful),
                new_faster=sum(d < -1e-6 for d in delta), old_faster=sum(d > 1e-6 for d in delta),
                median_new_minus_old_s=float(np.median(delta)) if delta else None,
                median_clearance_delta_m=float(np.median(clearance_delta)) if clearance_delta else None,
                new_only_success=sum(bool(a.get('realtime_success')) and not b.get('realtime_success') for a, b in pairs),
                old_only_success=sum(bool(b.get('realtime_success')) and not a.get('realtime_success') for a, b in pairs),
                pairs=[dict(condition=key(a), new_success=bool(a.get('realtime_success')),
                            old_success=bool(b.get('realtime_success')),
                            delta_s=a['flight_time_s']-b['flight_time_s'] if a.get('realtime_success') and b.get('realtime_success') else None,
                            new_run=a['run'], old_run=b['run']) for a, b in pairs])


def format_value(v, precision=3):
    return '—' if v is None else f'{v:.{precision}f}'


def main():
    p = argparse.ArgumentParser()
    p.add_argument('batch', type=Path)
    p.add_argument('--dynamics', action='store_true', help='ULog audit; run after timed flights, not during them')
    p.add_argument('--adjudicated', action='store_true', help='Use separately documented setup-failure adjudication, retaining raw statistics')
    p.add_argument('--supplement', type=Path, help='Separate contemporary controls and sparse RL-setting matches')
    a = p.parse_args()
    batch = a.batch.resolve()
    raw = selected(read(batch/'summary.json'))
    new = selected(read(batch/('adjudicated_summary.json' if a.adjudicated else 'summary.json')))
    new_by_key = {key(r): r for r in new}
    supplement = selected(read(a.supplement/'summary.json')) if a.supplement else []
    supplement_soft = {key(r): r for r in supplement if r['method']=='Checked-Soft'}
    comparisons, sources, matrices = [], [], {}
    historical_rows = []
    for matrix, name, relative in SOURCES:
        path = BASE/relative
        old = selected(read(path))
        matching = [new_by_key[key(r)] for r in old if key(r) in new_by_key]
        comparison = dict(matrix=matrix, method=name, source=str(path), source_sha256=digest(path),
                          old=stats(old), new_on_same_conditions=stats(matching), paired=paired(matching, old))
        comparisons.append(comparison)
        sources.append(dict(path=str(path), sha256=digest(path)))
        historical_rows.extend(dict(r, matrix=matrix, method=name) for r in old)
        if matrix not in matrices:
            matrices[matrix] = [key(r) for r in old]
    manifest_path = batch/'baseline_manifest.json'
    if manifest_path.exists() and read(manifest_path) != sources:
        raise RuntimeError('Historical baseline summaries changed since the first report')
    if not manifest_path.exists():
        manifest_path.write_text(json.dumps(sources, indent=2)+'\n')

    # All sparse CTBR development variants remain individually visible, with
    # failures retained. They are not pooled into a cherry-picked "best" method.
    sparse = []
    for path in sorted((BASE/'gs_mppi/direct_ctbr/results').glob('*/result.json')):
        r = read(path)
        if r.get('motion_rate') is None or key(r) not in new_by_key:
            continue
        verdict = path.parent/'RACING_EVALUATION.json'
        v = read(verdict) if verdict.exists() else {}
        n = new_by_key[key(r)]
        sparse.append(dict(method=path.parent.name, condition=key(r), run=str(path.parent),
                           result_sha256=digest(path), completed=r.get('completed'),
                           failure=r.get('failure'), historical_realtime_success=v.get('realtime_success'),
                           historical_valid_time_s=v.get('race_time_s'), new_success=n.get('realtime_success'),
                           new_valid_time_s=n.get('flight_time_s') if n.get('realtime_success') else None,
                           verdict_sha256=digest(verdict) if verdict.exists() else None,
                           note='sparse development; some GUI-on runs; not a frozen matrix'))

    preliminary = []
    for relative in ('three_phase_mppi/results/frozen_px4_20260923/summary.json',
                     'three_phase_mppi/results/frozen_v3_px4_20260923/summary.json'):
        path = BASE/relative
        for row in read(path):
            n = new_by_key.get(key(row))
            preliminary.append(dict(method=path.parent.name, source=str(path), source_sha256=digest(path),
                                    condition=key(row), completed=row['completed'], failure=row['failure'],
                                    gates=row['gates'], new_success=n.get('realtime_success') if n else None,
                                    note='three-phase independent implementation; original audit in source'))
    development = []
    development_run = BASE/'checked_soft_racing/results/development_soft_r1_t1_a1_p04_20261006'
    dev_audit = read(development_run/'checked_soft_audit.json')
    for relative in ('comparisons/paper_baselines_2026/results/two_papers_20260919/rl_development/summary.json',
                     'comparisons/paper_baselines_2026/results/two_papers_20260919/rl_static_check/summary.json'):
        path = BASE/relative
        for row in read(path):
            matching = key(row) == (1., 1., 1., .4, 0.)
            development.append(dict(method='State RL 2026 independent transfer', source=str(path),
                                    source_sha256=digest(path), condition=key(row), completed=row['completed'],
                                    failure=row['failure'], gates=row['gates'],
                                    new_development_result=dev_audit if matching else supplement_soft.get(key(row)),
                                    note='failed PX4 transfer; not author performance; outside formal matrix'))
    for path in sorted((ROOT/'复现/waypoint_mppi/results').glob('px4*/result.json')):
        row = read(path)
        matching = row.get('motion_rate') is not None and key(row) == (1., 1., 1., .4, 0.)
        development.append(dict(method='Waypoint MPPI independent: '+path.parent.name,
                                source=str(path), source_sha256=digest(path), completed=row.get('completed'),
                                failure=row.get('failure'), gates=len(row.get('crossings', [])),
                                condition=key(row) if row.get('motion_rate') is not None else None,
                                new_development_result=dev_audit if matching else None,
                                note='independent dynamic/PX4 adaptation; not official paper performance'))

    dynamics = []
    if a.dynamics:
        from convex_timevarying_window.gs_mppi.direct_ctbr.compare_targets import body_dynamics
        # Audit identical ground-truth definitions for all new cases and the
        # three primary baselines. Never write a file into a baseline directory.
        inputs = [dict(r, method='Checked-Soft', matrix='union38') for r in new]
        inputs += [r for r in historical_rows if r['matrix'] == 'center24']
        for i, r in enumerate(inputs):
            path = Path(r['run'])
            if not path.is_absolute():
                path = ROOT/path
            item = dict(method=r['method'], condition=key(r), run=str(path), historical_success=bool(r.get('realtime_success')))
            try:
                result = read(path/'result.json')
                item.update(body_dynamics(path, result['last_time_s']))
                item['result_sha256'] = digest(path/'result.json')
                item['strict_dynamics_and_primary_pass'] = item['historical_success'] and item['satisfies_direct_dynamics_limits']
            except Exception as exc:
                item['error'] = repr(exc)
            dynamics.append(item)
            print(f'DYNAMICS {i+1}/{len(inputs)} {r["method"]}', flush=True)
        (batch/'common_dynamics.json').write_text(json.dumps(dynamics, indent=2)+'\n')
    elif (batch/'common_dynamics.json').exists():
        dynamics = read(batch/'common_dynamics.json')

    output = dict(new_stats=stats(new), raw_first_batch_stats=stats(raw), using_setup_adjudication=a.adjudicated,
                  setup_adjudication=read(batch/'adjudication.json') if a.adjudicated else None,
                  comparisons=comparisons, source_manifest=sources,
                  supplement=supplement,
                  sparse_ctbr=sparse, three_phase=preliminary, independent_transfer_diagnostics=development,
                  common_dynamics=dynamics, conditions_completed=len(new), conditions_planned=38)
    (batch/'comparison.json').write_text(json.dumps(output, indent=2, ensure_ascii=False)+'\n')
    lines = ['# Checked-Soft：同设置历史对比', '',
             f'本次完成 {len(new)}/38 条件；每条件一个固定参数有效尝试。历史结果只读，未重新运行旧方法。',
             '新增项偏好机体净空 0.50 m，原 0.35 m 硬余量和 checked 准入不变。',
             '主验收为七窗返航、零接触、真值采样净空/房间检查与完整周期 <100 ms；不是连续安全认证。', '',
             '## 主矩阵与扩展矩阵', '',
             '同一矩阵内同条件可比；不同矩阵不能直接比较成功率。partial 表示历史批次未完成完整矩阵，仅对已记录条件比较。有效圈时只统计实时合格圈，失败保留在分母。', '',
             '| 矩阵 | 方法 | 实时合格/条件 | 合格圈时中位数 s | 合格净空中位数 m | 最大完整周期 ms |',
             '|---|---|---:|---:|---:|---:|']
    if a.adjudicated:
        lines[3:3] = [f'原始批次主验收 {stats(raw)["realtime_success"]}/{len(raw)}，原始 summary.json 保留不改。',
                      '下表采用单列的初始化异常判定和协议允许的一次同代码重试。',
                      '判据基于规划前 t=10..29 s：机体真值静止、已解锁 Offboard、有爬升指令但距要求起点 >2 m；不重试任何正常起飞后的算法失败。',
                      '证据见 adjudication.json；adjudicated_summary.json 同时保存原尝试与重试。', '']
    for matrix, keys in matrices.items():
        subset = [new_by_key[k] for k in keys if k in new_by_key]
        entries = [('Checked-Soft (new)', stats(subset))] + [(c['method'], c['old']) for c in comparisons if c['matrix'] == matrix]
        for name, s in entries:
            lines.append(f'| {matrix} | {name} | {s["realtime_success"]}/{s["n"]} | {format_value(s["median_success_time_s"])} | {format_value(s["median_success_clearance_m"])} | {format_value(s["worst_cycle_ms"])} |')
    lines += ['', '注意：原幅度矩阵的 Checked 15/16 包含一条缺真值日志，不能说那一条已证明碰撞；',
              'Continuation unchecked 只运行了部分矩阵，且含基础设施失败。表中是“证据通过数”，不是纯碰撞概率。', '',
              '## 同条件、双方均合格的圈时差', '',
              '负数表示新方法快。必须结合前表失败数看，不能仅用成功交集宣称整体更优。', '',
              '| 矩阵 | 旧方法 | 双方合格数 | 新方法更快数 | 新减旧中位数 s | 净空差中位数 m | 仅新通过 / 仅旧通过 |',
              '|---|---|---:|---:|---:|---:|---:|']
    for c in comparisons:
        p = c['paired']
        lines.append(f'| {c["matrix"]} | {c["method"]} | {p["common_successes"]} | {p["new_faster"]} | {format_value(p["median_new_minus_old_s"])} | {format_value(p["median_clearance_delta_m"])} | {p["new_only_success"]} / {p["old_only_success"]} |')
    lines += ['', '## 本次负结果', '']
    failed = [r for r in new if not r.get('realtime_success')]
    if not failed:
        lines.append('已完成条件中没有主口径失败；仍不等于总体成功概率为 100%。')
    for r in failed:
        lines.append(f'- r/t/a/phase={key(r)[:4]}：{"；".join(failure_reasons(r))}；gates={r.get("gates")}，failure={r.get("failure")}，净空={r.get("min_clearance_m")}，周期={maximum_cycle(r)} ms，审计错误={r.get("analysis_error")}。')
    lines += ['', '## 共同真值动力学复核（独立于历史主验收）', '',
              '统一使用原始约 20 ms 真值网格、姿态旋转的 0.24 m 机体偏移、数值差分；界为 12 m/s、8 m/s²、42°。',
              '该诊断含差分误差，不是连续认证；原主口径没有强制实际飞行满足这些名义界。', '']
    for name in sorted({r['method'] for r in dynamics}):
        group = [r for r in dynamics if r['method'] == name]
        lines.append(f'- {name}：主验收且三项实际动力学均通过 {sum(bool(r.get("strict_dynamics_and_primary_pass")) for r in group)}/{len(group)}；缺数据 {sum("error" in r for r in group)}。')
    lines += ['', '## 同期旧Checked控制组与额外迁移条件', '',
              '以下是单独的小样本诊断，不并入38条件，不挑选替代正式结果。', '',
              '| 运行 | 主验收 | 有效圈时 s | 净空 m |',
              '|---|---|---:|---:|']
    for r in supplement:
        lines.append(f'| {r["name"]} | {r.get("realtime_success")} | {format_value(r.get("flight_time_s") if r.get("realtime_success") else None)} | {format_value(r.get("min_clearance_m"))} |')
    lines += ['', '## 其他已比较方法与证据边界', '',
              '- 三阶段 MPPI 的两批四条件 PX4 结果逐条保存在 comparison.json；原失败未删除。',
              '- 后续 CTBR/safety-filter/flow-smooth/coupled 开发变体逐条列在 SPARSE_BASELINES.md，包括失败和重试，不挑最快版本代表整类方法。',
              '- 2026 状态 RL 独立迁移与 waypoint MPPI 独立实现的已有 PX4 诊断列在 comparison.json。相位 0.4、r/t/a=1/1/1 可对照本次开发试飞；其他条件未冒充正式配对。学习方法尚未完成同任务迁移，不作为作者官方性能。',
              '- 早期 GS-MPPI/三阶段名义模型与离线 TOGT 的名义时间不是 x500/PX4 有效圈时，不放入正式圈时排行。',
              '- 旧 checked/新方法使用相同 PVA 与预览；direct CTBR 直接输出推力/角速度。它们是系统比较，不是只改变目标函数的消融。',
              '- 历史跨日期单次测试受截止时间与负载影响；一两十分之一秒差异不能推断统计显著。', '',
              '原始来源路径及 SHA-256、所有配对和负结果见 comparison.json。', '']
    (batch/'REPORT.md').write_text('\n'.join(lines))
    sparse_lines = ['# 稀疏 CTBR 开发变体：不合并挑选最佳', '',
                    '只列与新 38 条件精确匹配的结果。未知表示缺验收文件，不解释为通过。GUI/开发调参差异见原目录。', '',
                    '| 旧变体 | r/t/a/phase | 旧主验收 | 旧有效圈时 s | 新有效圈时 s | 旧失败 |',
                    '|---|---|---|---:|---:|---|']
    for r in sparse:
        sparse_lines.append(f'| {r["method"]} | {r["condition"][:4]} | {r["historical_realtime_success"]} | {format_value(r["historical_valid_time_s"])} | {format_value(r["new_valid_time_s"])} | {r["failure"]} |')
    (batch/'SPARSE_BASELINES.md').write_text('\n'.join(sparse_lines)+'\n')
    settings_audit(batch, new_by_key, historical_rows)
    plot(batch, new, historical_rows)
    tracking_diagnostics(batch, new, historical_rows)
    print(json.dumps(dict(new=stats(new), comparisons=[{k:v for k,v in c.items() if k in ('matrix','method','new_on_same_conditions')} for c in comparisons]), indent=2))


def plot(batch, new, history):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    names = ['Checked-Soft', 'Checked', 'Wang 2023 independent', 'Direct-CTBR MPPI']
    labels = ['Checked-Soft', 'Checked', 'Wang (indep.)', 'CTBR MPPI']
    colors = ['#df6d2d', '#236192', '#4a987a', '#835b9c']
    ref_keys = {key(r) for r in history if r['matrix'] == 'center24'}
    groups = [[r for r in new if key(r) in ref_keys]] + [[r for r in history if r['matrix']=='center24' and r['method']==name] for name in names[1:]]
    fig, axes = plt.subplots(1, 3, figsize=(12.6, 4), constrained_layout=True)
    for i, rows in enumerate(groups):
        passed = [r for r in rows if r.get('realtime_success')]
        axes[0].bar(i, len(passed), color=colors[i])
        axes[0].text(i, len(passed)+.3, f'{len(passed)}/{len(rows)}', ha='center', fontsize=9)
        axes[1].scatter(np.full(len(passed), i), [r['flight_time_s'] for r in passed], color=colors[i], alpha=.65, s=24)
        axes[2].scatter(np.full(len(passed), i), [r['min_clearance_m'] for r in passed], color=colors[i], alpha=.65, s=24)
    axes[0].set_ylim(0, 27)
    for ax, title in zip(axes, ['Accepted runs (24 center-motion cases)', 'Accepted lap times [s]', 'Accepted minimum sampled clearance [m]']):
        ax.set_title(title, fontsize=10)
        ax.set_xticks(range(4), labels, rotation=25, ha='right')
        ax.grid(axis='y', alpha=.2)
    fig.savefig(batch/'center24_overview.png', dpi=180)
    plt.close(fig)
    # Fixed examples, no selection of a best lap: low/high frequency, t=1/4.
    fig, axes = plt.subplots(2, 2, figsize=(11, 6.5), constrained_layout=True)
    for ax, (rate, translation) in zip(axes.flat, [(1, 1), (1, 4), (10, 1), (10, 4)]):
        for i, rows in enumerate(groups):
            matches = [r for r in rows if key(r)==(float(rate), float(translation), 1., 0., 0.)]
            if not matches:
                continue
            r = matches[0]
            path = Path(r['run'])/'telemetry.csv'
            if not path.exists():
                continue
            data = np.genfromtxt(path, delimiter=',', names=True)
            if not {'t','vx','vy','vz'}.issubset(data.dtype.names):
                continue
            speed = np.sqrt(data['vx']**2+data['vy']**2+data['vz']**2)
            ax.plot(data['t'], speed, label=labels[i]+(' [FAIL]' if not r.get('realtime_success') else ''), color=colors[i], linewidth=1.1)
        ax.set_title(f'r={rate}, translation={translation}, rotation=1, phase=0')
        ax.set_xlabel('Race time [s]'); ax.set_ylabel('Estimated speed [m/s]'); ax.grid(alpha=.2)
    axes[0, 0].legend(fontsize=8)
    fig.savefig(batch/'fixed_case_speeds.png', dpi=180)
    plt.close(fig)
    # Local aperture coordinates remove the known translation/rotation while
    # retaining each method's actual estimated centre-plane crossing position.
    from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
    track, _ = build_track()
    target = (10., 4., 1., round(2.0943951023931953, 9), 0.)
    fig, axes = plt.subplots(2, 4, figsize=(12, 6.5), constrained_layout=True)
    for i, window in enumerate(track.windows):
        ax = axes.flat[i]
        ap = window.aperture
        if ap.kind == 'circle':
            angles = np.linspace(0., 2*np.pi, 160)
            boundary = ap.radius*np.column_stack([np.cos(angles), np.sin(angles)])
        else:
            boundary = np.vstack([ap.physical_vertices, ap.physical_vertices[0]])
        ax.plot(boundary[:,0], boundary[:,1], 'k-', lw=1.)
        for j, rows in enumerate(groups[:3]):
            matches = [r for r in rows if key(r)==target]
            if not matches:
                continue
            r = matches[0]
            result = read(Path(r['run'])/'result.json')
            events = {event['gate']: event for event in result.get('crossings', [])}
            if i in events and 'q' in events[i]:
                q = events[i]['q']
                ax.scatter(q[0], q[1], color=colors[j], s=38, label=labels[j]+(' [FAIL]' if not r.get('realtime_success') else ''))
        ax.set_aspect('equal'); ax.set_title(f'Gate {i+1}'); ax.grid(alpha=.2)
        ax.set_xlabel('Local q1 [m]'); ax.set_ylabel('Local q2 [m]')
    axes.flat[0].legend(fontsize=7)
    axes.flat[7].axis('off')
    axes.flat[7].text(0., .8, 'Fixed hard case: r10 / t4 / a1 / 120 deg\nEstimated centre-plane events.\nA point inside the aperture does NOT\ncertify whole-body safety.', fontsize=10, va='top')
    fig.savefig(batch/'hard_case_crossing_positions.png', dpi=180)
    plt.close(fig)


def tracking_diagnostics(batch, new, history):
    rows = []
    inputs = [('Checked-Soft', r) for r in new]
    inputs += [('Checked', r) for r in history if r['matrix']=='center24' and r['method']=='Checked']
    for name, r in inputs:
        path = Path(r['run'])
        if not (path/'telemetry.csv').exists() or not (path/'result.json').exists():
            continue
        data = np.genfromtxt(path/'telemetry.csv', delimiter=',', names=True)
        result = read(path/'result.json')
        if data.size < 2:
            continue
        time_steps = np.diff(data['t'])
        error = data['error']
        gate_errors = []
        for event in result.get('crossings', []):
            mask = np.abs(data['t']-event['time']) < .5
            gate_errors.append(dict(gate=event['gate'], crossing_time=event['time'],
                                    crossing_error_m=float(np.interp(event['time'], data['t'], error)),
                                    near_crossing_max_error_m=float(error[mask].max())))
        solves = result.get('solves', [])
        rows.append(dict(method=name, condition=key(r), run=str(path),
                         realtime_success=bool(r.get('realtime_success')), min_clearance_m=r.get('min_clearance_m'),
                         tracking_error_max_m=float(error.max()), tracking_error_p99_m=float(np.percentile(error,99)),
                         time_over_nominal_margin_s=float(np.sum(time_steps*(error[:-1]>.35))),
                         slow_time_below_2mps_s=float(np.sum(time_steps*(np.sqrt(data['vx'][:-1]**2+data['vy'][:-1]**2+data['vz'][:-1]**2)<2))),
                         accepted_replans=sum(bool(s.get('accepted')) for s in solves), total_replans=len(solves),
                         statuses=dict(Counter(s.get('status') for s in solves)),
                         per_gate_reference_errors=gate_errors))
    (batch/'tracking_diagnostics.json').write_text(json.dumps(rows, indent=2)+'\n')


def settings_audit(batch, new_by_key, history):
    rows = []
    for r in history:
        if key(r) not in new_by_key:
            continue
        old_path, new_path = Path(r['run']), Path(new_by_key[key(r)]['run'])
        if not old_path.is_absolute():
            old_path = ROOT/old_path
        if not (old_path/'result.json').exists() or not (new_path/'result.json').exists():
            continue
        old, new = read(old_path/'result.json'), read(new_path/'result.json')
        old_params, new_params = old.get('parameters', {}), new.get('parameters', {})
        common = set(old_params) & set(new_params)
        parameter_differences = {k: [old_params[k], new_params[k]] for k in common if old_params[k] != new_params[k]}
        old_world, new_world = old_path/'world.sdf', new_path/'world.sdf'
        old_cfg, new_cfg = old.get('config', {}), new.get('config', {})
        cfg_differences = {k: [old_cfg.get(k), new_cfg.get(k)] for k in set(old_cfg)|set(new_cfg) if old_cfg.get(k) != new_cfg.get(k)}
        rows.append(dict(method=r['method'], matrix=r['matrix'], condition=key(r),
                         old_run=str(old_path), new_run=str(new_path),
                         old_result_sha256=digest(old_path/'result.json'), new_result_sha256=digest(new_path/'result.json'),
                         identical_saved_world=(digest(old_world)==digest(new_world)) if old_world.exists() and new_world.exists() else None,
                         common_px4_parameter_differences=parameter_differences,
                         old_only_px4_parameter_keys=sorted(set(old_params)-set(new_params)),
                         new_only_px4_parameter_keys=sorted(set(new_params)-set(old_params)),
                         algorithm_config_differences=cfg_differences))
    (batch/'settings_audit.json').write_text(json.dumps(rows, indent=2)+'\n')


if __name__ == '__main__':
    main()
