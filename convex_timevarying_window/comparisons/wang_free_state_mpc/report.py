"""Compare all matched historical methods without changing their evidence."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np

from convex_timevarying_window.comparisons.checked_soft_racing.report import (
    ROOT, BASE, SOURCES, key, selected, stats, paired, format_value as fmt)
from convex_timevarying_window.comparisons.checked_soft_mpc_optimized.report import groups
from convex_timevarying_window.comparisons.checked_soft_racing.run import save
from convex_timevarying_window.wang_free_state_mpc.passage_audit import audit_run

EXTRA = [
    ('union38', 'Checked-Soft / optimized MPC', 'comparisons/checked_soft_mpc_optimized/results/frozen_20261006/condition_summary.json'),
    ('soft_partial', 'Checked-Soft / PVA', 'comparisons/checked_soft_racing/results/frozen_20261006/summary.json'),
    ('mpc_partial', 'MPC before optimization', 'comparisons/checked_soft_mpc_racing/results/frozen_20261006/condition_summary.json'),
    ('development2', 'TOGT-MINCO-s4 / MPC', 'togt_minco_mpc/results/px4_v3_lbfgs_20261007/summary.json'),
    ('development2', 'Wang free-state v1 / MPC', 'wang_free_state_mpc/results/free_v1_20261007/summary.json'),
    ('development2', 'Wang center-state v1 / MPC', 'wang_free_state_mpc/results/center_v1_20261007/summary.json'),
]


def read(path):
    return json.loads(Path(path).read_text())


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def absolute_run(row):
    path = Path(row['run'])
    return path if path.is_absolute() else ROOT/path


def legacy_metadata_view(run, batch):
    """Read-only input adapter for the explicitly fixed-amplitude 2026-09-18 matrix.

    Reuse the exact frozen auditor, with a normalized metadata copy in this
    NEW batch and a read-only-consumed ULog symlink; never alter an old run.
    """
    allowed = BASE/'comparisons/motion_rate_racing/results'
    if not run.is_relative_to(allowed):
        return None
    if run.relative_to(allowed).parts[0] not in (
        'local_sweep_frozen_20260918','frozen_20260918','gp_frozen_20260918'):
        return None
    raw = read(run/'result.json')
    added = {name:1. for name in ('translation_scale','rotation_scale') if name not in raw}
    if not added:
        return None
    protocol = BASE/'comparisons/motion_rate_racing/PROTOCOL.md'
    evidence = dict(original_run=str(run), added_fields=added, basis=str(protocol),
        basis_sha256=digest(protocol), reason='Frozen frequency-only experiment: translation and rotation amplitudes unchanged')
    view = batch/'historical_metadata_views'/hashlib.sha256(str(run).encode()).hexdigest()[:16]
    view.mkdir(parents=True,exist_ok=True)
    save(view/'result.json',dict(raw,**added))
    save(view/'normalization.json',evidence)
    target = view/'flight.ulg'
    if not target.exists():
        target.symlink_to(run/'flight.ulg')
    if target.resolve() != (run/'flight.ulg').resolve():
        raise RuntimeError('Unexpected legacy ULog link')
    return view,evidence


def enhanced(row, cache, cache_path):
    """The old label is immutable; compute a separately named stricter label."""
    run = absolute_run(row)
    identity = str(run)
    result, ulog = run/'result.json', run/'flight.ulg'
    fingerprint = dict(result_sha256=digest(result) if result.exists() else None,
                       ulog_sha256=digest(ulog) if ulog.exists() else None)
    prior = cache.get(identity)
    if prior is not None and prior['fingerprint'] != fingerprint:
        raise RuntimeError('Historical evidence changed: '+identity)
    retry_legacy = prior is not None and prior['audit'].get('error') in (
        "KeyError('translation_scale')", "KeyError('rotation_scale')")
    compatibility = legacy_metadata_view(run,cache_path.parent) if prior is None or retry_legacy else None
    if prior is None or (retry_legacy and compatibility):
        try:
            passage = audit_run(compatibility[0] if compatibility else run)
            if compatibility:
                passage['metadata_compatibility'] = compatibility[1]
        except Exception as exc:
            passage = dict(passed=False, violations=['audit_error'], error=repr(exc))
        previous = [prior['audit']] if prior else []
        prior = dict(fingerprint=fingerprint, audit=passage,previous_audits=previous)
        cache[identity] = prior
        save(cache_path, cache)
    passage = prior['audit']
    unknown = bool(passage.get('error') or any(v in passage['violations'] for v in
        ('audit_error', 'truth_does_not_cover_flight', 'insufficient_flight_data')))
    return dict(row, legacy_realtime_success=bool(row.get('realtime_success')),
        legacy_physical_success=bool(row.get('physical_success')),
        realtime_success=bool(row.get('realtime_success') and passage['passed']),
        physical_success=bool(row.get('physical_success') and passage['passed']),
        strict_dynamics_success=bool(row.get('strict_dynamics_success') and passage['passed']),
        single_passage_success=bool(passage['passed']), passage_violations=passage['violations'],
        event_evidence_unavailable=unknown)


def summary(rows):
    out = stats(rows)
    good = [r for r in rows if r.get('realtime_success')]
    out.update(legacy_realtime_success=sum(bool(r.get('legacy_realtime_success', r.get('realtime_success'))) for r in rows),
        event_evidence_unavailable=sum(bool(r.get('event_evidence_unavailable')) for r in rows),
        event_passed=sum(bool(r.get('single_passage_success')) for r in rows),
        min_success_time_s=min((r['flight_time_s'] for r in good), default=None),
        max_success_time_s=max((r['flight_time_s'] for r in good), default=None),
        min_success_clearance_m=min((r['min_clearance_m'] for r in good if r.get('min_clearance_m') is not None), default=None),
        mpc_max_ms=max((r['mpc_cycle_ms']['max'] for r in rows if r.get('mpc_cycle_ms')), default=None),
        sender_max_ms=max((r['sender_cycle_ms']['max'] for r in rows if r.get('sender_cycle_ms')), default=None),
        strict_dynamics_success=sum(bool(r.get('strict_dynamics_success')) for r in rows))
    return out


def reasons(row):
    if row.get('realtime_success'):
        return ''
    values = []
    if row.get('infrastructure_failure'):
        values.append('准备/设施异常')
    if row.get('failure'):
        values.append(str(row['failure']))
    if row.get('collision'):
        values.append('门框接触')
    if row.get('min_clearance_m') is not None and row['min_clearance_m'] < 0:
        values.append('真值净空为负')
    if row.get('analysis_error'):
        values.append('审计证据不足')
    if row.get('room_passed') is False:
        values.append('房间越界')
    if row.get('controller_deadline_passed') is False or (row.get('cycle_ms') or {}).get('max', 0) >= 100:
        values.append('在线超过100ms')
    values.extend(row.get('passage_violations', []))
    return '; '.join(values) or '其他验收未通过'


def failure_diagnostics(row):
    """Summarize observed symptoms, not an untested causal explanation."""
    run = absolute_run(row)
    raw = read(run/'result.json') if (run/'result.json').exists() else {}
    log = run/'solves_live.jsonl'
    solves = [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []
    accepted = [s for s in solves if s.get('accepted')]
    last = accepted[-1] if accepted else None
    candidates = [s for s in solves if s.get('coefficients') and
                  str(s.get('rejection_reason','')).startswith('non_single_passage:')]
    replay = []
    if candidates:
        for candidate in (candidates[:1]+candidates[-1:] if len(candidates)>1 else candidates):
            try:
                replay.append(candidate_event_evidence(raw,candidate))
            except Exception as exc:
                replay.append(dict(error=repr(exc)))
    geometry = None
    if 'missed_current_opening' in str(row.get('failure')):
        try:
            geometry = last_gate_geometry(raw,run)
        except Exception as exc:
            geometry = dict(error=repr(exc))
    return dict(condition_id=row['condition_id'], run=str(run), failure=row.get('failure'),
        gates=row.get('gates'), last_time_s=row.get('last_time_s'),
        completed_center_passages=len(raw.get('crossings',[])),
        planning_skips=raw.get('planning_skips'),
        total_replans=len(solves), accepted_replans=len(accepted),
        rejection_counts=dict(Counter(s.get('rejection_reason','unspecified') for s in solves if not s.get('accepted'))),
        last_accepted={k:last.get(k) for k in ('time','made_at','gate','phase','durations')} if last else None,
        reference_end_s=float(last['made_at']+sum(last['durations'])) if last and last.get('durations') else None,
        last_rejection={k:solves[-1].get(k) for k in ('time','gate','phase','rejection_reason')} if solves else None,
        mpc_failed_calls=row.get('mpc_failed_calls'), representative_candidate_replay=replay,
        last_recorded_current_gate_geometry=geometry)


def last_gate_geometry(raw,run):
    import csv
    from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
    from convex_timevarying_window.receding_gate_racing.scenario import vary_track
    with (run/'telemetry_live.csv').open() as f:
        samples = list(csv.DictReader(f))
    sample = samples[-1]
    track,_ = build_track()
    vary_track(track,raw['phase'],raw['layout'],raw['motion_rate'],raw['translation_scale'],raw['rotation_scale'])
    windows = [track.windows[i] for i in track.order]
    gate = int(sample['gate']);w = windows[gate]
    stamp = float(sample.get('state_time',sample['t']))
    position = np.array([float(sample[k]) for k in ('x','y','z')])
    q,z = w.world_to_local(position,stamp)
    extent = w.aperture.radius if w.aperture.kind=='circle' else np.linalg.norm(w.aperture.physical_vertices,axis=1).max()
    return dict(gate=gate,measurement_time=stamp,position=position.tolist(),
        window_local_q=q.tolist(),unoriented_signed_normal_coordinate=float(z),
        lateral_distance_m=float(np.linalg.norm(q)),aperture_enclosing_radius_m=float(extent),
        inside_aperture=bool(w.aperture.contains(q)),
        note='Last recorded estimated sample, possibly just before the terminating observation; not an exact reconstruction of the unlogged terminating call')


def candidate_event_evidence(raw, solve):
    from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
    from convex_timevarying_window.receding_gate_racing.scenario import vary_track
    from convex_timevarying_window.wang_free_state_mpc.trajectory import Plan
    from convex_timevarying_window.wang_free_state_mpc.events import plane_events
    track,_ = build_track()
    vary_track(track,raw['phase'],raw['layout'],raw['motion_rate'],raw['translation_scale'],raw['rotation_scale'])
    windows = [track.windows[i] for i in track.order]
    gate = int(solve['rejection_reason'].split(':')[-1])
    w = windows[gate]
    source = track.start if gate==0 else windows[gate-1].center0
    sign = float(np.sign(w.state_at(0.)[1][:,2]@(source-w.center0)) or 1.)
    durations = np.array(solve['durations'])
    plan = Plan(solve['made_at'],solve['candidate_first_gate'],durations,
        np.array(solve['coefficients']),np.array(solve['variables']),0.,solve['violation'],
        solve['status'],len(durations)//2-1)
    times = np.unique(np.r_[np.arange(plan.made_at,plan.end_time,.01),plan.end_time,plan.crossing_times])
    events,_ = plane_events(w,sign,plan.sample_many(times),times,gate=gate,
        nearby_margin=raw['config']['radius']+raw['config']['frame_thickness'])
    return dict(request_result_time=solve['time'],candidate_start=plan.made_at,
        rejected_gate=gate,designated_crossing_times=plan.crossing_times.tolist(),events=events,
        evidence='Read-only replay of a rejected, never executed nominal candidate; 10ms sampled')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('batch', type=Path)
    args = p.parse_args()
    batch = args.batch.resolve()
    if read(batch/'status.json')['state'] != 'complete':
        raise RuntimeError('Historical ULog audit is intentionally deferred until timed flights finish')
    current = read(batch/'condition_summary.json')
    first = read(batch/'first_attempt_summary.json')
    attempts = read(batch/'summary.json')
    cache_path = batch/'historical_passage_audits.json'
    cache = read(cache_path) if cache_path.exists() else {}
    manifest, historical, comparisons = [], [], []
    for matrix, name, relative in SOURCES+EXTRA:
        path = BASE/relative
        old = selected(read(path))
        audited = [enhanced(r, cache, cache_path) for r in old]
        source = dict(matrix=matrix, method=name, original=old, audited=audited)
        historical.append(source)
        manifest.append(dict(matrix=matrix, method=name, path=str(path), sha256=digest(path)))
        oldkeys = {key(r) for r in old}
        matching = [r for r in current if key(r) in oldkeys]
        legacy_new = [dict(r, realtime_success=r.get('legacy_realtime_success',r.get('realtime_success'))) for r in matching]
        comparisons.append(dict(matrix=matrix, method=name, old=summary(audited),
            new=summary(matching), strict_paired=paired(matching,audited),
            legacy_paired=paired(legacy_new,old)))
        print(f"AUDITED {matrix} {name}: {sum(r['realtime_success'] for r in audited)}/{len(audited)}",flush=True)
    manifest_path = batch/'baseline_manifest.json'
    if manifest_path.exists() and read(manifest_path) != manifest:
        raise RuntimeError('Historical source summaries changed')
    save(manifest_path,manifest)
    save(batch/'historical_comparison_rows.json',historical)
    totals = summary(current)
    diagnostics = [failure_diagnostics(r) for r in current if not r.get('realtime_success')]
    save(batch/'failure_diagnostics.json',diagnostics)
    result = dict(first_attempt=summary(first), selected_conditions=totals, attempts=len(attempts),
        comparisons=comparisons, failures=dict(Counter(reasons(r) for r in current if not r.get('realtime_success'))),
        failure_diagnostics=diagnostics)
    save(batch/'comparison.json',result)
    lines = ['# Wang-MINCO 自由状态 / MPC 事件版：38条件完整实验','',
        f'全部 {len(current)}/38 条件已执行，共 {len(attempts)} 趟；首尝试主验收 {result["first_attempt"]["realtime_success"]}/{len(first)}，允许设施重试后 {totals["realtime_success"]}/{len(current)}。',
        '飞行算法与event_v4开发版63项源码逐项一致，整个矩阵固定参数，不调参、不重试算法失败。',
        '本批全部重新飞行，不将开发两趟复用为正式数据。x500/PX4/Gazebo真实动力学闭环，不是轨迹动画。','',
        '## 如何算成功','',
        '旧口径：七窗返航、零门框接触、10ms真值整机球净空非负、房间界、所有规划/MPC/发送调用<100ms。',
        '本次还要求：七窗各一次正向穿越、顺序正确、整机离面且不在有限窗附近返穿。历史对照用相同事件审计重新只读复核；旧标签单列，原文件不改。',
        '12m/s、8m/s²、42°名义超限依用户要求只作诊断。采样/接触证据不是连续安全证明，每条件一次也不是随机总体成功概率。',
        f'本批旧口径通过 {totals["legacy_realtime_success"]}/{len(current)}；增加事件条件后 {totals["realtime_success"]}/{len(current)}。',
        f'合格圈时中位数 {fmt(totals["median_success_time_s"])} s，范围 {fmt(totals["min_success_time_s"])}–{fmt(totals["max_success_time_s"])} s；最小合格净空 {fmt(totals["min_success_clearance_m"],4)} m。',
        f'所有尝试中规划/MPC/发送最大值 {fmt(totals["worst_cycle_ms"],2)}/{fmt(totals["mpc_max_ms"],2)}/{fmt(totals["sender_max_ms"],2)} ms。','',
        '## 与已有方法对照','',
        '| 矩阵 / 历史方法 | 旧标签成功 | 加事件审计后 | 本次对应条件成功 | 双方合格 | 本次更快 | 本次减旧中位s |',
        '|---|---:|---:|---:|---:|---:|---:|']
    for c in comparisons:
        old,new,pair = c['old'],c['new'],c['strict_paired']
        lines.append(f"| {c['matrix']} / {c['method']} | {old['legacy_realtime_success']}/{old['n']} | {old['realtime_success']}/{old['n']} | {new['realtime_success']}/{new['n']} | {pair['common_successes']} | {pair['new_faster']} | {fmt(pair['median_new_minus_old_s'])} |")
    unknown = {r['run'] for source in historical for r in source['audited'] if r['event_evidence_unavailable']}
    lines += ['',f'历史共读取{len(cache)}个不同运行目录；{len(unknown)}趟无法完整复核事件（字段缺失、真值覆盖或日志缺失）。它们不算已证实返穿；本批这些缺证据趟在旧口径下也未成功。',
        '', '### 旧验收口径的计时追溯（不是严格各窗一次的竞速排名）','',
        '| 矩阵 / 历史方法 | 同条件双方旧口径合格 | 本次更快 | 本次减旧中位s |',
        '|---|---:|---:|---:|']
    for c in comparisons:
        pair = c['legacy_paired']
        lines.append(f"| {c['matrix']} / {c['method']} | {pair['common_successes']} | {pair['new_faster']} | {fmt(pair['median_new_minus_old_s'])} |")
    lines += ['','负数表示本次更快。只比较完全相同条件和双方都合格的圈时，同时保留全部失败分母。',
        '中心24、幅度16、频率12存在重叠；不能把它们加起来当成独立样本。development2仅两个开发条件，不与全矩阵等量齐观。',
        '历史独立复现及动态扩展不是原作者官方性能。状态RL未完成匹配动态场景PX4迁移，无可比有效圈速；离线TOGT名义24.6608s不是安全闭环成绩。',
        '历史试验并非同期随机重复，毫秒级差异不能当作显著提速；两个已用于开发的条件不构成独立留出。','',
        '## 本次分矩阵','',
        '| 矩阵 | 首尝试成功 | 允许设施重试后 | 合格圈时中位s |',
        '|---|---:|---:|---:|']
    for name,ids in groups().items():
        sub = [r for r in current if tuple(r['condition_id']) in ids]
        raw = [r for r in first if tuple(r['condition_id']) in ids]
        st = summary(sub)
        lines.append(f"| {name} | {sum(bool(r.get('realtime_success')) for r in raw)}/{len(raw)} | {st['realtime_success']}/{len(sub)} | {fmt(st['median_success_time_s'])} |")
    lines += ['','## 全部38个条件','',
        '| r/t/a/相位编号 | 尝试 | 主验收 | 合格圈时s | 采样净空m | 规划/MPC最大ms | 失败原因 |',
        '|---|---:|---|---:|---:|---|---|']
    for r in current:
        plan,mpc = r.get('cycle_ms') or {},r.get('mpc_cycle_ms') or {}
        lines.append(f"| {'/'.join(map(str,r['condition_id']))} | {r['attempt']} | {'通过' if r.get('realtime_success') else '失败'} | {fmt(r.get('flight_time_s') if r.get('realtime_success') else None)} | {fmt(r.get('min_clearance_m'),4)} | {fmt(plan.get('max'),2)}/{fmt(mpc.get('max'),2)} | {reasons(r)} |")
    lines += ['','r=运动频率倍数，t=平移幅度倍数，a=旋转幅度倍数；相位编号0/1/2为0/120/240度。',
        '失败前运行时间不写成完成时间。详细失败阶段、接触、事件和ULog见各趟目录。','',
        '## 历史新增事件问题与缺证据','']
    for source in historical:
        bad = [r for r in source['audited'] if r['legacy_realtime_success'] and not r['single_passage_success']]
        if bad:
            lines.append(f"- {source['matrix']} / {source['method']}：旧合格但事件未通过{len(bad)}趟，其中缺证据{sum(r['event_evidence_unavailable'] for r in bad)}趟。")
            for r in bad:
                lines.append(f"  - 条件{key(r)}：{', '.join(r['passage_violations'])}；原目录 `{r['run']}`。")
    lines += ['','## 保留全部失败和重试','']
    for r in attempts:
        if not r.get('realtime_success'):
            lines.append(f"- {r['condition_id']} attempt{r['attempt']}：{reasons(r)}；最后时间{fmt(r.get('last_time_s'))} s；原目录 `{r['run']}`。")
    if all(r.get('realtime_success') for r in attempts):
        lines.append('本批无失败或重试。')
    if diagnostics:
        lines += ['','### 失败日志中的直接证据','',
            '下列是日志观察，不等于证明赛道不可行，也不代替候选轨迹重放和独立消融。']
        for d in diagnostics:
            lines.append(f"- {d['condition_id']}：已记中心过窗{d['completed_center_passages']}个，接受/尝试规划={d['accepted_replans']}/{d['total_replans']}；最后接受={d['last_accepted']}；拒绝计数={d['rejection_counts']}；MPC失败调用={d['mpc_failed_calls']}。")
            if d['last_recorded_current_gate_geometry']:
                lines.append(f"  - 中止前最后记录的位置与目标窗几何：{d['last_recorded_current_gate_geometry']}。")
    lines += ['','## 图','', '![相同口径对比](center24_comparison.png)','',
        '![38条件完整结果](matrix_times.png)','',
        '图中的失败不显示为有效圈速；历史加事件复核标签不覆盖历史原标签。','',
        '## 可追溯文件','',
        '- `protocol.json` / `source_snapshot/`：参数、环境、65项编排与飞行源码快照。',
        '- `summary.json` / `first_attempt_summary.json` / `condition_summary.json`：全部尝试、首尝试与按协议选择结果。',
        '- `historical_passage_audits.json`：只读历史ULog事件复核、result/ULog SHA256。',
        '- `historical_metadata_views/`：旧频率矩阵未保存幅度字段；依其固定幅度协议补1倍的分析输入副本，ULog链接原文件，原日志不改；初次字段缺失错误仍保存在审计历史。',
        '- `baseline_manifest.json` / `historical_comparison_rows.json` / `comparison.json`：历史来源、逐项配对及摘要。',
        '- 每趟 `passage_audit.json`、`result.json`、ULog、接触、遥测及求解日志。','']
    (batch/'REPORT.md').write_text('\n'.join(lines))
    plot(batch,current,historical)
    save(batch/'analysis_source_sha256.json',{str(Path(__file__).relative_to(ROOT)):digest(__file__)})
    print(json.dumps(result['selected_conditions'],ensure_ascii=False,indent=2),flush=True)


def plot(batch,current,historical):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    center = groups()['center24']
    data = [('Current event-v4',[r for r in current if tuple(r['condition_id']) in center])]
    for name,matrix in [('Checked-Soft / optimized MPC','union38'),('Checked','center24'),
                        ('Wang 2023 independent','center24'),('Direct-CTBR MPPI','center24')]:
        source = next(s for s in historical if s['method']==name and s['matrix']==matrix)
        wanted = {key(r) for r in data[0][1]}
        data.append((name,[r for r in source['audited'] if key(r) in wanted]))
    fig,axes = plt.subplots(1,2,figsize=(11,4.8),constrained_layout=True)
    for i,(name,rows) in enumerate(data):
        good = [r for r in rows if r.get('realtime_success')]
        legacy = sum(bool(r.get('legacy_realtime_success',r.get('realtime_success'))) for r in rows)
        axes[0].bar(i-.17,legacy,width=.32,color='lightgray',label='Legacy checks' if i==0 else None)
        axes[0].bar(i+.17,len(good),width=.32,color='tab:blue',label='+ same passage audit' if i==0 else None)
        axes[0].text(i+.17,len(good)+.3,f'{len(good)}/{len(rows)}',ha='center',fontsize=8)
        axes[1].scatter(np.full(len(good),i),[r['flight_time_s'] for r in good],s=22)
    for ax,title in zip(axes,['Accepted runs (center24)','Accepted lap times [s]']):
        ax.set_title(title)
        ax.set_xticks(range(len(data)),[d[0] for d in data],rotation=25,ha='right',fontsize=8)
        ax.grid(axis='y',alpha=.2)
    axes[0].set_ylim(0,27);axes[0].legend(fontsize=8)
    fig.savefig(batch/'center24_comparison.png',dpi=160);plt.close(fig)
    fig,ax = plt.subplots(figsize=(15,5),constrained_layout=True)
    old = next(s['audited'] for s in historical if s['method']=='Checked-Soft / optimized MPC')
    index = {key(r):i for i,r in enumerate(current)}
    for label,rows,marker in [('Current event-v4',current,'o'),('Previous optimized MPC',old,'x')]:
        good = [r for r in rows if r.get('realtime_success') and key(r) in index]
        ax.scatter([index[key(r)] for r in good],[r['flight_time_s'] for r in good],marker=marker,label=label)
    failed = [i for i,r in enumerate(current) if not r.get('realtime_success')]
    for i in failed:
        ax.axvspan(i-.35,i+.35,color='tab:red',alpha=.12)
    ax.set_xticks(range(len(current)),['/'.join(map(str,r['condition_id'])) for r in current],rotation=90,fontsize=8)
    ax.set_ylabel('Accepted lap time [s]');ax.set_xlabel('r / translation / rotation / phase index')
    ax.set_title('All 38 conditions; shaded columns = current failure, no valid lap time')
    ax.grid(axis='y',alpha=.2);ax.legend()
    fig.savefig(batch/'matrix_times.png',dpi=160);plt.close(fig)


if __name__ == '__main__':
    main()
