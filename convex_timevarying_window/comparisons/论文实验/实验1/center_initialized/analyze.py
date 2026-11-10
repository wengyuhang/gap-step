"""Report closed-loop performance and per-request nominal incumbent separately."""
import json
from pathlib import Path
from convex_timevarying_window.comparisons.single_gate_constant_velocity.run import save


def analyze(out):
    out=Path(out);status=json.loads((out/'status.json').read_text())
    if status['state']!='complete' or not status['sources_unchanged']:raise RuntimeError('incomplete or changed source')
    rows=json.loads((out/'summary.json').read_text());requests=[]
    for row in rows:
        raw=json.loads((Path(row['directory'])/'result.json').read_text())
        if row['variant']!='ours':continue
        for solve in raw['solves']:
            if not solve['accepted']:continue
            s=solve['optimizer_stats'];center=s['center_reference']['duration_s'];selected=sum(solve['durations'])
            if selected>center+1e-12 or s['candidate_policy']!='center_then_free_incumbent':raise RuntimeError('incumbent guarantee/interface mismatch')
            requests.append(dict(amplitude_m=row['amplitude_m'],frequency_hz=row['frequency_hz'],center_nominal_s=center,
                returned_nominal_s=selected,nominal_saved_s=center-selected,selected_stage=s['selected_stage'],
                center_stage_ms=s['center_stage_ms'],free_stage_ms=s['free_stage_ms'],total_request_ms=solve['cycle_ms'],
                accepted_at_s=solve['time'],reference_start_s=solve['made_at'],acceptance_lag_s=solve['time']-solve['made_at']))
    lines=['# 中心解初始化：三组开发复测','',
        '每次自由规划内部先求中心、再开放选点；两阶段总成本计入。相同旧MINCO/约束/MPC/小窗/碰撞/终点。',
        '每条件各一次；新自由与原单阶段中心比较，不是等计算预算消融。全部完成时间含首解等待、到终点和减速。','',
        '|振幅m|频率Hz|自由结果|自由完整s|中心结果|中心完整s|自由节省|','|---:|---:|---|---:|---|---:|---:|']
    for amp,f in sorted({(r['amplitude_m'],r['frequency_hz']) for r in rows}):
        group={r['variant']:r for r in rows if (r['amplitude_m'],r['frequency_hz'])==(amp,f)};a,b=group['ours'],group['center']
        values=[str(amp),str(f)]
        for r in (a,b):values.extend([r['verdict'],f"{r['flight_time_s']:.3f}" if r['passed'] else '—'])
        values.append(f"{100*(b['flight_time_s']-a['flight_time_s'])/b['flight_time_s']:.2f}%" if a['passed'] and b['passed'] else '—')
        lines.append('| '+' | '.join(values)+' |')
    lines+=['','## 同一次请求的名义不劣性','',
        '|振幅m|频率Hz|内部中心s|返回s|选择|中心ms|自由精修ms|完整请求ms|接纳滞后s|',
        '|---:|---:|---:|---:|---|---:|---:|---:|---:|']
    for r in requests:
        keys=['amplitude_m','frequency_hz','center_nominal_s','returned_nominal_s','selected_stage','center_stage_ms','free_stage_ms','total_request_ms','acceptance_lag_s']
        lines.append('| '+' | '.join(f'{r[k]:.4f}' if isinstance(r[k],float) else str(r[k]) for k in keys)+' |')
    lines+=['','名义保证只相对同一请求内的中心轨迹，两次独立飞行的初态/绝对请求时刻不同，不保证闭环逐条件提速。',
        '超过100ms如实统计，参考接纳晚于其起始时刻也不隐去；不暂停/重置窗口来弥补计算。',
        '结果为当前三组开发验证，不覆盖旧论文矩阵，也不估计随机成功概率。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n');save(out/'analysis.json',dict(flights=len(rows),accepted_free_requests=requests,
        all_nominal_not_worse=all(r['returned_nominal_s']<=r['center_nominal_s']+1e-12 for r in requests),
        closed_loop_guarantee=False,equal_compute_budget=False))
    print('\n'.join(lines),flush=True)
