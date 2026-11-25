"""Explicit full-matrix tables, request audit and raw-data index; no flight changes."""
import csv
import json
import math
import os
from pathlib import Path
import numpy as np
from convex_timevarying_window.comparisons.single_gate_constant_velocity.run import save
from convex_timevarying_window.comparisons.single_gate_swing.run import CASES

LABELS = dict(completed_no_contact='无碰撞完赛', collision='碰撞', missed_gate='漏窗',
    not_completed='未完成', online_no_accepted_plan='未接纳轨迹', evidence_failure='证据不足')


def collect_requests(rows):
    requests = []
    for row in rows:
        raw = json.loads((Path(row['directory'])/'result.json').read_text())
        for solve in raw['solves']:
            stats = solve.get('optimizer_stats') or {}
            candidates = stats.get('temporal_candidates', [])
            if len(candidates) != 4 or not stats.get('all_time_seeds_attempted'):
                raise RuntimeError('four-start protocol evidence missing: '+row['directory'])
            if any(c['seed_variables'][2:4] != [0., 0.] for c in candidates):
                raise RuntimeError('non-centre spatial initialization')
            duration = sum(solve['durations']) if solve['accepted'] else None
            if solve['accepted']:
                best = min(c['duration_s'] for c in candidates if c['admitted'])
                if not math.isclose(duration, best, abs_tol=1e-12, rel_tol=0):
                    raise RuntimeError('selected candidate was not the shortest admitted')
            requests.append(dict(amplitude_m=row['amplitude_m'], frequency_hz=row['frequency_hz'],
                variant=row['variant'], accepted=solve['accepted'], nominal_duration_s=duration,
                selected_seed=stats.get('selected_seed_index'), candidates=candidates,
                total_request_ms=solve['cycle_ms'],
                acceptance_lag_s=solve['time']-solve['made_at'] if solve['accepted'] else None,
                directory=row['directory']))
    return requests


def summarize(out, final=False):
    out = Path(out)
    rows = json.loads((out/'summary.json').read_text())
    rows = sorted(rows, key=lambda r:(r['amplitude_m'], r['frequency_hz'], r['variant']))
    requests = collect_requests(rows)
    comparisons = []
    for frequency, amplitude in CASES:
        pair = {r['variant']:r for r in rows if r['frequency_hz']==frequency and r['amplitude_m']==amplitude}
        a, b = pair.get('ours', {}), pair.get('center', {})
        both = a.get('passed') and b.get('passed')
        comparisons.append(dict(amplitude_m=amplitude, frequency_hz=frequency,
            ours_verdict=a.get('verdict', 'pending'), ours_time_s=a.get('flight_time_s') if a.get('passed') else None,
            center_verdict=b.get('verdict', 'pending'), center_time_s=b.get('flight_time_s') if b.get('passed') else None,
            saving_percent=100*(b['flight_time_s']-a['flight_time_s'])/b['flight_time_s'] if both else None))
    methods = {}
    for variant in ('ours', 'center'):
        rr = [r for r in rows if r['variant']==variant]
        sr = [r for r in requests if r['variant']==variant]
        times = [r['total_request_ms'] for r in sr]
        mpc = [float(s['cycle_ms']) for r in rr
            for s in json.loads((Path(r['directory'])/'result.json').read_text())['mpc_cycles'] if s.get('time',0)>=0]
        methods[variant] = dict(completed_no_collision=sum(bool(r.get('passed')) for r in rr),
            collisions=sum(bool(r.get('collision_detected')) for r in rr),
            incomplete_no_collision=sum(not r.get('passed') and not r.get('collision_detected') for r in rr),
            flights=len(rr), planning_requests=len(sr), candidate_optimizations=sum(len(r['candidates']) for r in sr),
            planning_median_ms=float(np.median(times)) if times else None,
            planning_p95_ms=float(np.percentile(times,95)) if times else None,
            planning_max_ms=max(times,default=None), planning_over100=sum(t>100 for t in times),
            mpc_p95_ms=float(np.percentile(mpc,95)) if mpc else None, mpc_max_ms=max(mpc,default=None),
            mpc_over100=sum(t>100 for t in mpc),
            max_acceptance_lag_s=max((r['acceptance_lag_s'] for r in sr if r['accepted']),default=None))
    dynamic = [c for c in comparisons if c['amplitude_m']>0 and c['saving_percent'] is not None]
    result = dict(final=final, completed_records=len(rows), total_records=34, methods=methods,
        conditions=comparisons, requests=requests,
        dynamic_pairs_with_times=len(dynamic), dynamic_free_faster=sum(c['saving_percent']>0 for c in dynamic),
        dynamic_center_faster=sum(c['saving_percent']<0 for c in dynamic),
        dynamic_mean_paired_saving_percent=float(np.mean([c['saving_percent'] for c in dynamic])) if dynamic else None)
    save(out/'analysis.json', result)
    with (out/'comparison.csv').open('w', newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(comparisons[0]));w.writeheader();w.writerows(comparisons)
    fields=['amplitude_m','frequency_hz','variant','verdict','flight_time_s','frame_collision','ground_collision',
        'planner_max_ms','mpc_max_ms','actual_distance_m','maximum_speed_mps','reuse_from_previous_batch','directory']
    with (out/'all_results.csv').open('w', newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(rows)
    lines=['# 实验1：四时间初值完整平移对比','',
        f"状态：{'完成' if final else '进行中'}，已取得{len(rows)}/34条记录；静态一次，加2m、4m振幅下0.5～4Hz、0.5Hz步长。",'',
        '双方每个条件实飞一次、初相零；每次规划的四个空间初值均为中心，时间初值尺度1/0.8/1.2/1.5，各选最短合格轨迹。',
        '小窗2.90×2.30m，x500/PX4、同单重射击MPC、同约束和碰撞判据。完整时间从任务开始到同一窗后终点及减速，包含初始规划等待；不是只到过窗平面。',
        '原六趟经全部既有源码SHA-256等价性核对后保留，新增28趟；没有重飞择优。新增文件仅完整矩阵协调与分析，映射见reuse_audit.json。','',
        '## 表1：完整任务时间与碰撞','',
        '|振幅 m|频率 Hz|自由结果|自由完整 s|中心结果|中心完整 s|自由节省|',
        '|---:|---:|---|---:|---|---:|---:|']
    for c in comparisons:
        values=[f"{c['amplitude_m']:g}", '静态' if c['amplitude_m']==0 else f"{c['frequency_hz']:g}"]
        for v in ('ours','center'):
            values.extend([LABELS.get(c[v+'_verdict'],'待完成'), f"{c[v+'_time_s']:.3f}" if c[v+'_time_s'] is not None else '—'])
        values.append(f"{c['saving_percent']:.2f}%" if c['saving_percent'] is not None else '—')
        lines.append('| '+' | '.join(values)+' |')
    lines+=['','节省率=(中心完整时间−自由完整时间)/中心完整时间；正值为自由点更快。只在双方无碰撞完赛时计算。','',
        '## 表2：全部四初值的在线计算时间','',
        '|方法|无碰撞完赛/记录|碰撞|无碰撞未完成|规划中位 ms|规划P95 ms|规划最大 ms|规划>100ms/请求|MPC最大 ms|',
        '|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for v in ('ours','center'):
        s=methods[v]; fmt=lambda value: f'{value:.2f}' if value is not None else '—'
        values=['自由点' if v=='ours' else '强制中心',f"{s['completed_no_collision']}/{s['flights']}",s['collisions'],s['incomplete_no_collision'],
            fmt(s['planning_median_ms']),fmt(s['planning_p95_ms']),fmt(s['planning_max_ms']),
            f"{s['planning_over100']}/{s['planning_requests']}",fmt(s['mpc_max_ms'])]
        lines.append('| '+' | '.join(map(str,values))+' |')
    if dynamic:
        lines+=['',f"动态已配对{len(dynamic)}组：自由更快{result['dynamic_free_faster']}组，中心更快{result['dynamic_center_faster']}组；逐组节省率均值{result['dynamic_mean_paired_saving_percent']:.2f}%。"]
    lines+=['','## 表3：逐条件计算与原始证据','',
        '|振幅 m|频率 Hz|方法|碰框/碰地|规划最大 ms|MPC最大 ms|来源|原始数据|',
        '|---:|---:|---|---|---:|---:|---|---|']
    for r in rows:
        rel=os.path.relpath(Path(r['directory'])/'result.json',out)
        values=[f"{r['amplitude_m']:g}",f"{r['frequency_hz']:g}",'自由点' if r['variant']=='ours' else '强制中心',
            f"{r.get('frame_collision','?')}/{r.get('ground_collision','?')}",f"{r['planner_max_ms']:.2f}" if r.get('planner_max_ms') is not None else '—',
            f"{r['mpc_max_ms']:.2f}" if r.get('mpc_max_ms') is not None else '—',
            '源码等价保留' if r.get('reuse_from_previous_batch') else '本批新飞',f'[{Path(r["directory"]).name}]({rel})']
        lines.append('| '+' | '.join(values)+' |')
    lines+=['','## 口径与记录','',
        '- 每次请求完整记录四条初值、失败候选、所选索引、耗时和接纳滞后，见analysis.json。飞前构建/JIT独立记录于原始result.json，不计飞行任务时间。',
        '- 全部四候选及安全验收计入规划耗时，当前沿用无墙钟上限；超过100ms照实列出。未修改原150ms规划提前量，正接纳滞后保留。',
        '- 无碰撞完赛要求实际正向穿越一次、整机出窗、到达共同终点且框/地无接触。碰撞依据任务区间Gazebo原生接触；ULog真值用于独立复核，不以选点合法或求解器收敛代替。',
        '- 各条件一次用于确定性对照，不据此估计随机成功概率。未接纳与漏窗均归未完成，不能当作碰撞或物理不可行。',
        '- 物理步长1ms、接触1000Hz；零接触为本次仿真观测。名义局部几何是浮点数值验收，动力学为17节点/段，不扩称闭环证书。',
        '- 未起飞设施故障允许一次重启，原始记录及原因保留；执行过的任务不重飞。源码/配对配置审计分别见protocol.json及pair_*.json。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')
    return result
