"""All-case flight comparison plus all-start timing and candidate selection audit."""
import json
from pathlib import Path
import numpy as np
from convex_timevarying_window.comparisons.single_gate_constant_velocity.run import save

LABELS=dict(completed_no_contact='无碰撞完成',collision='碰撞',missed_gate='漏窗',not_completed='未完成',online_no_accepted_plan='未接纳',evidence_failure='证据不足')


def analyze(out):
    out=Path(out);status=json.loads((out/'status.json').read_text())
    if status['state']!='complete' or not status['sources_unchanged']:raise RuntimeError('incomplete or changed source')
    rows=json.loads((out/'summary.json').read_text());requests=[]
    for row in rows:
        raw=json.loads((Path(row['directory'])/'result.json').read_text())
        for solve in raw['solves']:
            stats=solve['optimizer_stats'];candidates=stats.get('temporal_candidates',[])
            if len(candidates)!=4 or not stats['all_time_seeds_attempted']:raise RuntimeError('all four requested initializations were not attempted')
            if any(r['seed_variables'][2:4]!=[0.,0.] for r in candidates):raise RuntimeError('non-centre spatial initialization')
            if solve['accepted']:
                duration=sum(solve['durations']);best=min(r['duration_s'] for r in candidates if r['admitted'])
                if not np.isclose(duration,best,atol=1e-12,rtol=0):raise RuntimeError('not the shortest admitted candidate')
            else:duration=None
            requests.append(dict(amplitude_m=row['amplitude_m'],frequency_hz=row['frequency_hz'],variant=row['variant'],
                accepted=solve['accepted'],nominal_s=duration,selected_seed=stats['selected_seed_index'],
                candidate_durations_s=[r['duration_s'] for r in candidates],total_request_ms=solve['cycle_ms'],
                acceptance_lag_s=solve['time']-solve['made_at'] if solve['accepted'] else None))
    result=dict(flights=len(rows),requests=requests,methods={})
    for variant in ('ours','center'):
        r=[x for x in rows if x['variant']==variant];t=[x['total_request_ms'] for x in requests if x['variant']==variant]
        result['methods'][variant]=dict(completed_no_collision=sum(x['passed'] for x in r),collisions=sum(bool(x.get('collision_detected')) for x in r),
            incomplete=sum(not x['passed'] and not x.get('collision_detected') for x in r),max_planning_ms=max(t,default=None),over_100ms=sum(v>100 for v in t))
    lines=['# 四时间初值直接优化：自由与中心对比','','双方均从窗口中心起步，各自直接优化四组相同的时间初值，选最短合格候选；不先求中心给自由热启动。',
        '每条件每方法实飞一次，未按四条轨迹各飞一次择优。时间含首解等待、过窗、到同一终点及减速；全部四候选计算及验收计入规划耗时。',
        '当前无墙钟上限，保持原小窗/约束/MPC/PX4/碰撞/终点；不保证100ms或全局最优。','',
        '|振幅m|频率Hz|自由结果|自由完整s|中心结果|中心完整s|自由节省|','|---:|---:|---|---:|---|---:|---:|']
    for amp,f in sorted({(r['amplitude_m'],r['frequency_hz']) for r in rows}):
        pair={r['variant']:r for r in rows if (r['amplitude_m'],r['frequency_hz'])==(amp,f)};a,b=pair['ours'],pair['center'];values=[f'{amp:g}',f'{f:g}']
        for r in (a,b):values.extend([LABELS.get(r['verdict'],r['verdict']),f"{r['flight_time_s']:.3f}" if r['passed'] else '—'])
        values.append(f"{100*(b['flight_time_s']-a['flight_time_s'])/b['flight_time_s']:.2f}%" if a['passed'] and b['passed'] else '—');lines.append('| '+' | '.join(values)+' |')
    lines+=['','## 四候选与完整计算时间','','|振幅m|频率Hz|方法|四条合格名义时长s（失败为null）|选中索引|完整周期ms|接纳滞后s|','|---:|---:|---|---|---:|---:|---:|']
    for r in requests:lines.append('| '+' | '.join(map(str,[r['amplitude_m'],r['frequency_hz'],r['variant'],json.dumps(r['candidate_durations_s']),r['selected_seed'],round(r['total_request_ms'],2),r['acceptance_lag_s']]))+' |')
    lines+=['','正接纳滞后表示返回时参考起点已过去；原始记录保留，不重置时间或暂停窗口掩盖。',
        '两次独立飞行实际PVAJ/请求时刻不逐位相同；同输入对照另见multistart_probe批次。三组各一次，不估计随机成功概率。',
        '只有收敛、有限约束与局部安全验收均通过的轨迹参与选择；优化器成功或名义过窗不等于闭环安全成功。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n');save(out/'analysis.json',result);print('\n'.join(lines),flush=True)
