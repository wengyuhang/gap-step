"""Report every frozen trial; failed flights never acquire a valid lap time."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from pyulog import ULog
from scipy.spatial.transform import Rotation

from .scenario import Case, build_case, CENTERS, MOTION_AXES, audit_scene
from .run import conditions, source_hashes
from convex_timevarying_window.comparisons.checked_soft_racing.run import ROOT, save
from convex_timevarying_window.comparisons.wang_free_state_mpc.report import reasons, failure_diagnostics
from convex_timevarying_window.three_phase_mppi.report import continuous_quaternions

LABELS = {'ours':'Current: free-point MINCO + MPC','wang':'Wang 2023: independent reproduction'}
SHORT = {'ours':'当前方法','wang':'Wang 独立复现'}
COLORS = {'ours':'#006d77','wang':'#c85a17'}


def read(path): return json.loads(Path(path).read_text())
def number(value, digits=3): return '—' if value is None else f'{value:.{digits}f}'


def summarize(rows):
    good = [r for r in rows if r.get('realtime_success')]
    times = [r['flight_time_s'] for r in good]
    maximum = lambda values: max((v for v in values if v is not None),default=None)
    return dict(total=len(rows),passed=len(good),contacts=sum(bool(r.get('collision')) for r in rows),
        completed=sum(r.get('flight_time_s') is not None for r in rows),
        events_passed=sum(bool(r.get('single_passage_success')) for r in rows),
        median_time_s=float(np.median(times)) if times else None,
        min_time_s=min(times,default=None),max_time_s=max(times,default=None),
        min_valid_clearance_m=min((r['min_clearance_m'] for r in good),default=None),
        min_any_clearance_m=min((r['min_clearance_m'] for r in rows if r.get('min_clearance_m') is not None),default=None),
        planner_max_ms=maximum((r.get('cycle_ms') or {}).get('max') for r in rows),
        mpc_max_ms=maximum((r.get('mpc_cycle_ms') or {}).get('max') for r in rows),
        sender_max_ms=maximum(r.get('sender_max_ms',(r.get('sender_cycle_ms') or {}).get('max')) for r in rows),
        strict_nominal_dynamics_passed=sum(bool(r.get('strict_dynamics_success')) for r in rows))


def truth_trace(row):
    path = Path(row['run']);raw = read(path/'result.json')
    log = ULog(str(path/'flight.ulg'))
    p = log.get_dataset('vehicle_local_position_groundtruth').data
    a = log.get_dataset('vehicle_attitude_groundtruth').data
    stamp = p['timestamp']*1e-6
    valid = (stamp>=30.) & (stamp<=30.+raw['last_time_s'])
    absolute = stamp[valid];times = absolute-30.
    q = continuous_quaternions(np.column_stack([a[k] for k in ('q[1]','q[2]','q[3]','q[0]')]))
    R = Rotation.from_quat(np.column_stack([np.interp(absolute,a['timestamp']*1e-6,q[:,i]) for i in range(4)]))
    ned = np.column_stack([p[k][valid] for k in ('x','y','z')]).astype(float)
    ned += R.apply(np.tile([0.,0.,-.24],(len(times),1)))
    positions = ned[:,[1,0,2]]*np.array([1.,1.,-1.])
    speed = np.linalg.norm(np.gradient(positions,times,axis=0),axis=1)
    return times,positions,speed


def plot(batch, rows):
    figs = batch/'figures';figs.mkdir(exist_ok=True)
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    track,_ = build_case(Case(1.,2.,0.))
    fig,ax = plt.subplots(figsize=(8,6),layout='constrained')
    route = np.vstack([track.start,CENTERS,track.goal])
    ax.plot(route[:,0],route[:,1],'--',color='.7',lw=1,label='Nominal order (not a flight path)')
    for i,w in enumerate(track.windows):
        d = np.eye(3)[MOTION_AXES[i]]
        sweep = np.array([w.center0-2*d,w.center0+2*d])
        ax.plot(sweep[:,0],sweep[:,1],color='#7c8fb0',lw=8,alpha=.4,label='Center sweep: A=2 m' if i==0 else None)
        c = w.center0
        ax.scatter(c[0],c[1],s=25,color='#32496c')
        ax.text(c[0]+.7,c[1]+.7,f'W{i+1}')
        n = w.state_at(0.)[1][:,2]
        ax.arrow(c[0],c[1],2*n[0],2*n[1],head_width=.7,color='#32496c',length_includes_head=True)
    ax.scatter(*track.start[:2],marker='*',s=140,c='black',label='Start / finish')
    ax.set(xlabel='World x [m]',ylabel='World y [m]',title='Fixed-plane seven-gate course (top view)',aspect='equal')
    ax.grid(alpha=.2);ax.legend(loc='upper right',fontsize=8)
    fig.savefig(figs/'course.png',dpi=160);plt.close(fig)

    fig,axes = plt.subplots(1,2,figsize=(11,4),layout='constrained')
    combos = [(f,a) for f in (.6,1.) for a in (1.2,2.)]
    labels = [f'{f:g} Hz\nA={a:g} m' for f,a in combos]
    for method,offset in [('ours',-.18),('wang',.18)]:
        groups = [[r for r in rows if r['method']==method and (r['frequency_hz'],r['amplitude_m'])==c] for c in combos]
        counts = [sum(r['realtime_success'] for r in group) for group in groups]
        x = np.arange(4)+offset
        bars = axes[0].bar(x,counts,.32,color=COLORS[method],label=LABELS[method])
        axes[0].bar_label(bars,labels=[f'{n}/3' for n in counts],padding=3)
        for index,group in enumerate(groups):
            for row in group:
                if row['realtime_success']:
                    axes[1].scatter(index+offset,row['flight_time_s'],color=COLORS[method],s=45,
                        marker=['o','s','^'][int(round(row['phase']/(2*np.pi/3)))])
            valid = [r['flight_time_s'] for r in group if r['realtime_success']]
            if valid: axes[1].plot([index+offset-.1,index+offset+.1],[np.median(valid)]*2,color=COLORS[method],lw=2)
            else: axes[1].text(index+offset,.02,'no pass',transform=axes[1].get_xaxis_transform(),rotation=90,
                               color=COLORS[method],ha='center',va='bottom',fontsize=8)
    axes[0].set(ylabel='Fully accepted flights',ylim=(0,3.6),title='Success counts; 3 prescribed phases')
    axes[1].set(ylabel='Valid lap time [s]',title='Only accepted flights; marks = phases')
    for ax in axes: ax.set_xticks(range(4),labels);ax.grid(axis='y',alpha=.2)
    axes[0].legend(loc='upper center',fontsize=8)
    fig.savefig(figs/'comparison.png',dpi=170);plt.close(fig)

    # Fixed in advance: maximum frequency/amplitude, phase 0, even if failed.
    selected = [r for r in rows if tuple(r['condition_id'])==(1.,2.,0.)]
    fig,axes = plt.subplots(1,2,figsize=(12,4.5),layout='constrained')
    events = []
    for row in selected:
        method = row['method'];path = Path(row['run'])
        if not (path/'flight.ulg').exists(): continue
        t,positions,speed = truth_trace(row)
        axes[0].plot(t[2:-2],speed[2:-2],color=COLORS[method],label=LABELS[method])
        axes[1].plot(positions[:,0],positions[:,1],color=COLORS[method],label=LABELS[method])
        passage = read(path/'passage_audit.json')
        for event in passage.get('aperture_events',[]):
            events.append(dict(method=method,**event))
            axes[0].scatter(event['time'],np.interp(event['time'],t,speed),s=22,color=COLORS[method],
                            marker='o' if event['direction']=='forward' else 'x')
        if not row['realtime_success']:
            axes[0].annotate('NOT accepted',(t[-1],speed[-1]),fontsize=8,color=COLORS[method])
    for i,c in enumerate(CENTERS): axes[1].text(c[0],c[1],f'W{i+1}',fontsize=8)
    axes[0].set(xlabel='Race time [s]',ylabel='Body-center speed [m/s]',title='1 Hz, A=2 m, phase=0: speed')
    axes[1].set(xlabel='World x [m]',ylabel='World y [m]',title='Same fixed condition: actual paths',aspect='equal')
    for ax in axes: ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.savefig(figs/'hard_speed_path.png',dpi=170);plt.close(fig)
    save(batch/'hard_crossings.json',events)

    fig,axes = plt.subplots(2,4,figsize=(12,6),layout='constrained')
    for i,w in enumerate(track.windows):
        ax = axes.flat[i];bound = w.aperture.boundary_points(100);bound = np.vstack([bound,bound[:1]])
        ax.plot(bound[:,0],bound[:,1],color='.35');ax.scatter(0,0,c='.65',marker='+')
        for e in events:
            if e['gate']==i:
                ax.scatter(*e['q'],c=COLORS[e['method']],s=45,marker='o' if e['direction']=='forward' else 'x')
        ax.set(title=f'W{i+1}',aspect='equal',xlabel='Local horizontal [m]',ylabel='Local vertical [m]')
        ax.grid(alpha=.2)
    axes.flat[7].axis('off')
    axes.flat[7].text(0,.9,'1 Hz, A=2 m, phase=0\n\nTeal: current\nOrange: Wang independent\nCircle: forward crossing\nCross: backward crossing\n\nActual body-center intersection;\nno mark means not crossed.',va='top')
    fig.savefig(figs/'hard_crossing_points.png',dpi=170);plt.close(fig)


def main():
    p = argparse.ArgumentParser();p.add_argument('batch',type=Path);a = p.parse_args();batch = a.batch.resolve()
    if read(batch/'status.json')['state']!='complete': raise RuntimeError('Wait until timed flights finish')
    protocol = read(batch/'protocol.json')
    if protocol['pilot']: raise ValueError('Pilot is not a formal comparison')
    if source_hashes()!=protocol['source_sha256']: raise RuntimeError('Source mismatch')
    snapshot_errors = [rel for rel,digest in protocol['source_sha256'].items()
                       if hashlib.sha256((batch/'source_snapshot'/rel).read_bytes()).hexdigest()!=digest]
    if snapshot_errors: raise RuntimeError(snapshot_errors)
    rows = read(batch/'condition_summary.json');attempts = read(batch/'summary.json')
    expected = {(m,c) for m in LABELS for c in conditions()}
    actual = {(r['method'],tuple(r['condition_id'])) for r in rows}
    if expected!=actual or len(rows)!=24: raise ValueError('Incomplete or duplicate formal matrix')
    summary = {m:summarize([r for r in rows if r['method']==m]) for m in LABELS}
    paired = []
    for key in conditions():
        pair = {r['method']:r for r in rows if tuple(r['condition_id'])==key}
        if all(r['realtime_success'] for r in pair.values()):
            ours,wang = [pair[m]['flight_time_s'] for m in ('ours','wang')]
            paired.append(dict(condition_id=key,ours_s=ours,wang_s=wang,delta_s=ours-wang,
                               improvement_percent=100*(wang-ours)/wang))
    diagnostics = []
    for row in rows:
        if not row['realtime_success']:
            with audit_scene(Case(*row['condition_id'])):
                try: detail = failure_diagnostics(row)
                except Exception as exc: detail = dict(error=repr(exc))
            diagnostics.append(dict(method=row['method'],condition_id=row['condition_id'],reasons=reasons(row),detail=detail))
    save(batch/'comparison.json',dict(methods=summary,paired=paired,attempt_count=len(attempts),
         failed=diagnostics,verified_source_files=len(protocol['source_sha256']),
         report_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()))
    lines = ['# 固定平面、快速平移七窗：完整对比结果','',
        '两方都是本轮新跑的 Gazebo + PX4 x500 闭环飞行；没有使用离线轨迹替代飞行。',
        '窗口不旋转，也不沿法向移动，只在固定竖直平面内水平往复。原七种开口尺寸不变，新布局为顺向闭环。',
        '频率0.6/1.0 Hz，单侧幅度1.2/2.0 m，初相位0/120/240度：12条件×2方法。',
        '每个条件每个方法只飞一次；三种相位是预设条件，不是重复随机试验，不能据此推断统计显著性。',
        '当前方法保留自由选点MINCO、Checked-Soft、事件v4与原消元MPC；Wang保留独立复现的中心硬过点与PVA跟踪。',
        '这是整套方法对比，不是仅选点不同的消融，也不是Wang作者官方实现的性能。','',
        '## 总结果','',
        '|方法|合格/12|合格圈时中位数(s)|范围(s)|门框接触趟数|合格最小净空(m)|',
        '|---|---:|---:|---:|---:|---:|']
    for m,s in summary.items():
        lines.append(f"|{SHORT[m]}|{s['passed']}/12|{number(s['median_time_s'])}|{number(s['min_time_s'])}–{number(s['max_time_s'])}|{s['contacts']}|{number(s['min_valid_clearance_m'])}|")
    lines += ['',f"总尝试{len(attempts)}趟；正式方法-条件24项。源码及快照核对{len(protocol['source_sha256'])}项一致。",'']
    if paired:
        delta = np.array([r['delta_s'] for r in paired]);pct = np.array([r['improvement_percent'] for r in paired])
        lines += [f'两方共同合格的{len(paired)}个条件中，当前方法更快{sum(delta<0)}次、更慢{sum(delta>0)}次；',
            f'配对圈时差（当前−Wang）中位数{np.median(delta):+.3f}s，配对相对缩短中位数{np.median(pct):.2f}%。','']
    else:
        lines += ['没有双方共同合格的条件，不能计算公平的成功圈时配对优势。','']
    lines += ['不同方法成功条件可能不同，不能只比较各自成功样本的中位数就宣称全面更快。','',
        '## 频率与幅度','',
        '|频率(Hz)|单侧幅度(m)|当前合格/3|当前合格中位数(s)|Wang合格/3|Wang合格中位数(s)|',
        '|---:|---:|---:|---:|---:|---:|']
    for f in (.6,1.):
        for amp in (1.2,2.):
            stats = [summarize([r for r in rows if r['method']==m and r['frequency_hz']==f and r['amplitude_m']==amp]) for m in LABELS]
            lines.append(f"|{f}|{amp}|{stats[0]['passed']}/3|{number(stats[0]['median_time_s'])}|{stats[1]['passed']}/3|{number(stats[1]['median_time_s'])}|")
    lines += ['','## 所有条件','',
        '失败不填写有效圈时。记录的终止时间与圈时是不同指标；原始结果仍保留。','',
        '|频率|幅度|相位(度)|当前方法|Wang独立复现|','|---:|---:|---:|---|---|']
    for key in conditions():
        cells=[]
        for m in LABELS:
            row = next(r for r in rows if r['method']==m and tuple(r['condition_id'])==key)
            cells.append(f"{row['flight_time_s']:.3f}s" if row['realtime_success'] else f"失败：{reasons(row)}")
        lines.append(f'|{key[0]}|{key[1]}|{np.degrees(key[2]):.0f}|{cells[0]}|{cells[1]}|')
    lines += ['','## 在线耗时与证据边界','',
        '|方法|最大规划调用(ms)|最大MPC调用(ms)|最大主机发送调用(ms)|','|---|---:|---:|---:|']
    for m,s in summary.items(): lines.append(f"|{SHORT[m]}|{number(s['planner_max_ms'])}|{number(s['mpc_max_ms'])}|{number(s['sender_max_ms'])}|")
    lines += ['', 'Wang没有外部MPC，表中“—”不是零耗时；PX4内部控制计算耗时没有用主机发送耗时代替。',
        '合格要求：七窗各一次正向、整机完整离面、返航误差<0.55m且速度<1m/s、无接触、',
        '10ms真值整机球净空非负、房间界内、主机规划/控制/发送调用均<100ms。',
        '保守球净空为负并不自动等于物理碰撞；两者分开报告。名义12m/s、8m/s²、42°超限仅作诊断。',
        '这些是仿真和采样证据，不是连续域安全证明，也不是最坏情况实时性证明。',
        'pilot目录的开发联调不计入矩阵；只允许日志证明的基础设施故障重试，算法失败不重试。','',
        '## 图与原始记录','',
        '![固定平面赛道](figures/course.png)','',
        '![成功与圈时](figures/comparison.png)','',
        '以下固定展示1Hz、2m、相位0条件，不按成败挑选。','',
        '![速度与路径](figures/hard_speed_path.png)','',
        '![穿越位置](figures/hard_crossing_points.png)','',
        '[逐趟汇总](condition_summary.json)、[全部尝试](summary.json)、[协议与源码清单](protocol.json)、',
        '[配对比较与失败诊断](comparison.json)、[该条件逐门穿越点](hard_crossings.json)。','']
    (batch/'REPORT.md').write_text('\n'.join(lines))
    plot(batch,rows)
    print(json.dumps(dict(summary=summary,common_valid=len(paired),paired=paired),ensure_ascii=False,indent=2))


if __name__=='__main__': main()
