"""Complete outcome table, planning/execution costs and paper figures."""
import json,os
from pathlib import Path
import csv
import numpy as np
from convex_timevarying_window.comparisons.single_gate_constant_velocity.run import save
from .scenario import LAYOUTS,LEVELS,Case,manifest,build_case
LABELS={'completed_no_contact':'无碰撞完赛','collision':'碰撞','missed_gate':'漏窗','not_completed':'未完成','planning_not_found':'未找到合格规划','evidence_failure':'证据不足','pending':'待完成'}

def report(out,rows,final=False):
    out=Path(out);data=[]
    for layout in LAYOUTS:
        for level in LEVELS:
            p={r['variant']:r for r in rows if r['case']==dict(layout=layout,level=level)};d=dict(layout=layout,level=level)
            for v in ('ours','center','wang'):
                r=p.get(v,{});d[v+'_verdict']=r.get('verdict','pending');d[v+'_time_s']=r.get('flight_time_s') if r.get('passed') else None
            a,b=p.get('ours',{}),p.get('center',{});d['free_saving_percent']=100*(b['flight_time_s']-a['flight_time_s'])/b['flight_time_s'] if a.get('passed') and b.get('passed') else None
            data.append(d)
    with (out/'comparison.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(data[0]));w.writeheader();w.writerows(data)
    save(out/'analysis.json',dict(final=final,records=len(rows),total=27,conditions=data,
        methods={v:dict(records=sum(r['variant']==v for r in rows),no_collision_completed=sum(r['variant']==v and r.get('passed',False) for r in rows),collisions=sum(r['variant']==v and r.get('collision_detected',False) for r in rows)) for v in ('ours','center','wang')}))
    lines=['# 实验2：全局七窗混合运动竞速','',f"状态：{'完成' if final else '进行中'}，{len(rows)}/27条记录。",'',
        '本文自由点与强制中心均一次联合规划七窗及返航，没有中间停车点、没有滚动视域或飞行中重规划。Wang保留论文局部结构，单独标注为独立复现。',
        '每条赛道含4个平移矩形窗与3个绕外部支点摆动圆窗，初相零；静态/1Hz2m/3Hz4m三档，各一次。',
        '竞速时间为执行开始至七窗后返航及减速。全局赛前建图/求解耗时另表列出。','',
        '|赛道|等级|自由结果|自由完整s|中心结果|中心完整s|Wang复现结果|Wang完整s|自由对中心节省|',
        '|---|---|---|---:|---|---:|---|---:|---:|']
    for d in data:
        values=[d['layout'],d['level']]
        for v in ('ours','center','wang'):values.extend([LABELS.get(d[v+'_verdict'],d[v+'_verdict']),f"{d[v+'_time_s']:.3f}" if d[v+'_time_s'] is not None else '—'])
        values.append(f"{d['free_saving_percent']:.2f}%" if d['free_saving_percent'] is not None else '—');lines.append('| '+' | '.join(values)+' |')
    lines+=['','## 规划与执行成本','',
        '|赛道/等级|方法|全局四初值求解s|名义轨迹s|在线请求数|在线规划最大ms|MPC最大ms|原始记录|',
        '|---|---|---:|---:|---:|---:|---:|---|']
    for r in rows:
        fmt=lambda x:f'{x:.3f}' if x is not None else '—'
        rel=os.path.relpath(Path(r['directory'])/'result.json',out)
        values=[r['case']['layout']+'/'+r['case']['level'],r['variant'],fmt(r.get('global_planning_s')),fmt(r.get('nominal_time_s')),str(r.get('planner_requests',0)),fmt(r.get('planner_max_ms')),fmt(r.get('mpc_max_ms')),f'[原始结果]({rel})']
        lines.append('| '+' | '.join(values)+' |')
    lines+=['','碰撞依据框/地原生接触，未找到规划不等同物理不可行。只在自由/中心双方无碰撞完赛时计算时间差。',
        '每条件各一次不估计随机成功概率；源码、八路接触发布端、真实窗口位姿与ULog独立验收保留。',
        '全局优化不是全局最优证明，名义局部包络和17节点/段动力学不是连续闭环证书。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')

def figures(out,rows):
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    out=Path(out);fig,axes=plt.subplots(1,3,figsize=(13,4.4),subplot_kw={'projection':'3d'})
    for ax,layout in zip(axes,LAYOUTS):
        track,_=build_case(Case(layout,'static'));p=np.vstack([track.start,*[w.center0 for w in track.windows],track.goal]);ax.plot(*p.T,'k--',alpha=.4)
        for i,w in enumerate(track.windows):
            q=w.boundary_at(0);q=np.vstack([q,q[0]]);ax.plot(*q.T,color='#277da8' if w.motion.kind=='swing' else '#df7e24');ax.text(*w.center0,str(i+1),fontsize=8)
        ax.scatter(*track.start,c='green',marker='*',s=60);ax.set_title(layout);ax.set_xlabel('x [m]');ax.set_ylabel('y [m]');ax.set_zlabel('z [m]');ax.set_box_aspect((1,1,.45))
    fig.tight_layout();fig.savefig(out/'layouts.png',dpi=180);fig.savefig(out/'layouts.pdf');plt.close(fig)
    if not rows:return
    fig,axes=plt.subplots(1,3,figsize=(12,3.5),sharey=True)
    for ax,layout in zip(axes,LAYOUTS):
        for v,label,color in [('ours','Free','C0'),('center','Center','C1'),('wang','Wang independent','C2')]:
            yy=[next((r.get('flight_time_s') if r.get('passed') else np.nan for r in rows if r['case']==dict(layout=layout,level=l) and r['variant']==v),np.nan) for l in LEVELS]
            ax.plot(range(3),yy,'o-',label=label,color=color)
        ax.set_xticks(range(3),['Static','1Hz / 2m','3Hz / 4m']);ax.set_title(layout);ax.grid(alpha=.2)
    axes[0].set_ylabel('Full race time including return [s]');axes[-1].legend(fontsize=8);fig.tight_layout();fig.savefig(out/'race_times.png',dpi=180);fig.savefig(out/'race_times.pdf');plt.close(fig)
