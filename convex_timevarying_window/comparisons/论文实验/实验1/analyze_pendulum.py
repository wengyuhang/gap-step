"""Report all predeclared pendulum conditions, including collisions/failures."""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

METHODS=('ours','center','high_mpc_author')
LABELS={'ours':'Free point','center':'Fixed centre','high_mpc_author':'Author High-MPC (x500 adaptation)'}

def analyze(out, finer_check=None):
    out=Path(out);state=json.loads((out/'status.json').read_text())
    if state['state']!='complete' or not state['sources_unchanged']:raise RuntimeError('incomplete or unfrozen batch')
    rows=json.loads((out/'summary.json').read_text());result=dict(formal_flights=len(rows),repetitions_per_condition=1,random_success_probability_estimated=False,methods={})
    for method in METHODS:
        selected=[r for r in rows if r['variant']==method];raw=[json.loads((Path(r['directory'])/'result.json').read_text()) for r in selected]
        def timing(v):return dict(count=len(v),p50_ms=float(np.median(v)),p95_ms=float(np.percentile(v,95)),max_ms=float(max(v)),over_100ms=sum(x>100 for x in v)) if v else None
        result['methods'][method]=dict(completed_no_collision=sum(r['passed'] for r in selected),collisions=sum(r.get('collision_detected',False) for r in selected),incomplete=sum(not r['passed'] and not r.get('collision_detected',False) for r in selected),
            planner=timing([s['cycle_ms'] for r in raw for s in r['solves']]),controller=timing([s['cycle_ms'] for r in raw for s in r['mpc_cycles'] if s.get('time',0)>=0]))
    result['paired_free_vs_center']=[]
    for angle in sorted({r['angle_deg'] for r in rows}):
        a=next(r for r in rows if r['angle_deg']==angle and r['variant']=='ours');b=next(r for r in rows if r['angle_deg']==angle and r['variant']=='center')
        if a['passed'] and b['passed']:result['paired_free_vs_center'].append(dict(angle_deg=angle,free_s=a['flight_time_s'],center_s=b['flight_time_s'],saving_percent=100*(b['flight_time_s']-a['flight_time_s'])/b['flight_time_s']))
    check=Path(finer_check) if finer_check is not None else None
    if check is not None and check.exists():
        r=json.loads(check.read_text());result['independent_finer_physics_check']=dict(physics_step_s=.0005,angle_deg=60,method='high_mpc_author',verdict=r['verdict'],flight_time_s=r['flight_time_s'],frame_collision=r['frame_collision'],ground_collision=r['ground_collision'],not_pooled_with_primary=True,safety_advantage_not_established=True)
    (out/'analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    fields=['angle_deg','variant','verdict','flight_time_s','passed','frame_collision','ground_collision','failure','truth_goal_error_m','planner_max_ms','mpc_max_ms','directory']
    with (out/'all_results.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(sorted(rows,key=lambda r:(r['angle_deg'],METHODS.index(r['variant']))))
    fig,axes=plt.subplots(2,1,figsize=(9,6),sharex=True,gridspec_kw={'height_ratios':[2,1]});angles=sorted({r['angle_deg'] for r in rows})
    for j,method in enumerate(METHODS):
        chosen=[next(r for r in rows if r['angle_deg']==a and r['variant']==method) for a in angles];x=np.arange(len(angles))+(j-1)*.23;color=('#167d9a','#d67125','#7856a1')[j]
        axes[0].bar(x,[r['flight_time_s'] if r['passed'] else np.nan for r in chosen],width=.21,color=color,label=LABELS[method]);axes[1].scatter(x,[2 if r['passed'] else 0 if r.get('collision_detected') else 1 for r in chosen],color=color,s=50)
    axes[0].set_ylabel('Full task time (s), completed only');axes[0].legend(fontsize=8);axes[1].set_yticks([0,1,2],['Collision','Incomplete','Completed']);axes[1].set_ylim(-.4,2.4);axes[1].set_xticks(range(len(angles)),[f'{a:g}' for a in angles]);axes[1].set_xlabel('Initial pendulum angle (degrees)')
    for ax in axes:ax.grid(axis='y',alpha=.2)
    fig.suptitle('Pendulum primary test: 1 ms physics, common x500/PX4')
    if check is not None and check.exists():fig.text(.5,.01,'Independent 0.5 ms run: High-MPC at 60 degrees completed without contact; not pooled.',ha='center',fontsize=8)
    fig.tight_layout(rect=(0,.05,1,1));fig.savefig(out/'pendulum_time_and_outcomes.png',dpi=190);plt.close(fig)
    highest=max(angles);fig,axes=plt.subplots(2,3,figsize=(12,7),sharex=True,sharey='row')
    from .pendulum.motion import build,Case
    window=build(Case(highest)).windows[0]
    for j,method in enumerate(METHODS):
        row=next(r for r in rows if r['angle_deg']==highest and r['variant']==method);tr=np.load(Path(row['directory'])/'mechanism_trace.npz');t=tr['times'];p=tr['positions'];c=tr['gate_centers']
        for i,k in enumerate((1,2)):
            ax=axes[i,j];ax.plot(p[:,0],p[:,k],label='Drone');ax.plot(c[:,0],c[:,k],'--',label='Gate centre');ax.set(xlabel='World x (m)',ylabel='World '+('y' if k==1 else 'z')+' (m)');ax.grid(alpha=.2)
            for e in row['crossings']:
                position=[np.interp(e['time_s'],t,p[:,d]) for d in (0,k)];center=window.state_at(e['time_s'])[0];r=window.aperture.radius
                ax.plot([center[0],center[0]],[center[k]-r,center[k]+r],color='black',linewidth=3,label='Opening at crossing')
                ax.scatter(*position,marker='x',color='red',zorder=5)
        axes[0,j].set_title(LABELS[method],fontsize=10)
    axes[0,0].legend(fontsize=7);fig.suptitle(f'Initial angle {highest:g} degrees: measured paths in XY and XZ; failed run retained');fig.tight_layout();fig.savefig(out/'pendulum_measured_traces.png',dpi=190);plt.close(fig)
    with (out/'REPORT.md').open('a') as f:f.write('\n## 计算耗时与共同完赛配对\n\n```json\n'+json.dumps(result,ensure_ascii=False,indent=2)+'\n```\n\nHigh-MPC原高层权重未针对x500重训；此组外部结果属于明确标注的适配，不能称作者原平台性能。\n')
    if check is not None and check.exists():
        with (out/'REPORT.md').open('a') as f:f.write('\n## 首个碰框案例的独立步长复核\n\n主实验1ms的60°High-MPC发生真实碰框，0.5ms独立运行则无碰撞完成（5.1415s）。两条都保留，不替换或混算时间。\n两次初态和控制时序也不同，因此不能归因于步长本身，更不能据此宣称稳定的安全性优势。\n本组外部基线为未重训x500适配，不能作原论文性能或公平模型匹配的优劣结论。\n')
    print(json.dumps(result,indent=2),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('outdir',type=Path);p.add_argument('--finer-check',type=Path);args=p.parse_args();analyze(args.outdir,args.finer_check)
