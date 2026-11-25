"""Read-only analysis of frozen inputs; generate separate reports and figures."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track

def main():
    p=argparse.ArgumentParser();p.add_argument('--matrix',type=Path,required=True);p.add_argument('--outdir',type=Path,required=True);args=p.parse_args()
    args.outdir.mkdir(parents=True,exist_ok=False)
    runs=[(d,json.loads((d/'summary.json').read_text())) for d in sorted(args.matrix.iterdir()) if (d/'summary.json').exists()]
    seven=[r for r in runs if not r[1]['condition']['single']];single=[r for r in runs if r[1]['condition']['single']]
    first=runs[0][1]['source_sha256'];identical=all(d['source_sha256']==first for _,d in runs)
    assert identical,'Do not pool different source snapshots'
    all_ms=[];seven_ms=[];table=[];stage_counts={};avoid=short=0;accepted=[];rejected=0
    for directory,d in runs:
        logs=[json.loads(x) for x in (directory/'cycles.jsonl').read_text().splitlines()]
        all_ms.extend(x['cycle_ms'] for x in logs)
        if not d['condition']['single']:
            seven_ms.extend(x['cycle_ms'] for x in logs)
            for x in logs:stage_counts[x['stage']]=stage_counts.get(x['stage'],0)+1
            avoid+=sum(x['avoiding'] for x in logs);short+=sum(x['reactive_filter'] for x in logs)
            accepted.extend(x['solve_ms'] for x in d['updates'] if x['accepted']);rejected+=sum(not x['accepted'] for x in d['updates'])
        trace=np.load(directory/'trace.npz')['commands'];speed=np.linalg.norm(trace[:,4:7],axis=1)
        table.append(dict(condition=directory.name,completed=d['completed'],gates=d['gates'],time=d['time'],
                          min_clear=d['minimum_clearance'],max_speed=float(speed.max()),mppi_max=d['cycles']['max']))
    stats=dict(source_identical=identical,source_files=len(first),single_completed=sum(x['completed'] for _,x in single),
               single_count=len(single),seven_completed=sum(x['completed'] for _,x in seven),seven_count=len(seven),
               mppi_count=len(all_ms),mppi_max=max(all_ms),mppi_over100=sum(x>=100 for x in all_ms),
               seven_median=float(np.median(seven_ms)),seven_p99=float(np.quantile(seven_ms,.99)),
               seven_over20=sum(x>20 for x in seven_ms),seven_cycles=len(seven_ms),
               stage_counts=stage_counts,avoidance_cycles=avoid,single_step_fallback_cycles=short,
               accepted_togt_count=len(accepted),accepted_togt_max=max(accepted),rejected_togt_attempts=rejected,
               conditions=table)
    (args.outdir/'statistics.json').write_text(json.dumps(stats,indent=2)+'\n')
    colors=plt.get_cmap('tab10').colors;fig,axes=plt.subplots(2,1,figsize=(10,6),sharex=True,layout='constrained')
    labels=[]
    for i,(directory,d) in enumerate(seven):
        label=f"rate {d['condition']['motion_rate']:g}, phase {d['condition']['phase']:g} deg";labels.append(label)
        z=np.load(directory/'trace.npz')['rows'];axes[0].plot(z[:,0],np.linalg.norm(z[:,4:7],axis=1),lw=1,color=colors[i],label=label)
        axes[1].plot(z[:,0],z[:,-3],lw=.9,color=colors[i])
    axes[0].axhline(12,color='black',ls='--',lw=.8);axes[0].set_ylabel('Speed [m/s]');axes[0].legend(ncol=2,fontsize=8)
    axes[1].axhline(0,color='black',ls='--',lw=.8);axes[1].set_ylim(-.05,2.);axes[1].set_ylabel('Interval clearance [m]');axes[1].set_xlabel('Nominal flight time [s]')
    fig.suptitle('Six-state nominal model only — not PX4; clearance includes 0.35 m error allowance')
    fig.savefig(args.outdir/'speed_clearance.png',dpi=160);plt.close(fig)
    track,_=build_track();fig,axes=plt.subplots(2,4,figsize=(11,6),layout='constrained')
    for gate,ax in enumerate(axes.ravel()[:7]):
        points=track.windows[track.order[gate]].aperture.boundary_points(180);points=np.vstack((points,points[0]))
        ax.plot(points[:,0],points[:,1],color='.3',lw=1.3);ax.plot(0,0,'+',color='.5')
        for i,(_,d) in enumerate(seven):
            for crossing in d['center_crossings']:
                if crossing['gate']==gate:ax.scatter(*crossing['local_point'],s=26,color=colors[i],label=labels[i])
        ax.set_title(f'Gate {gate+1}');ax.set_aspect('equal');ax.set_xlim(-1.65,1.65);ax.set_ylim(-1.65,1.65);ax.grid(alpha=.2)
    axes.ravel()[-1].axis('off');handles,labels=axes.ravel()[0].get_legend_handles_labels();axes.ravel()[-1].legend(handles,labels,loc='center',fontsize=9)
    fig.suptitle('Actual centre-plane crossings in moving gate coordinates [m]\nThese are not full-body exit events; body clearance is audited separately')
    fig.savefig(args.outdir/'crossing_positions.png',dpi=160);plt.close(fig)
    lines=['# 冻结名义模型矩阵','',
           '模型：六状态双积分器，求解期间仿真时钟暂停。不是PX4，不是连续安全证明，也不与旧PX4论文基线排圈速。','',
           '| 条件 | 完成 | 门数 | 飞行时间(s) | 区间最小余量(m) | 最高速度(m/s) | MPPI最大(ms) |',
           '|---|---:|---:|---:|---:|---:|---:|']
    for r in table:lines.append(f"| {r['condition']} | {r['completed']} | {r['gates']} | {r['time']:.2f} | {r['min_clear']:.4f} | {r['max_speed']:.2f} | {r['mppi_max']:.2f} |")
    lines+=['',f"全部{len(runs)}条件的{len(first)}个源码快照哈希一致。MPPI总计{len(all_ms)}拍，超过100 ms：{stats['mppi_over100']}拍。",
            f"七窗中位/P99：{stats['seven_median']:.2f}/{stats['seven_p99']:.2f} ms；{stats['seven_over20']}/{len(seven_ms)}拍超过20 ms，不宣称严格50 Hz。",
            f"七窗TOGT接收{len(accepted)}次，接收值最大{max(accepted):.2f} ms；另外{rejected}次未返回参考。该版未记录失败求解的实际完整耗时，不能据此说所有TOGT调用均在100 ms内。",
            f"阶段拍数：{stage_counts}；普通段自主避让{avoid}拍；短步后备动作{short}拍。",'',
            '四个固定条件不是总体成功率估计；不包含执行延迟、定位误差实现或姿态/电机动力学。',
            '中心过平面坐标由5 ms邻点插值，不把它当整机出门。整体目标仍需真实执行层验证。','',
            '![速度与净空](speed_clearance.png)','![穿越位置](crossing_positions.png)']
    (args.outdir/'REPORT.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps({k:v for k,v in stats.items() if k!='conditions'},indent=2))

if __name__=='__main__':main()
