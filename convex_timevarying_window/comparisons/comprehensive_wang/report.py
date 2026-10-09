"""Read-only aggregation of frozen flights; no success-only overall ranking."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from .registry import FREQUENCIES,AMPLITUDES,HARD
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import build_case,Case


def aggregate(rows):
    result={}
    for method in ('ours','wang','center','no_soft','no_dense'):
        group=[r for r in rows if r['trial']['method']==method]
        valid=[r for r in group if not r['infrastructure_failure'] and not r.get('audit_error')]
        success=[r for r in valid if r['race_passed']]
        result[method]=dict(scheduled=len(group),valid=len(valid),infrastructure_missing=len(group)-len(valid),
            race_success=len(success),full_task_success=sum(r['main_passed'] for r in valid),
            conditional_median_race_s=float(np.median([r['race_time_s'] for r in success])) if success else None,
            collisions=sum(bool(r.get('collision_detected')) for r in valid))
    by_pair={}
    for r in rows:
        if r['trial']['stage']=='main':by_pair.setdefault(r['pair'],{})[r['trial']['method']]=r
    paired=[]
    for name,group in by_pair.items():
        if set(group)=={'ours','wang'} and all(r.get('race_passed') for r in group.values()):
            paired.append(dict(pair=name,ours_s=group['ours']['race_time_s'],wang_s=group['wang']['race_time_s'],
                ours_minus_wang_s=group['ours']['race_time_s']-group['wang']['race_time_s']))
    return dict(methods=result,successful_pairs=paired,
        paired_median_difference_s=float(np.median([p['ours_minus_wang_s'] for p in paired])) if paired else None)


def main():
    p=argparse.ArgumentParser();p.add_argument('batch',type=Path);a=p.parse_args();out=a.batch
    status=json.loads((out/'status.json').read_text());rows=json.loads((out/'summary.json').read_text())
    summary=aggregate(rows);summary['batch_status']=status
    (out/'aggregate.json').write_text(json.dumps(summary,indent=2)+'\n')
    fig,axes=plt.subplots(1,2,figsize=(9,3.8),layout='constrained')
    for ax,method in zip(axes,('ours','wang')):
        data=np.full((3,3),np.nan)
        for fi,f in enumerate(FREQUENCIES):
            for ai,amp in enumerate(AMPLITUDES):
                rr=[r for r in rows if r['trial']['method']==method and r['trial']['frequency']==f and r['trial']['amplitude']==amp and not r['infrastructure_failure'] and not r.get('audit_error')]
                if rr:
                    data[fi,ai]=sum(r['race_passed'] for r in rr)/len(rr)
                    ax.text(ai,fi,f"{sum(r['race_passed'] for r in rr)}/{len(rr)}",ha='center',va='center',color='black')
        im=ax.imshow(data,vmin=0,vmax=1,cmap='YlGn');ax.set_title(method)
        ax.set_xticks(range(3),AMPLITUDES);ax.set_yticks(range(3),FREQUENCIES)
        ax.set_xlabel('Amplitude (m, one-sided)');ax.set_ylabel('Frequency (Hz)')
    fig.colorbar(im,ax=axes,label='Safe race completion fraction');fig.savefig(out/'success_heatmap.png',dpi=170);plt.close(fig)
    fig,ax=plt.subplots(figsize=(7,4),layout='constrained');tt=np.linspace(0,60,601)
    for method in ('ours','wang'):
        valid=[r for r in rows if r['trial']['method']==method and not r['infrastructure_failure'] and not r.get('audit_error')]
        if valid:
            yy=[sum(r['race_passed'] and r['race_time_s']<=t for r in valid)/len(valid) for t in tt]
            ax.plot(tt,yy,label=f'{method} (n={len(valid)})')
    ax.set(xlabel='Race time (s)',ylabel='Fraction safely completed by this time',ylim=(0,1.02));ax.legend();ax.grid(alpha=.25)
    fig.savefig(out/'safe_completion_vs_time.png',dpi=170);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(9,4),layout='constrained')
    for method in ('ours','wang'):
        values=[];control=[]
        for r in rows:
            if r['trial']['method']!=method or r['infrastructure_failure']:continue
            raw=json.loads((Path(r['directory'])/'result.json').read_text())
            values += [s['cycle_ms'] for s in raw['solves']]
            control += [s['cycle_ms'] for s in raw['mpc_cycles']]
        for ax,data in zip(axes,(values,control)):
            if data:
                x=np.sort(data);ax.plot(x,np.arange(1,len(x)+1)/len(x),label=method)
    for ax,title in zip(axes,('Full planner cycle','MPC cycle')):
        ax.axvline(100,color='red',ls='--',label='100 ms');ax.set(title=title,xlabel='Latency (ms)',ylabel='Empirical CDF');ax.legend();ax.grid(alpha=.25)
    fig.savefig(out/'online_latency.png',dpi=170);plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,4),layout='constrained')
    for offset,method in enumerate(('ours','center','no_soft','no_dense')):
        yy=[]
        for f,amp in HARD:
            group=[r for r in rows if r['trial']['method']==method and r['trial']['frequency']==f and r['trial']['amplitude']==amp and not r['infrastructure_failure'] and not r.get('audit_error')]
            yy.append(sum(r['race_passed'] for r in group)/len(group) if group else np.nan)
        ax.bar(np.arange(3)+(offset-1.5)*.18,yy,width=.18,label=method)
    ax.set_xticks(range(3),[f'{f}Hz / {amp}m' for f,amp in HARD]);ax.set(ylabel='Safe race completion fraction',ylim=(0,1.15));ax.legend(ncol=2)
    fig.savefig(out/'ablation_success.png',dpi=170);plt.close(fig)
    # Fixed representative, never select the best-looking run after inspection.
    representative=[r for r in rows if r['trial']['stage']=='main' and r['trial']['frequency']==1 and r['trial']['amplitude']==2 and r['trial']['phase_degrees']==0]
    fig,ax=plt.subplots(figsize=(8,4),layout='constrained')
    for r in representative:
        file=Path(r['directory'])/'truth_audit.npz'
        if file.exists():
            z=np.load(file);ax.plot(z['times'],z['speed'],label=r['trial']['method'])
            for e in r.get('body_exits',[]):ax.axvline(e['time'],alpha=.12)
    ax.set(xlabel='Race clock (s)',ylabel='Truth speed (m/s)',title='Preselected: 1Hz / 2m / initial phase');ax.legend();ax.grid(alpha=.25)
    fig.savefig(out/'representative_speed.png',dpi=170);plt.close(fig)
    track,_=build_case(Case(1.,2.,0.))
    fig,axes=plt.subplots(2,4,figsize=(12,6),layout='constrained')
    for gate,(ax,w) in enumerate(zip(axes.flat,track.windows)):
        if w.aperture.kind=='circle':
            ang=np.linspace(0,2*np.pi,201);outline=w.aperture.radius*np.column_stack([np.cos(ang),np.sin(ang)])
        else:
            vertices=w.aperture.physical_vertices;outline=np.vstack([vertices,vertices[0]])
        ax.plot(outline[:,0],outline[:,1],color='black');ax.scatter([0],[0],marker='+',color='gray',label='center')
        for r in representative:
            events=[e for e in r.get('events',[]) if e['gate']==gate and e['inside']]
            if events:
                q=np.array([e['q'] for e in events]);ax.scatter(q[:,0],q[:,1],label=r['trial']['method'])
        ax.set(title=f'Gate {gate+1}',xlabel='Local u (m)',ylabel='Local v (m)');ax.set_aspect('equal');ax.grid(alpha=.2)
    axes.flat[-1].axis('off');axes.flat[0].legend(fontsize=8)
    fig.savefig(out/'representative_crossing_points.png',dpi=170);plt.close(fig)
    text=['# 完整对比实验结果','',f"执行状态：{status['state']}；记录 {len(rows)}/{status['total']} 条件。",'',
        '均为独立实现/平台适配，不是作者官方性能。全部原始初相位、每条件一次，无相位扫描或重复。失败中止时刻不是成绩。',
        '初相位此前已用于开发；本轮不是留出相位验证，不推断随机成功率或重复稳定性。','',
        '| 方法 | 有效设施条件 | 安全过七窗 | 完整任务通过 | 实际接触 | 成功条件中位时间/s |',
        '|---|---:|---:|---:|---:|---:|']
    for method,s in summary['methods'].items():
        text.append(f"| {method} | {s['valid']} | {s['race_success']} | {s['full_task_success']} | {s['collisions']} | {s['conditional_median_race_s']} |")
    text+=['',f"双方共同成功配对数：{len(summary['successful_pairs'])}；配对时间差中位数：{summary['paired_median_difference_s']} s。",
        '若无共同成功，不报告速度优势。条件成功中位数不能直接用于整体速度排名。','',
        '![安全完成比例](success_heatmap.png)','![安全完成随时间变化](safe_completion_vs_time.png)',
        '![实时计算](online_latency.png)','![消融](ablation_success.png)','![预指定速度案例](representative_speed.png)',
        '![预指定局部穿越位置](representative_crossing_points.png)','',
        'summary.json/attempts.json保留逐条件与所有设施重试；protocol.json/source_snapshot为冻结版本。',
        'race_passed只审核七窗竞速前缀；main_passed另含返航。零接触不等于正包络余量，4ms采样不是连续认证。']
    (out/'REPORT.md').write_text('\n'.join(text)+'\n')


if __name__=='__main__':main()
