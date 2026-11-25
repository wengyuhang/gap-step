"""One three-row result table, with failures retained and computation separate."""
import csv
import json
import os
from pathlib import Path
import numpy as np
from .scenario import LEVELS

METHODS=('ours','center','dynamic_togt','dynamic_togt_center')
NAMES=dict(ours='本文自由点',center='本文中心',dynamic_togt='动态TOGT自由点',dynamic_togt_center='动态TOGT中心')
LABELS=dict(completed_no_contact='无碰撞完赛',collision='碰撞',missed_gate='漏窗',not_completed='未完成',planning_not_found='未找到合格规划',evidence_failure='证据不足',pending='待完成')


def report(out,rows,final=False):
    data=[];lines=['# 基础组：全局七窗混合动态竞速','',f"状态：{'完成' if final else '进行中'}，{len(rows)}/12条件记录。",'',
        '固定紧凑基础布局；四个水平平移窗、三个外支点摆动窗，平面固定。各条件原初相一次；所有时间包含起飞执行至七窗整机退出、返航及减速。',
        '赛前一次全局规划，不滚动、不在飞行中重规划。只有无碰撞完赛才填写有效竞速时间；规划未找到与碰撞分别记录。','',
        '|运动等级|本文自由点|本文中心|动态TOGT自由点|动态TOGT中心|','|---|---:|---:|---:|---:|']
    for level,(f,a) in LEVELS.items():
        pair={r['variant']:r for r in rows if r['case']['level']==level};row=dict(level=level,frequency_hz=f,amplitude_m=a);cells=[]
        for m in METHODS:
            r=pair.get(m,{});v=r.get('verdict','pending');t=r.get('flight_time_s') if r.get('passed') else None
            row[m+'_verdict']=v;row[m+'_time_s']=t;cells.append(f'{t:.3f} s，无碰撞' if t is not None else LABELS[v])
        for free,center,key in [('ours','center','ours_saving_percent'),('dynamic_togt','dynamic_togt_center','togt_saving_percent'),('ours','dynamic_togt','ours_vs_togt_saving_percent')]:
            x,y=pair.get(free,{}),pair.get(center,{})
            row[key]=100*(y['flight_time_s']-x['flight_time_s'])/y['flight_time_s'] if x.get('passed') and y.get('passed') else None
        data.append(row);lines.append('|'+('静态' if level=='static' else f'{f:g} Hz／{a:g} m')+'|'+'|'.join(cells)+'|')
    with (out/'comparison.csv').open('w',newline='') as stream:
        w=csv.DictWriter(stream,fieldnames=list(data[0]));w.writeheader();w.writerows(data)
    analysis=dict(final=final,records=len(rows),total=12,conditions=data,methods={m:dict(records=sum(r['variant']==m for r in rows),completions=sum(r['variant']==m and r['passed'] for r in rows),collisions=sum(r['variant']==m and r.get('collision_detected',False) for r in rows)) for m in METHODS})
    (out/'analysis.json').write_text(json.dumps(analysis,ensure_ascii=False,indent=2)+'\n')
    lines+=['','## 计算与原始证据','', '|等级|方法|结果|建图 s|四初值求解 s|名义时间 s|MPC最大 ms|来源|','|---|---|---|---:|---:|---:|---:|---|']
    for r in sorted(rows,key=lambda r:(list(LEVELS).index(r['case']['level']),METHODS.index(r['variant']))):
        p=Path(r['directory']);raw=json.loads((p/'result.json').read_text());build=raw.get('global_build_s',raw.get('build_s'));fmt=lambda v:'—' if v is None else f'{v:.3f}'
        lines.append('|'+r['case']['level']+'|'+NAMES[r['variant']]+'|'+LABELS[r['verdict']]+'|'+fmt(build)+'|'+fmt(r.get('global_planning_s'))+'|'+fmt(r.get('nominal_time_s'))+'|'+fmt(r.get('mpc_max_ms'))+'|'+('[保留原始实飞]' if r.get('reused') else '[原始记录]')+'('+os.path.relpath(p/'result.json',out)+')|')
    lines+=['','静态TOGT两条沿用已验证实飞，核对源差异、场景和证据哈希后保留，不重飞择优。完整冻结设置见protocol.json和PROTOCOL.md；继承来源见reuse_audit.json。',
        '四方法分别沿用原目标/约束与选解规则；本文两方只差选点自由度，TOGT两方只差选点自由度。仅双方成功时比较圈时；一次仿真不估计随机成功概率，不把未找到规划等同物理无解。']
    if (out/'actual_flights.png').exists():lines+=['','![实际闭环轨迹](actual_flights.png)']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def figures(out,rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from .scenario import build_case,Case
    fig,axes=plt.subplots(1,3,figsize=(15,4.9));colors=['#1764ab','#7ba6cf','#df7b21','#e8b77d']
    for ax,level in zip(axes,LEVELS):
        track,_=build_case(Case('original',level));anchors=np.vstack([track.start,*[w.center0 for w in track.windows],track.goal]);ax.plot(*anchors[:,:2].T,':',c='.6',lw=1)
        for i,w in enumerate(track.windows):
            ax.scatter(*w.center0[:2],c='black',s=14);ax.annotate(str(i+1),w.center0[:2],xytext=(3,3),textcoords='offset points',fontsize=8)
        for m,c in zip(METHODS,colors):
            r=next((r for r in rows if r['case']['level']==level and r['variant']==m),None)
            if r is None:continue
            f=Path(r['directory'])/'truth_trace.npz'
            if not f.exists():continue
            p=np.load(f)['positions'];label=m+(' %.3fs'%r['flight_time_s'] if r['passed'] else ' '+r['verdict'])
            ax.plot(*p[::3,:2].T,c=c,lw=1.2,label=label)
        ax.scatter(*track.start[:2],marker='*',s=70,c='black');ax.set(title=level,xlabel='x (m)',ylabel='y (m)');ax.set_aspect('equal');ax.grid(alpha=.15);ax.legend(fontsize=6)
    fig.tight_layout();fig.savefig(out/'actual_flights.png',dpi=180);fig.savefig(out/'actual_flights.pdf');plt.close(fig)
