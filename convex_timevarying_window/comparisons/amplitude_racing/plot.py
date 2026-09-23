"""Static amplitude-study figures from actual PX4 telemetry and audits."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track


def plot(root):
    root=Path(root);rows=json.loads((root/'summary.json').read_text())
    rows=[r for r in rows if not r.get('infrastructure_failure')]
    grouped=[]
    for key in sorted({(r['rate'],r['translation_scale'],r['rotation_scale']) for r in rows}):
        rr=[r for r in rows if (r['rate'],r['translation_scale'],r['rotation_scale'])==key]
        wins=[r for r in rr if r.get('physical_success')]
        grouped.append(dict(rate=key[0],translation_scale=key[1],rotation_scale=key[2],n=len(rr),
            successes=len(wins),realtime_successes=sum(r.get('realtime_success',False) for r in rr),
            mean_finish_s=float(np.mean([r['flight_time_s'] for r in wins])) if wins else None,
            max_cycle_ms=max((r['cycle_ms']['max'] for r in rr if r.get('cycle_ms')),default=None),
            min_clearance_m=min((r['min_clearance_m'] for r in rr if 'min_clearance_m' in r),default=None),
            max_passed_gates=max((r.get('gates',0) for r in rr),default=0),
            collisions=sum(r.get('collision',False) for r in rr)))
    (root/'grouped.json').write_text(json.dumps(grouped,indent=2)+'\n')
    fig,axes=plt.subplots(1,3,figsize=(13,4),layout='constrained')
    for rate,color in [(1,'tab:blue'),(10,'tab:orange')]:
        g=[r for r in grouped if r['rate']==rate and r['translation_scale']==r['rotation_scale']]
        x=[r['translation_scale'] for r in g]
        axes[0].plot(x,[r['successes']/r['n']*100 for r in g],'o-',color=color,label=f'{rate}x frequency')
        axes[1].plot(x,[r['mean_finish_s'] if r['mean_finish_s'] is not None else np.nan for r in g],'o-',color=color)
        axes[2].plot(x,[r['max_cycle_ms'] for r in g],'o-',color=color)
        for r in g:axes[0].annotate(f"{r['successes']}/{r['n']}",(r['translation_scale'],100*r['successes']/r['n']),xytext=(0,8 if rate==1 else -17),textcoords='offset points',ha='center',color=color)
    axes[0].set(ylabel='Physical success (%)',ylim=(-12,115));axes[0].legend()
    axes[1].set(ylabel='Successful finish time (s)',title='Missing point = no successful finish')
    axes[2].set(ylabel='Max full planning cycle (ms)');axes[2].axhline(100,color='red',ls='--',lw=1)
    for ax in axes:ax.set(xlabel='Translation AND RPY amplitude multiplier',xticks=[1,2,4]);ax.grid(alpha=.2)
    fig.suptitle('Same algorithm, x500, course and safety margins; all fixed phase conditions')
    fig.savefig(root/'amplitude_summary.png',dpi=180);plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(15,4),layout='constrained')
    labels=['Baseline','Translation only 4x','Rotation only 4x','Both 4x'];pairs=[(1,1),(4,1),(1,4),(4,4)]
    gg=[next((g for g in grouped if g['rate']==10 and (g['translation_scale'],g['rotation_scale'])==pair),None) for pair in pairs]
    axes[0].bar(range(4),[g['successes'] if g else np.nan for g in gg],color=['gray','tab:blue','tab:orange','tab:green'])
    axes[0].set(ylabel='Successful runs',ylim=(0,2.5),yticks=[0,1,2])
    for i,g in enumerate(gg):
        if g:axes[0].text(i,g['successes']+.05,f"{g['successes']}/{g['n']}",ha='center')
        times=[r['flight_time_s'] for r in rows if r['rate']==10 and (r['translation_scale'],r['rotation_scale'])==pairs[i] and r.get('physical_success')]
        axes[1].scatter([i]*len(times),times,color='tab:blue')
        margins=[r['min_clearance_m'] for r in rows if r['rate']==10 and (r['translation_scale'],r['rotation_scale'])==pairs[i] and 'min_clearance_m' in r]
        axes[2].scatter([i]*len(margins),margins,color='tab:orange')
    axes[1].set(ylabel='Successful finish time (s)')
    axes[2].set(ylabel='Min clearance BEFORE termination (m)');axes[2].axhline(0,color='red',ls='--',lw=1)
    for ax in axes:ax.set_xticks(range(4),labels,rotation=15);ax.grid(axis='y',alpha=.2)
    fig.suptitle('10x frequency: translation vs rotation\nSuccess also requires no contact AFTER termination')
    fig.savefig(root/'translation_vs_rotation.png',dpi=180);plt.close(fig)
    fig,axes=plt.subplots(2,3,figsize=(14,7),layout='constrained')
    for i,rate in enumerate([1,10]):
        for j,amp in enumerate([1,2,4]):
            ax=axes[i,j];match=[r for r in rows if r['rate']==rate and r['translation_scale']==amp and r['rotation_scale']==amp and r['phase_id']==0]
            if not match:continue
            r=match[0];data=np.genfromtxt(Path(r['run'])/'telemetry.csv',delimiter=',',names=True)
            speed=np.sqrt(data['vx']**2+data['vy']**2+data['vz']**2)
            ax.plot(data['t'],speed,lw=1,color='tab:blue')
            result=json.loads((Path(r['run'])/'result.json').read_text())
            for c in result['crossings']:ax.axvline(c['time'],color='gray',alpha=.3)
            ax.set(title=f"frequency {rate}x, amplitude {amp}x: {'PASS' if r.get('physical_success') else 'FAIL'}",xlabel='Race time (s)',ylabel='Actual speed (m/s)');ax.grid(alpha=.2)
    fig.suptitle('Fixed phase 0 — actual PX4 speeds, not planned speeds; verticals = crossings')
    fig.savefig(root/'speed_profiles.png',dpi=180);plt.close(fig)
    track,_=build_track();fig,axes=plt.subplots(2,4,figsize=(13,7),layout='constrained')
    colors={1:'gray',2:'tab:blue',4:'tab:orange'}
    for i,(ax,w) in enumerate(zip(axes.flat,track.windows)):
        if w.aperture.kind=='circle':
            theta=np.linspace(0,2*np.pi,150);vs=w.aperture.radius*np.c_[np.cos(theta),np.sin(theta)]
        else:vs=w.aperture.physical_vertices;vs=np.r_[vs,vs[:1]]
        ax.plot(vs[:,0],vs[:,1],color='black');ax.scatter(0,0,marker='+',color='gray')
        for r in rows:
            if r['rate']!=10 or r['translation_scale']!=r['rotation_scale']:continue
            result=json.loads((Path(r['run'])/'result.json').read_text())
            for c in result['crossings']:
                if c['gate']==i:ax.scatter(*c['q'],color=colors[r['translation_scale']],marker=['o','s'][r['phase_id']],s=35)
        ax.set(title=f'W{i+1}',xlabel='Local u (m)',ylabel='Local v (m)',aspect='equal');ax.grid(alpha=.2)
    ax=axes.flat[-1];ax.axis('off')
    for amp,col in colors.items():ax.scatter([],[],color=col,label=f'Both amplitudes {amp}x')
    ax.legend(loc='center');ax.text(.5,.1,'10x frequency; circle/square = phase 0/120 deg\nFailed-run crossings retained; no point = no crossing',transform=ax.transAxes,ha='center',fontsize=8)
    fig.savefig(root/'crossing_positions.png',dpi=180);plt.close(fig)
    text=['# 幅度实验结果','',f'已完成 {len(rows)}/16 个非基础设施故障条件；不足16时为阶段性报告。',
          '','固定算法与余量；每条件两个固定相位，不是总体成功率估计。','',
          '| 频率 | 平移幅度 | 摆角幅度 | 成功 | 实时成功 | 成功平均时间 s | 最大周期 ms | 最小采样净空 m |',
          '|---:|---:|---:|---:|---:|---:|---:|---:|']
    for g in grouped:
        finish='—' if g['mean_finish_s'] is None else f"{g['mean_finish_s']:.3f}"
        cycle='—' if g['max_cycle_ms'] is None else f"{g['max_cycle_ms']:.3f}"
        clearance='—' if g['min_clearance_m'] is None else f"{g['min_clearance_m']:.3f}"
        text.append(f"| {g['rate']} | {g['translation_scale']} | {g['rotation_scale']} | {g['successes']}/{g['n']} | {g['realtime_successes']}/{g['n']} | {finish} | {cycle} | {clearance} |")
    text+=['','表中最小净空包含失败飞行，但10 ms采样区间截至飞行循环终止；负数为外接球采样相交。',
           '零接触判定还覆盖终止后的清理阶段。仅摆角4倍、10倍频率相位120°在21.804 s中止，21.908 s碰撞W6；不能将表中的正净空解释为无碰撞。',
           '额外在接触时刻插值ULog真值：该次W6球净空为−0.198 m，详见[接触时刻审计](contact_audit.json)。',
           '成功须完成七门及返航、零接触、真值净空和六侧工作区通过；不是连续安全证书。','',
           '[幅度汇总](amplitude_summary.png) · [分离平移/旋转](translation_vs_rotation.png) · [速度](speed_profiles.png) · [穿越点](crossing_positions.png)','']
    for r in rows:
        if r.get('physical_success'):continue
        reasons=[]
        if r.get('failure'):reasons.append(r['failure'])
        if not r.get('room_passed',False):reasons.append('超出配置工作区；不等同于撞墙')
        if r.get('collision'):reasons.append('Gazebo门框接触')
        if r.get('min_clearance_m',0)<0:reasons.append('外接球采样净空为负')
        if r.get('analysis_error'):reasons.append(r['analysis_error'])
        text.append(f"- {Path(r['run']).name}：过门{r.get('gates','?')}/7；"+'；'.join(reasons))
    (root/'REPORT.md').write_text('\n'.join(text));print('\n'.join(text))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root');plot(p.parse_args().root)
