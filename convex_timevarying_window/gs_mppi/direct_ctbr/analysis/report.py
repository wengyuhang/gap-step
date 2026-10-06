"""Analyze direct-CTBR frozen nominal data without modifying its inputs."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import MASS,KF

def audit(rows):
    x=rows[:,1:18];w,qx,qy,qz=x[:,6:10].T
    body_z=np.column_stack((2*(qx*qz+w*qy),2*(qy*qz-w*qx),1-2*(qx*qx+qy*qy)))
    force=KF*np.sum(x[:,13:17]**2,axis=1);drag=8.06428e-5*np.sum(x[:,13:17],axis=1)/MASS
    axial=np.sum(x[:,3:6]*body_z,axis=1)
    acceleration=body_z*force[:,None]/MASS-[0,0,9.8066]-drag[:,None]*(x[:,3:6]-axial[:,None]*body_z)
    return dict(max_speed=float(np.linalg.norm(x[:,3:6],axis=1).max()),max_acceleration=float(np.linalg.norm(acceleration,axis=1).max()),
                max_tilt=float(np.rad2deg(np.arccos(np.clip(body_z[:,2],-1,1))).max()),
                max_xy_bodyrate=float(np.linalg.norm(x[:,10:12],axis=1).max()),max_yaw_rate=float(abs(x[:,12]).max()))

def main():
    p=argparse.ArgumentParser();p.add_argument('--matrix',type=Path,required=True);p.add_argument('--outdir',type=Path,required=True);a=p.parse_args();a.outdir.mkdir(parents=True,exist_ok=False)
    cases=[];allms=[];planning=[];source=None;checks=[]
    for f in sorted(a.matrix.glob('*/summary.json')):
        d=json.loads(f.read_text());source=d['source_sha256'] if source is None else source
        if source!=d['source_sha256']:raise ValueError('Cannot pool different snapshots')
        z=np.load(f.parent/'trace.npz')['rows'];physical=audit(z) if len(z) else {}
        logs=[json.loads(l) for l in (f.parent/'cycles.jsonl').read_text().splitlines()];ms=[r['cycle_ms'] for r in logs];allms+=ms
        planning += [r['wall_ms'] for r in d['updates']];checks +=[len(r['cpu_checks']) for r in logs]
        cases.append(dict(name=f.parent.name,single=d['condition']['single'],phase_degrees=float(np.rad2deg(d['condition']['phase'])),
            rate=d['condition']['motion_rate'],completed=d['completed'],time=d['time'],gates=d['gates'],failure=d['failure'],
            clearance=d['minimum_clearance'],cycles=d['cycles'],planner=d['planner'],prediction_error=d['prediction_execution_error_max'],physical=physical))
    seven=[r for r in cases if not r['single']];single=[r for r in cases if r['single']]
    stats=dict(cases=cases,source_identical=True,source_count=len(source),single_passed=sum(r['completed'] for r in single),single_total=len(single),
        seven_passed=sum(r['completed'] for r in seven),seven_total=len(seven),mppi_count=len(allms),mppi_max=max(allms),
        mppi_median=float(np.median(allms)),mppi_p99=float(np.quantile(allms,.99)),mppi_over100=sum(v>=100 for v in allms),
        mppi_over20=sum(v>20 for v in allms),togt_count=len(planning),togt_max=max(planning,default=0.),togt_over100=sum(v>=100 for v in planning),cpu_checks_max=max(checks))
    (a.outdir/'statistics.json').write_text(json.dumps(stats,indent=2)+'\n')
    fig,ax=plt.subplots(figsize=(10,4),layout='constrained')
    for d in seven:
        z=np.load(a.matrix/d['name']/'trace.npz')['rows'];ax.plot(z[:,0],np.linalg.norm(z[:,4:7],axis=1),lw=1,label=f"rate {d['rate']:g}, phase {d['phase_degrees']:g} deg")
    ax.axhline(12,color='black',ls='--',lw=.8);ax.set(xlabel='Nominal time [s]',ylabel='Speed [m/s]',title='Direct CTBR / 17-state model — solver clock paused, not PX4');ax.legend(ncol=2,fontsize=8)
    fig.savefig(a.outdir/'speed.png',dpi=150);plt.close(fig)
    if seven:
        z=np.load(a.matrix/seven[0]['name']/'trace.npz')['rows'];fig,axes=plt.subplots(4,1,figsize=(10,7),sharex=True,layout='constrained')
        actual=np.column_stack((KF*np.sum(z[:,14:18]**2,axis=1),z[:,11:14]))
        for j,ax in enumerate(axes):
            ax.plot(z[:,0],z[:,18+j],lw=.9,label='MPPI CTBR command');ax.plot(z[:,0],actual[:,j],lw=.8,ls='--',label='Plant response')
            ax.set_ylabel(('T [N]','roll [rad/s]','pitch [rad/s]','yaw [rad/s]')[j])
        axes[0].legend(ncol=2,fontsize=8);axes[-1].set_xlabel('Nominal time [s]');fig.suptitle('Commands and actuator response included in prediction — '+seven[0]['name'])
        fig.savefig(a.outdir/'direct_controls.png',dpi=150);plt.close(fig)
    lines=['# 直接CTBR冻结名义矩阵','',
           '预测与执行均为17状态经验CTBR模型；实际输出T、wx、wy、wz，不经过加速度跟踪器。',
           '求解时钟暂停；没有把同模型一致性当作PX4模型准确性或鲁棒安全证明。','',
           '| 条件 | 完成 | 门数 | 时间(s) | 区间余量(m) | MPPI最大(ms) | TOGT最大(ms) |',
           '|---|---:|---:|---:|---:|---:|---:|']
    for r in cases:lines.append(f"| {r['name']} | {r['completed']} | {r['gates']} | {r['time']:.2f} | {r['clearance']:.5f} | {r['cycles']['max']:.2f} | {r['planner']['max']:.2f} |")
    lines+=['',f"{len(cases)}个条件、{len(source)}个记录源码哈希一致；相位输入明确为弧度，表中的p120对应2π/3。",
            f"MPPI {len(allms)}拍，最大{max(allms):.2f} ms，>=100 ms {stats['mppi_over100']}拍，>20 ms {stats['mppi_over20']}拍。",
            f"TOGT全部{len(planning)}次调用计入，包括没有返回参考者；最大{stats['togt_max']:.2f} ms，>=100 ms {stats['togt_over100']}次。",
            '原机体/0.35 m误差假设/框厚/工作区及物理上限均未降低；名义返航验收仍为位置0.55 m、速度1 m/s，不是精确PVAJ相等。',
            '当前有限样本通过不表示任意同类型赛道可保证成功，投影也没有全局递归可行性证明。','',
            '![速度](speed.png)','![直接控制及响应](direct_controls.png)']
    (a.outdir/'REPORT.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps({k:v for k,v in stats.items() if k!='cases'},indent=2))

if __name__=='__main__':main()
