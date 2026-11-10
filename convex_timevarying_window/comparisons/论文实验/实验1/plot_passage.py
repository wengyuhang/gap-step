"""Measured high-dynamic passage: exact gate queries, planned/actual point and speeds."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Circle
import numpy as np
from convex_timevarying_window.comparisons.single_gate_swing.scenario import Case,build_case

HERE=Path(__file__).resolve().parent

def main():
    out=HERE/'results/translation_v1_20261105';rows=json.loads((out/'summary.json').read_text());fig,axes=plt.subplots(3,2,figsize=(10,10));records=[]
    for j,method in enumerate(('ours','center')):
        row=next(r for r in rows if r['amplitude_m']==4 and r['frequency_hz']==4 and r['variant']==method);path=Path(row['directory']);raw=json.loads((path/'result.json').read_text());trace=np.load(path/'mechanism_trace.npz')
        w=build_case(Case(4.,4.,0.))[0].windows[0];R=w.state_at(0)[1];t=trace['times'];p=trace['positions'];c=np.array([w.state_at(float(v))[0] for v in t]);q=(p-c)@R
        at=row['crossings'][0]['time_s'];actual=np.array([np.interp(at,t,p[:,k]) for k in range(3)]);actualq=R.T@(actual-w.state_at(at)[0]);bodyv=np.array([np.interp(at,t,trace['velocity'][:,k]) for k in range(3)]);gatev=w.motion.translation(at)[1]
        solve=next(s for s in raw['solves'] if s['accepted']);planned_t=solve['made_at']+solve['durations'][0];planned=np.asarray(solve['coefficients'])[0].sum(axis=0);plannedq=R.T@(planned-w.state_at(planned_t)[0])
        mask=abs(t-at)<.16;ax=axes[0,j];ax.add_patch(Rectangle((-1.45,-1.15),2.9,2.3,fill=False,color='black',linewidth=2));ax.plot(q[mask,0],q[mask,1],'.-',color='#167d9a');ax.scatter(*plannedq[:2],marker='+',s=100,color='#329543',label='Planned point');ax.scatter(*actualq[:2],marker='x',s=70,color='#b82f2f',label='Measured point')
        ax.add_patch(Circle(actualq[:2],raw['config']['radius'],fill=False,color='grey',label='Sphere proxy at plane'))
        for velocity,color in ((gatev,'#8c50a3'),(bodyv-gatev,'#d67125')):
            v=(velocity@R)[:2];length=np.linalg.norm(v)
            if length>1e-6:ax.arrow(*actualq[:2],*(.32*v/length),head_width=.06,length_includes_head=True,color=color)
        ax.set(xlim=(-1.8,1.8),ylim=(-1.5,1.5),xlabel='Gate-local axis 1 (m)',ylabel='Gate-local axis 2 (m)',title='Free point' if method=='ours' else 'Fixed centre');ax.set_aspect('equal')
        if j==0:ax.legend(loc='upper left',fontsize=8)
        dt=t-at;gate_velocity=trace['gate_velocity'];relative=trace['velocity']-gate_velocity
        for values,label,color in ((trace['velocity'],'Drone','#167d9a'),(gate_velocity,'Gate','#8c50a3'),(relative,'Relative','#d67125')):axes[1,j].plot(dt[mask],np.linalg.norm(values,axis=1)[mask],label=label,color=color)
        axes[1,j].set(ylabel='Speed (m/s)',xlabel='Time from centre crossing (s)')
        if j==0:axes[1,j].legend(fontsize=8)
        axes[2,j].plot(dt[mask],q[mask,2],color='#167d9a');h=raw['config']['radius']+raw['config']['frame_thickness'];axes[2,j].axhspan(-h,h,color='grey',alpha=.15);axes[2,j].set(ylabel='Signed normal distance (m)',xlabel='Time from centre crossing (s)')
        for ax in axes[1:,j]:
            ax.axvspan(row['entry_time_s']-at,row['exit_time_s']-at,color='#167d9a',alpha=.12,label='Body passage');ax.axvline(0,color='black',linestyle='--',linewidth=.8);ax.grid(alpha=.2)
        records.append(dict(method=method,planned_crossing_s=planned_t,actual_crossing_s=at,planned_gate_point_m=plannedq[:2].tolist(),measured_gate_point_m=actualq[:2].tolist(),gate_velocity_world_mps=gatev.tolist(),body_velocity_world_mps=bodyv.tolist(),relative_velocity_world_mps=(bodyv-gatev).tolist(),body_passage_s=row['body_passage_duration_s'],point_definition='linearly interpolated measured body; exact known gate query at each event time'))
    fig.suptitle('4 m / 4 Hz: full passage, point choice and relative motion\nTop arrows: gate (purple), relative drone motion (orange); directions only',fontsize=12);fig.tight_layout(rect=(0,0,1,.94));fig.savefig(out/'crossing_motion_and_speed.png',dpi=190);plt.close(fig);(out/'passage_mechanism.json').write_text(json.dumps(records,indent=2)+'\n')

if __name__=='__main__':main()
