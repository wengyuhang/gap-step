"""Actual Gazebo crossing, gate-local plane slices of the enclosing sphere."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon
from convex_timevarying_window.comparisons.single_gate_translation.audit import audit
from convex_timevarying_window.comparisons.single_gate_translation.scenario import Case,build_case


def main():
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);args=p.parse_args()
    path=args.directory;raw=json.loads((path/'result.json').read_text())
    metrics=audit(path)
    (path/'crossing_section_metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
    trace=np.load(path/'mechanism_trace.npz');ts=trace['times'];local=trace['local_positions']
    cross=metrics['crossings'][0]['time_s'];radius=raw['config']['radius']
    track,_=build_case(Case(**raw['case']));w=track.windows[0];R=w.state_at(0.)[1]
    fig,axes=plt.subplots(1,3,figsize=(12,4.8),layout='constrained')
    samples=[]
    for ax,dt in zip(axes,(-.02,0.,.02)):
        t=cross+dt;q=np.array([np.interp(t,ts,local[:,k]) for k in range(3)])
        section=np.sqrt(max(0.,radius**2-q[2]**2))
        ax.add_patch(Polygon(w.aperture.physical_vertices,closed=True,fill=False,edgecolor='#333333',lw=7))
        ax.add_patch(Circle(q[:2],radius,fill=False,edgecolor='#e4572e',ls='--',alpha=.5,
                            label='Enclosing-sphere projection'))
        if abs(q[2])<=radius:
            ax.add_patch(Circle(q[:2],section,color='#e4572e',alpha=.35,label='Sphere / gate-plane slice'))
        ax.plot(*q[:2],'o',color='#b52f17',label='Actual drone center projection')
        ax.plot(0,0,'+',color='black',ms=10,label='Gate center')
        near=np.abs(ts-cross)<.035
        ax.plot(local[near,0],local[near,1],color='#e4572e',lw=1)
        h=1e-5;v=((w.state_at(t+h)[0]-w.state_at(t-h)[0])/(2*h))@R
        if np.linalg.norm(v[:2])>1e-6:
            direction=v[:2]/np.linalg.norm(v[:2])
            ax.arrow(-.9,-.85,*(direction*.45),width=.015,color='#235789',length_includes_head=True)
        ax.text(-1.55,-1.49,f'Gate in-plane velocity: ({v[0]:.1f}, {v[1]:.1f}) m/s\n'
                f'Center normal offset: {q[2]:+.3f} m',fontsize=8)
        ax.set(xlim=(-1.7,1.7),ylim=(-1.65,1.45),aspect='equal',xlabel='Gate-local u (m)',ylabel='Gate-local v (m)',
               title=f't = {t:.6f} s ({dt*1000:+.0f} ms)')
        ax.grid(alpha=.2)
        samples.append(dict(time_s=t,center_gate_local_m=q.tolist(),sphere_section_radius_m=section,
                            gate_velocity_local_mps=v.tolist()))
    axes[0].legend(loc='upper left',fontsize=6.5)
    fig.suptitle('4 Hz / 4 m: actual Gazebo flight, not nominal trajectory\n'
                 f'Forward plane crossing at {cross:.6f} s; physical frame contacts: 0\n'
                 'Orange disk is an enclosing-sphere slice, NOT the actual x500 mesh silhouette',fontsize=11)
    fig.savefig(path/'crossing_section.png',dpi=190)
    (path/'crossing_section_samples.json').write_text(json.dumps(samples,indent=2)+'\n')
    print(json.dumps(dict(crossing=metrics['crossings'],entry=metrics['entry_time_s'],exit=metrics['exit_time_s'],
                         passed=metrics['passed'],radius=radius,image=str(path/'crossing_section.png'))))


if __name__=='__main__':main()
