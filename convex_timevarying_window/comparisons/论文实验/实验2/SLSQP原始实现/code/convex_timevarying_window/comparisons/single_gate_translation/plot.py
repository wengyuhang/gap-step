"""Publication-style diagnostic for true single-gate x500 crossing traces."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from .scenario import Case,build_case

CASES=((1.,0.),(1.,2.),(2.,2.))
COLORS={'center':'#235789','ours':'#e4572e'}
LABELS={'center':'Center','ours':'Free crossing point'}


def main():
    p=argparse.ArgumentParser();p.add_argument('result_dir',type=Path);a=p.parse_args()
    root=a.result_dir
    rows=json.loads((root/'summary.json').read_text())
    lookup={(x['trial']['frequency'],x['trial']['amplitude'],x['trial']['method']):x for x in rows}
    fig,axes=plt.subplots(3,2,figsize=(11,8.5),layout='constrained')
    for i,(f,amplitude) in enumerate(CASES):
        track,_=build_case(Case(f,amplitude,0.));w=track.windows[0]
        lateral=w.state_at(0.)[1][:,1]
        left,right=axes[i]
        for method in ('center','ours'):
            row=lookup[(f,amplitude,method)]
            trace=np.load(Path(row['directory'])/'mechanism_trace.npz')
            t=trace['times'];pos=trace['positions'];center=trace['gate_centers']
            gate=(center-w.center0)@lateral
            drone=(pos-w.center0)@lateral
            cross=row['crossings'][0]['time_s']
            color=COLORS[method]
            left.plot(t,drone,c=color,lw=1.6,label=LABELS[method])
            left.plot(cross,np.interp(cross,t,drone),'o',c=color,ms=5)
            selection=np.abs(t-cross)<=.6
            right.plot(t[selection]-cross,trace['proxy_clearance'][selection],
                       c=color,lw=1.6,label=LABELS[method])
            if row['entry_time_s'] is not None and row['exit_time_s'] is not None:
                right.axvspan(row['entry_time_s']-cross,row['exit_time_s']-cross,
                              color=color,alpha=.06)
        t=np.linspace(0,max(lookup[(f,amplitude,m)]['flight_time_s'] for m in COLORS),800)
        moving=np.array([w.state_at(float(x))[0] for x in t])
        gate=(moving-w.center0)@lateral
        left.fill_between(t,gate-1.15,gate+1.15,color='gray',alpha=.14,label='Gate aperture span')
        left.plot(t,gate,color='black',lw=1.,ls='--',label='Gate center')
        left.set_ylabel('Lateral world position (m)')
        left.set_title(f'A={amplitude:g} m, f={f:g} Hz')
        left.grid(alpha=.2)
        right.axhline(0,color='black',lw=.8,ls='--')
        right.set_ylabel('Sampled sphere proxy clearance (m)')
        right.set_title('Around plane crossing; shaded: body transit')
        right.grid(alpha=.2)
    axes[-1,0].set_xlabel('Time after takeoff (s)')
    axes[-1,1].set_xlabel('Time relative to plane crossing (s)')
    axes[0,0].legend(fontsize=8,loc='upper left')
    axes[0,1].legend(fontsize=8,loc='upper right')
    target=root/'mechanism_comparison.png';fig.savefig(target,dpi=180)
    print(target)


if __name__=='__main__':main()
