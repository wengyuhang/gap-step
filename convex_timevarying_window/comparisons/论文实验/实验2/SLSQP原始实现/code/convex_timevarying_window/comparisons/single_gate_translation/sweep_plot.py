"""Scientific matrix and crossing-event figures from frozen paired flights."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import numpy as np
from .scenario import Case,build_case

FREQUENCIES=(1.,1.5,2.,2.5,3.,3.5,4.)
AMPLITUDES=(2.,2.5,3.,3.5,4.)
CASES=((2.5,3.5),(3.,4.),(4.,2.))
COLORS={'center':'#235789','ours':'#e4572e'}


def load(root):
    rows=json.loads((root/'summary.json').read_text())
    return {(r['trial']['frequency'],r['trial']['amplitude'],r['trial']['method']):r for r in rows}


def matrix(root,rows):
    fig,axes=plt.subplots(1,3,figsize=(14,4.4),layout='constrained')
    for method,ax in zip(('ours','center'),axes[:2]):
        data=np.full((len(AMPLITUDES),len(FREQUENCIES)),np.nan)
        for i,a in enumerate(AMPLITUDES):
            for j,f in enumerate(FREQUENCIES):
                r=rows.get((f,a,method))
                if r is None:continue
                data[i,j]=0 if r['collision_detected'] else 1 if r['passed'] else 2
        ax.imshow(np.ma.masked_invalid(data),vmin=0,vmax=2,
                  cmap=ListedColormap(['#d73027','#1a9850','#fdae61']),aspect='auto')
        ax.set_title('Free crossing point' if method=='ours' else 'Gate center')
        for i,a in enumerate(AMPLITUDES):
            for j,f in enumerate(FREQUENCIES):
                r=rows.get((f,a,method))
                if r is None:continue
                symbol='X' if r['collision_detected'] else f"{r['flight_time_s']:.1f}" if r['passed'] else '?'
                ax.text(j,i,symbol,ha='center',va='center',color='white',fontsize=9,weight='bold')
    advantage=np.full((len(AMPLITUDES),len(FREQUENCIES)),np.nan)
    for i,a in enumerate(AMPLITUDES):
        for j,f in enumerate(FREQUENCIES):
            pair=[rows.get((f,a,m)) for m in ('center','ours')]
            if all(r is not None and r['passed'] for r in pair):
                advantage[i,j]=pair[0]['flight_time_s']-pair[1]['flight_time_s']
    ax=axes[2]
    cmap=plt.get_cmap('RdBu').copy();cmap.set_bad('white')
    im=ax.imshow(np.ma.masked_invalid(advantage),vmin=-5,vmax=5,cmap=cmap,aspect='auto')
    ax.set_title('Center time − free time (s)')
    for i in range(len(AMPLITUDES)):
        for j in range(len(FREQUENCIES)):
            if np.isfinite(advantage[i,j]):
                ax.text(j,i,f'{advantage[i,j]:+.2f}',ha='center',va='center',fontsize=8)
    fig.colorbar(im,ax=ax,label='Positive: free point is faster',shrink=.83)
    for ax in axes:
        ax.set_xticks(range(len(FREQUENCIES)),FREQUENCIES)
        ax.set_yticks(range(len(AMPLITUDES)),AMPLITUDES)
        ax.set_xlabel('Frequency (Hz)');ax.set_ylabel('Amplitude (m)')
        ax.grid(which='minor',color='#dddddd',lw=.7)
        ax.set_xticks(np.arange(-.5,len(FREQUENCIES),1),minor=True)
        ax.set_yticks(np.arange(-.5,len(AMPLITUDES),1),minor=True)
    fig.savefig(root/'sweep_matrix.png',dpi=180)


def contacts(root,rows):
    fig,axes=plt.subplots(3,1,figsize=(9,8),layout='constrained')
    for ax,(f,a) in zip(axes,CASES):
        track,_=build_case(Case(f,a,0.));w=track.windows[0]
        lateral=w.state_at(0.)[1][:,1]
        pair=[rows[(f,a,m)] for m in ('center','ours')]
        events=([e['time_s'] for r in pair for e in r['crossings']]+
                [c['first_contact_sim_time_s']-30 for r in pair for c in r['contacts']
                 if c['first_contact_sim_time_s'] is not None])
        end=min(max(events)+.5,5.)
        for method in ('center','ours'):
            r=rows[(f,a,method)]
            trace=np.load(Path(r['directory'])/'mechanism_trace.npz')
            ts=trace['times'];pos=trace['positions']
            y=(pos-w.center0)@lateral
            selection=(ts>=1.8)&(ts<=end)
            ax.plot(ts[selection],y[selection],c=COLORS[method],label=method,lw=1.7)
            for event in r['crossings']:
                ax.plot(event['time_s'],np.interp(event['time_s'],ts,y),'o',c=COLORS[method],ms=5)
            for c in r['contacts']:
                if c['first_contact_sim_time_s'] is None:continue
                time=c['first_contact_sim_time_s']-30
                ax.plot(time,np.interp(time,ts,y),'X',c=COLORS[method],ms=9)
        tt=np.linspace(0,end,500)
        gate=np.array([w.state_at(float(t))[0] for t in tt]);center=(gate-w.center0)@lateral
        ax.fill_between(tt,center-1.15,center+1.15,color='gray',alpha=.17,label='aperture')
        ax.plot(tt,center,'--',c='black',lw=.9,label='gate center')
        ax.set_xlim(1.8,end)
        ax.set_title(f'{f:g} Hz / {a:g} m — X: actual frame contact, dot: plane crossing')
        ax.set_ylabel('Lateral world position (m)')
        ax.grid(alpha=.2)
    axes[0].legend(loc='upper right',fontsize=8)
    axes[-1].set_xlabel('Time after takeoff (s)')
    fig.savefig(root/'collision_mechanisms.png',dpi=180)


def main():
    p=argparse.ArgumentParser();p.add_argument('result_dir',type=Path);a=p.parse_args()
    root=a.result_dir;rows=load(root)
    matrix(root,rows);contacts(root,rows)
    print(root/'sweep_matrix.png');print(root/'collision_mechanisms.png')


if __name__=='__main__':main()
