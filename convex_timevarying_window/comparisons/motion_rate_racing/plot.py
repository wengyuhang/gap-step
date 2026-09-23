"""Plots use actual telemetry and gate-local crossing points, not references."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track


def plot(root):
    root=Path(root); rows=json.loads((root/'summary.json').read_text())
    valid=[r for r in rows if not r.get('infrastructure_failure')]
    rates=sorted({r['rate'] for r in rows})
    colors=plt.cm.viridis(np.linspace(.1,.9,len(rates)))
    palette=dict(zip(rates,colors)); metrics=[]
    for r in valid:
        a=np.genfromtxt(Path(r['run'])/'telemetry.csv',delimiter=',',names=True)
        if a.size<2: continue
        s=np.sqrt(a['vx']**2+a['vy']**2+a['vz']**2)
        mean=float(np.trapz(s,a['t'])/(a['t'][-1]-a['t'][0]))
        metrics.append({**r,'mean_speed_mps':mean,'max_speed_mps':float(s.max())})
    (root/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
    fig,axes=plt.subplots(1,3,figsize=(13,3.8),layout='constrained')
    for rate in rates:
        rr=[r for r in valid if r['rate']==rate]
        n=len(rr); wins=sum(r.get('physical_success',False) for r in rr)
        realtime=sum(r.get('realtime_success',False) for r in rr)
        if n:
            axes[0].scatter(rate,wins/n*100,color=palette[rate],s=65)
            axes[0].scatter(rate,realtime/n*100,marker='x',color='black',s=55)
            axes[0].annotate(f'{wins}/{n}',(rate,wins/n*100),xytext=(0,-18),textcoords='offset points',ha='center')
        success=[r['flight_time_s'] for r in rr if r.get('physical_success')]
        for i,value in enumerate(success): axes[1].scatter(rate+(i-1)*.12,value,color=palette[rate],s=50)
        if not success: axes[1].annotate('no finish',(rate,0),xycoords=('data','axes fraction'),ha='center',va='bottom',rotation=90)
        for r in rr:
            if r.get('cycle_ms',{}).get('max') is not None: axes[2].scatter(rate,r['cycle_ms']['max'],color=palette[rate],s=45)
    axes[0].set(ylabel='Success (%)',ylim=(-5,110),title='Physical success; x = also meets 100 ms')
    axes[1].set(ylabel='Flight time (s)',title='Successful runs only (all phase conditions)')
    axes[2].set(ylabel='Max complete planning cycle (ms)',title='Includes unsuccessful planning cycles')
    axes[2].axhline(100,color='red',ls='--',lw=1)
    for ax in axes: ax.set_xlabel('Window motion frequency multiplier'); ax.set_xticks(rates); ax.grid(alpha=.2)
    fig.suptitle('Frozen online planner: same track, amplitude, vehicle and controller')
    fig.savefig(root/'frequency_summary.png',dpi=180); plt.close(fig)
    fig,axes=plt.subplots(len(rates),1,figsize=(12,2.7*len(rates)),sharex=True,layout='constrained',squeeze=False)
    for ax,rate in zip(axes[:,0],rates):
        rr=[r for r in valid if r['rate']==rate and r['phase_id']==0]
        if not rr: ax.set_title(f'{rate:g}x: not available'); continue
        r=rr[0]; run=Path(r['run']); a=np.genfromtxt(run/'telemetry.csv',delimiter=',',names=True)
        if a.size<2: continue
        s=np.sqrt(a['vx']**2+a['vy']**2+a['vz']**2)
        ax.plot(a['t'],s,color=palette[rate],lw=1.5)
        data=json.loads((run/'result.json').read_text())
        for cross in data['crossings']:
            ax.axvline(cross['time'],color='gray',alpha=.4,lw=.8)
            ax.text(cross['time'],12.7,f"W{cross['gate']+1}",fontsize=8,ha='center')
        ax.set(ylabel='Speed (m/s)',ylim=(0,max(14,float(s.max())+1)),title=f"{rate:g}x, phase 0: {'success' if r.get('physical_success') else 'FAILED'}")
        ax.grid(alpha=.2)
    axes[-1,0].set_xlabel('Race time (s), excluding takeoff preparation')
    fig.suptitle('Actual PX4 estimated speed; vertical lines = measured gate crossings')
    fig.savefig(root/'speed_profiles.png',dpi=180); plt.close(fig)
    track,_=build_track(); fig,axes=plt.subplots(2,4,figsize=(13,7),layout='constrained')
    for i,(ax,w) in enumerate(zip(axes.flat,track.windows)):
        if w.aperture.kind=='circle':
            theta=np.linspace(0,2*np.pi,200); vertices=w.aperture.radius*np.c_[np.cos(theta),np.sin(theta)]
        else:
            vertices=w.aperture.physical_vertices; vertices=np.r_[vertices,vertices[:1]]
        ax.plot(vertices[:,0],vertices[:,1],color='black',lw=1.7,label='Physical opening')
        ax.scatter(0,0,marker='+',color='gray',s=60)
        for r in valid:
            data=json.loads((Path(r['run'])/'result.json').read_text())
            for cross in data['crossings']:
                if cross['gate']==i:
                    ax.scatter(*cross['q'],color=palette[r['rate']],marker=['o','s','^'][r['phase_id']%3],s=42,alpha=.8)
        ax.set(title=f'W{i+1}',xlabel='Local u (m)',ylabel='Local v (m)',aspect='equal'); ax.grid(alpha=.2)
    axes.flat[-1].axis('off')
    for rate in rates: axes.flat[-1].scatter([],[],color=palette[rate],label=f'{rate:g}x frequency')
    axes.flat[-1].legend(loc='center')
    axes.flat[-1].text(.5,.12,'o / square / triangle: phases 0 / 120 / 240 deg\nDots: vehicle centre at crossing\nFailed-run crossings are retained',transform=axes.flat[-1].transAxes,ha='center',fontsize=9)
    fig.suptitle('Crossings in each moving gate\'s own frame (not fixed world coordinates)')
    fig.savefig(root/'crossing_positions.png',dpi=180); plt.close(fig)
    from matplotlib.collections import LineCollection
    fig,axes=plt.subplots(1,2,figsize=(12,5),layout='constrained')
    from convex_timevarying_window.receding_gate_racing.scenario import vary_track
    for ax,rate in zip(axes,[1,3]):
        rr=[r for r in valid if r['rate']==rate and r['phase_id']==0]
        if not rr:continue
        r=rr[0];run=Path(r['run']);data=json.loads((run/'result.json').read_text())
        a=np.genfromtxt(run/'telemetry.csv',delimiter=',',names=True)
        speed=np.sqrt(a['vx']**2+a['vy']**2+a['vz']**2)
        xy=np.c_[a['x'],a['y']];segments=np.stack([xy[:-1],xy[1:]],axis=1)
        lc=LineCollection(segments,cmap='turbo',norm=plt.Normalize(0,12),linewidths=2)
        lc.set_array(speed[:-1]);ax.add_collection(lc)
        track,_=build_track();vary_track(track,motion_rate=rate)
        for cross in data['crossings']:
            w=track.windows[cross['gate']];c,R,*_=w.state_at(cross['time'])
            q=np.array(cross['q']);world=c+R[:,:2]@q
            ax.scatter(*world[:2],marker='o',s=35,facecolor='white',edgecolor='black',zorder=4)
            ax.annotate(f"W{cross['gate']+1}",world[:2],xytext=(5,5),textcoords='offset points',fontsize=9)
        ax.scatter(*track.start[:2],marker='*',color='black',s=85)
        ax.set(xlim=(-29,30),ylim=(-31,24),aspect='equal',xlabel='World x (m)',ylabel='World y (m)',title=f'{rate}x motion, phase 0')
        ax.grid(alpha=.2)
    fig.colorbar(lc,ax=axes,label='Actual speed (m/s)',shrink=.85)
    fig.suptitle('Actual flight path coloured by speed; circles = gate crossings')
    fig.savefig(root/'trajectory_speed.png',dpi=180);plt.close(fig)
    print(root)


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('root'); plot(p.parse_args().root)
