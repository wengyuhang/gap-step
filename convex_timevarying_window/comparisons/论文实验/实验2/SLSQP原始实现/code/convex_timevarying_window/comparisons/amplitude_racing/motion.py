"""Describe actual input motion in SI units; numerical extrema, not bounds."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.receding_gate_racing.scenario import vary_track


def main(root):
    root=Path(root);protocol=json.loads((root/'protocol.json').read_text());records=[]
    # Same 60 s race interval, 10 ms grid; these are sampled maxima.
    times=np.arange(0,60.0001,.01)
    for rate,translation,rotation,group in protocol['conditions']:
        track,_=build_track();vary_track(track,motion_rate=rate,translation_scale=translation,rotation_scale=rotation)
        centre=angular=edge=0.
        for w in track.windows:
            poses=[w.state_at(t) for t in times]
            R=np.array([p[1] for p in poses]);dc=np.array([p[2] for p in poses]);dR=np.array([p[3] for p in poses])
            centre=max(centre,float(np.linalg.norm(dc,axis=1).max()))
            skew=np.einsum('tik,tjk->tij',dR,R)
            omega=np.c_[skew[:,2,1],skew[:,0,2],skew[:,1,0]]
            angular=max(angular,float(np.rad2deg(np.linalg.norm(omega,axis=1).max())))
            q=w.aperture.boundary_points(circle_samples=64,safe=False)
            speed=np.linalg.norm(dc[:,None,:]+np.einsum('tij,pj->tpi',dR[:,:,:2],q),axis=2)
            edge=max(edge,float(speed.max()))
        records.append(dict(rate=rate,translation_scale=translation,rotation_scale=rotation,
                            max_sampled_centre_speed_mps=centre,max_sampled_angular_speed_deg_s=angular,
                            max_sampled_aperture_edge_speed_mps=edge))
    (root/'motion_metrics.json').write_text(json.dumps(records,indent=2)+'\n')
    fig,axes=plt.subplots(1,2,figsize=(11,3.8),layout='constrained')
    times=np.linspace(0,3,1000)
    for amp,color in [(1,'gray'),(2,'tab:blue'),(4,'tab:orange')]:
        track,_=build_track();vary_track(track,motion_rate=10,translation_scale=amp,rotation_scale=amp)
        w=track.windows[0]
        axes[0].plot(times,[w.state_at(t)[0][0]-w.center0[0] for t in times],label=f'{amp}x amplitude',color=color)
        axes[1].plot(times,[np.rad2deg(w.motion.rotation(t)[0][2]) for t in times],color=color)
    axes[0].set(ylabel='W1 centre x displacement (m)');axes[0].legend()
    axes[1].set(ylabel='W1 yaw offset (deg)')
    for ax in axes:ax.set_xlabel('Race time (s)');ax.grid(alpha=.2)
    fig.suptitle('10x frequency: amplitude changes distance/angle, NOT the period')
    fig.savefig(root/'window_motion.png',dpi=180);plt.close(fig)
    print(json.dumps(records,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root');main(p.parse_args().root)
