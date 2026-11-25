"""Evidence-separated JSON report and static scientific plots."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.receding_gate_racing.scenario import vary_track

def continuous_quaternions(q):
    q=np.asarray(q,dtype=float).copy()
    flips=np.where(np.sum(q[1:]*q[:-1],axis=1)<0,-1.,1.)
    q*=np.r_[1.,np.cumprod(flips)][:,None]
    return q

def audit_px4(path,r):
    from pyulog import ULog
    from scipy.spatial.transform import Rotation
    from convex_timevarying_window.dynamic_gate_mppi_v3.geometry import Geometry
    from .controller import config
    u=ULog(str(path/'flight.ulg'));d=u.get_dataset('vehicle_local_position_groundtruth').data
    a=u.get_dataset('vehicle_attitude_groundtruth').data;e=u.get_dataset('vehicle_local_position').data
    q=continuous_quaternions(np.column_stack([a[k] for k in ('q[1]','q[2]','q[3]','q[0]')]))
    t=np.arange(30.,30.+r['last_time_s'],.01)
    def positions(times):
        quat=np.column_stack([np.interp(times,a['timestamp']*1e-6,q[:,j]) for j in range(4)])
        rotation=Rotation.from_quat(quat)
        p=np.column_stack([np.interp(times,d['timestamp']*1e-6,d[k]) for k in ('x','y','z')])
        p+=rotation.apply(np.tile([0,0,-.24],(len(times),1)))
        return p[:,[1,0,2]]*[1,1,-1],rotation
    p,rotation=positions(t)
    raw=d['timestamp']*1e-6;raw=raw[(raw>=30)&(raw<=30+r['last_time_s'])]
    pr,_=positions(raw);v=np.gradient(pr,raw,axis=0);acc=np.gradient(v,raw,axis=0)
    ep=np.column_stack([np.interp(t,e['timestamp']*1e-6,e[k]) for k in ('y','x','z')])*[1,1,-1]+np.array(r['origin_enu_m'])
    track,_=build_track();vary_track(track,r['phase'],r['layout'],r['motion_rate'],r['translation_scale'],r['rotation_scale'])
    geometry=Geometry(track,config());centers,rots=geometry.poses(t-30)
    clear=[]
    for i,f in enumerate(geometry.frames):
        local=np.einsum('tji,tj->ti',rots[:,i],p-centers[:,i])
        clear.append(float((np.hypot(f.boundary_distance(local[:,:2]),local[:,2])-r['config']['radius']-.085).min()))
    ms=np.array([x['cycle_ms'] for x in r['solves']]);updates=r['reference_updates']
    audit=dict(completed=r['completed'],failure=r['failure'],gates=len(r['crossings']),contacts=r['collision_detected'],last_time_s=r['last_time_s'],
        min_sampled_body_clearance_m=min(clear),per_gate_clearance_m=clear,
        max_truth_tilt_deg=float(np.degrees(np.arccos(np.clip(rotation.as_matrix()[:,2,2],-1,1))).max()),
        max_sampled_body_speed_mps=float(np.linalg.norm(v[2:-2],axis=1).max()),max_sampled_body_acceleration_mps2=float(np.linalg.norm(acc[2:-2],axis=1).max()),
        max_absolute_estimator_position_error_m=float(np.linalg.norm(ep-p,axis=1).max()),
        body_derivative_sampling_s=float(np.median(np.diff(raw))),
        all_six_workspace_faces_passed=bool(np.all(p>np.array(r['config']['lower'])) and np.all(p<np.array(r['config']['upper']))),
        mppi=dict(count=len(ms),median_ms=float(np.median(ms)),p99_ms=float(np.quantile(ms,.99)),max_ms=float(ms.max()),over20=int(sum(ms>20)),over100=int(sum(ms>=100))),
        togt=dict(returned=len(updates),accepted=sum(x['accepted'] for x in updates),deadline_rejections=sum(isinstance(x['error'],dict) and x['error'].get('error')=='worker_deadline_exceeded' for x in updates)),
        quaternion_signs_unwrapped=True,audit_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        evidence='10 ms body-centre geometry; raw-grid finite-difference dynamics; not continuous certification')
    audit['model_error_assumption_met']=audit['max_absolute_estimator_position_error_m']<=r['config']['error_margin']
    audit['strict_sampled_success']=bool(r['completed'] and not r['collision_detected'] and audit['min_sampled_body_clearance_m']>0 and audit['all_six_workspace_faces_passed'] and audit['max_truth_tilt_deg']<=42 and audit['max_sampled_body_speed_mps']<=12 and audit['max_sampled_body_acceleration_mps2']<=8 and audit['mppi']['over100']==0)
    audit['strict_sampled_success'] &= audit['togt']['deadline_rejections']==0 and all(x.get('solve_ms') is None or x['solve_ms']<100 for x in updates)
    (path/'audit.json').write_text(json.dumps(audit,indent=2)+'\n');print(json.dumps(audit,indent=2))
    return raw-30,pr,v

def report(path):
    px4=(path/'result.json').exists()
    r=json.loads((path/('result.json' if px4 else 'summary.json')).read_text())
    if px4:
        t,p,v=audit_px4(path,r)
        condition=r
    else:
        x=np.load(path/'trace.npz')['rows'];t=x[:,0];p=x[:,1:4];v=x[:,4:7];condition=r['condition']
    track,_=build_track();vary_track(track,condition['phase'],condition.get('layout',0),condition['motion_rate'],condition['translation_scale'],condition['rotation_scale'])
    fig,ax=plt.subplots(1,2,figsize=(11,4.2),constrained_layout=True)
    ax[0].plot(t,np.linalg.norm(v,axis=1),lw=1.3);ax[0].axhline(12,color='r',ls='--',lw=.8,label='12 m/s limit')
    for c in r['crossings']:
        ax[0].axvline(c['time'],color='.6',lw=.6);ax[0].text(c['time'],.5,str(c['gate']+1),fontsize=8)
    ax[0].set(xlabel='Race time (s)',ylabel='Speed (m/s)',title='Actual body speed' if px4 else 'Nominal model speed');ax[0].legend()
    ax[1].plot(p[:,0],p[:,1],lw=1);ax[1].scatter([w.center0[0] for w in track.windows],[w.center0[1] for w in track.windows],marker='s',s=35)
    for i,w in enumerate(track.windows):ax[1].text(w.center0[0],w.center0[1],f' W{i+1}',fontsize=8)
    ax[1].set(xlabel='x (m)',ylabel='y (m)',title='Top view');ax[1].axis('equal')
    fig.suptitle(('PX4 / Gazebo' if px4 else 'Same-model diagnostic')+f" — completed={r['completed']}")
    fig.savefig(path/'flight.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(1,7,figsize=(14,2.6),constrained_layout=True)
    for i,(w,ax) in enumerate(zip(track.windows,axes)):
        if w.aperture.kind=='circle':
            angle=np.linspace(0,2*np.pi,150);boundary=w.aperture.radius*np.column_stack((np.cos(angle),np.sin(angle)))
        else:boundary=np.vstack((w.aperture.physical_vertices,w.aperture.physical_vertices[0]))
        ax.plot(boundary[:,0],boundary[:,1],'k-',lw=1)
        # Plot centre-plane crossing, not the later full-body exit point.
        previous=track.start if i==0 else track.windows[i-1].state_at(0)[0]
        c,rot,*_=w.state_at(0);sign=-np.sign(rot[:,2]@(previous-c))
        local=np.array([np.r_[*w.world_to_local(pos,float(at))] for pos,at in zip(p,t)])
        z=sign*local[:,2]
        for k in np.flatnonzero((z[:-1]<=0)&(z[1:]>0)):
            f=-z[k]/(z[k+1]-z[k]);xy=(1-f)*local[k,:2]+f*local[k+1,:2]
            if w.aperture.contains(xy):ax.scatter(*xy,c='tab:blue',s=18)
        ax.set_title(f'W{i+1}');ax.axis('equal');ax.grid(alpha=.2)
    fig.suptitle('Centre-plane crossing locations (local gate coordinates, m)')
    fig.savefig(path/'crossings.png',dpi=160);plt.close(fig)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);report(p.parse_args().run)
