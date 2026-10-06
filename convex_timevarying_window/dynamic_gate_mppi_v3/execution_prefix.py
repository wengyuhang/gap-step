"""Short actual-feedback checks; not a full-passage or robustness certificate."""
import numpy as np


def prefix_violation(model, states, times, limits=None, geometry=None):
    x=np.asarray(states);cfg=model.config if limits is None else limits
    if not np.all(np.isfinite(x)):return 'nonfinite'
    if np.any(x[:,:3]<cfg.lower) or np.any(x[:,:3]>cfg.upper):return 'workspace'
    if np.any(np.linalg.norm(x[:,3:6],axis=1)>cfg.speed):return 'speed'
    if np.any(1-2*(x[:,7]**2+x[:,8]**2)<np.cos(np.deg2rad(cfg.tilt))):return 'tilt'
    if np.any(np.linalg.norm(np.diff(x[:,3:6],axis=0)/np.diff(times)[:,None],axis=1)>cfg.acceleration):return 'acceleration'
    if np.any(np.linalg.norm(x[:,10:12],axis=1)>3.839724354) or np.any(np.abs(x[:,12])>3.490658504):return 'body_rate'
    if np.any(x[:,13:]<150-1e-6) or np.any(x[:,13:]>1000+1e-6):return 'motor_speed'
    clear=(model.geometry if geometry is None else geometry).interval_clearances(x[:,:3],x[:,3:6],np.asarray(times))
    if np.any(clear<=0):return 'collision'
    return None


def feedback_prefix(model,policy,state,now,duration=.1,previous=None):
    """Recheck the existing policy with the actual 100 Hz feedback hold."""
    states=[state.copy()];times=now+np.arange(int(round(duration/.01))+1)*.01
    limited=previous is not None
    for t in times[:-1]:
        u=policy.command(states[-1],float(t),previous,.01) if limited else policy.command(states[-1],float(t))
        states.append(model.integrate(states[-1],u,.01))
        previous=u
    return np.asarray(states),times


def prefix_task_violation(track,progress,states,times,cfg):
    if progress.first>=len(track.order):return None
    w=track.windows[track.order[progress.first]]
    margin=cfg.radius+cfg.error_margin;slab=margin+cfg.frame_thickness
    for x,t in zip(states,times):
        q,z=w.world_to_local(x[:3],float(t));z*=progress.signs[progress.first]
        if abs(z)<=slab and (not w.aperture.contains(q) or w.aperture.boundary_distance(q)<=margin):return 'passage'
        if progress.crossing==2 and z<0:return 'post_center_reverse'
    return None
