"""Local passage endpoints and recovery merit, not a viability certificate."""
import numpy as np

# Covers one 100 ms solve, 80 ms activation lead and a 100 ms execution
# prefix, rounded up. No extra waiting before TOGT is introduced.
PASSAGE_TAIL_STEPS=30


def checked_end_steps(indices,first,event_tail,xp=np):
    steps=indices.shape[1]-1
    if not event_tail:return xp.full(len(indices),steps,dtype=xp.int32)
    forward=indices[:,-1]>first
    exits=xp.argmax(indices>first,axis=1)
    return xp.where(forward,xp.minimum(steps,2*((exits+PASSAGE_TAIL_STEPS+1)//2)),steps).astype(xp.int32)


def entry_merit(states,pose,previous_pose,sign,cfg,xp=np):
    """Time-scaled entry alignment proxy. Never used as safety authority.

    Use the same future gate pose for initial/terminal comparisons, so gate
    motion alone cannot manufacture progress. Penalize lateral velocity too.
    """
    rotation=pose[...,3:].reshape(pose.shape[:-1]+(3,3))
    q=xp.einsum('...i,...ij->...j',states[...,:3]-pose[...,:3],rotation)
    frame_v=(pose[...,:3]-previous_pose[...,:3])/.01
    v=xp.einsum('...i,...ij->...j',states[...,3:6]-frame_v,rotation)
    depth=cfg.radius+cfg.error_margin+cfg.frame_thickness+.3
    a=.65*cfg.acceleration
    lateral=xp.linalg.norm(q[...,:2],axis=-1)
    lateral_v=xp.linalg.norm(v[...,:2],axis=-1)
    normal=xp.abs(sign*q[...,2]+depth)
    away=xp.maximum(0.,-sign*v[...,2])
    return xp.maximum(normal/cfg.speed,xp.sqrt(2.*normal/a))+xp.sqrt(2.*lateral/a)+lateral_v/a+away/a


def candidate_pool(valid,indices,phases,first,crossing,avoid,now,deadline,
                   recovery_progress=None,xp=np):
    pool=valid.copy();forward=indices[:,-1]>first
    exits=xp.argmax(indices>first,axis=1)
    pool &= (~forward)|(exits<=indices.shape[1]-1-PASSAGE_TAIL_STEPS)
    if crossing==2:pool &= forward
    if deadline is not None:pool &= forward&(now+.01*exits<=deadline+1e-7)
    if (crossing or avoid) and bool(xp.any(pool&forward)):
        pool &= forward
    elif avoid and not crossing and recovery_progress is not None:
        # Safe circling is not an accepted recovery update. If no improving
        # candidate exists, retain the checked old policy or report no update.
        pool &= recovery_progress
    return pool
