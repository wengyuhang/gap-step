"""Generic kinematic warm/cold seed; no experiment data dependency."""
import numpy as np
from convex_timevarying_window.wang_free_state_mpc.trajectory import arrival_jerk_for_middle

def seed(planner,state,now,first,travel_speed,gate_speed):
    ids=planner.ids(first);gates=len(ids);legs=gates+1;vi,ai,ji,qi=planner.offsets(gates)
    x=np.zeros(4*legs+8*gates);head=np.asarray(state[:9]).reshape(3,3);stamp=now
    old=planner.admitted;old_ids=planner.ids(old.first_gate) if old is not None else []
    for k in range(legs):
        terminal=k==gates
        wi=(ids[-1]+1) if terminal and planner.free_terminal(first) else None if terminal else ids[k]
        old_k=old_ids.index(wi) if not terminal and wi in old_ids else None
        reuse=old_k is not None
        q=old.x[planner.offsets(len(old_ids))[3]+2*old_k:planner.offsets(len(old_ids))[3]+2*old_k+2] if reuse else np.zeros(2)
        target=planner.track.goal if wi is None else planner.windows[wi].state_at(stamp)[0]
        if reuse:T=float(old.crossing_times[old_k]-stamp)
        else:
            distance=np.linalg.norm(target-head[0])
            T=max(.8,distance/travel_speed,np.sqrt(3*distance/planner.cfg.acceleration))
        if T<=planner.cfg.min_leg_seconds:raise ValueError('committed seed')
        if wi is not None:
            center,R=planner.windows[wi].state_at(stamp+T)[:2];target=center if terminal else center+R[:,:2]@q
        if terminal:velocity=np.zeros(3);acceleration=np.zeros(3)
        elif reuse:velocity=old.sample(stamp+T,1);acceleration=old.sample(stamp+T,2)
        else:
            nxt=planner.windows[wi+1].center0 if wi+1<len(planner.windows) else planner.track.goal
            direction=nxt-head[0];velocity=direction/max(np.linalg.norm(direction),1e-9)*gate_speed
            acceleration=np.zeros(3)
        middle=old.sample(stamp+.5*T) if reuse else .5*(head[0]+target)
        tail=np.array([target,velocity,acceleration]);x[k]=np.log(T)
        if not terminal:
            x[vi+3*k:vi+3*k+3]=velocity/planner.cfg.speed
            x[ai+3*k:ai+3*k+3]=acceleration/planner.cfg.acceleration
            x[qi+2*k:qi+2*k+2]=q
        x[ji+3*k:ji+3*k+3]=arrival_jerk_for_middle(head,tail,middle,T)/planner.cfg.jerk
        head=tail;stamp+=T
    return x
