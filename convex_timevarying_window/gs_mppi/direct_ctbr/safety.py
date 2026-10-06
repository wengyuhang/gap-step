"""Current-command CTBR safety layer for the measured 17-state plant.

The 20 ms interval is checked against known moving gate poses at 5 ms spacing.
If a local projection fails, a small input-space search avoids declaring a
false impossibility.  This is numerical one-step admission, not a certificate
of recursive feasibility or a substitute for the next measured-state check.
"""
import numpy as np
from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import MASS

# Per-publication continuity budget. This is a preference subordinate to the
# verified physical interval: if no safe command fits, widen it rather than
# publishing an unverified smooth command or declaring the state impossible.
SLEW_PER_SEND=np.array([2.,.3,.3,.2])


def filter_current(engine, state, proposal, now, first, phase, mode=0, dt=.02,
                   poses=None, reference=None, previous=None):
    if poses is None:
        poses=engine.poses(now+np.linspace(0.,dt,5))
    if reference is None:
        reference=np.tile(np.r_[state[:3],np.zeros(6)],(2,1))

    def evaluate(command, iterations):
        command=np.asarray(command,dtype=float)
        metrics,_,actions=engine.rollout(state,command[None,None],
                                         reference,poses,first,phase,mode,dt,
                                         gpu=False,projection_iterations=iterations,
                                         previous_command=command)
        return metrics,actions[0].copy() if len(actions) else None

    raw=np.asarray(proposal,dtype=float)
    prior=None if previous is None else np.asarray(previous,dtype=float)
    attempts=0;first_check=None
    levels=(1.,2.,4.,None) if prior is not None else (None,)
    for level in levels:
        candidate=raw if level is None else np.clip(raw,prior-level*SLEW_PER_SEND,prior+level*SLEW_PER_SEND)
        check,action=evaluate(candidate,4);attempts+=1
        if first_check is None:first_check=check
        if action is not None and check[1]==0 and (level is None or
                np.all(np.abs(action-prior)<=level*SLEW_PER_SEND+1e-8)):
            return check,action,attempts,False

    thrust=np.clip(raw[0],4*engine.config.min_rotor_thrust,4*engine.config.max_rotor_thrust)
    hover=MASS*9.8066
    # Search only the four physical CTBR controls, not a new path.  Braking
    # thrust and holding measured body rates can succeed when the projection
    # gradient is dominated by a delayed motor/attitude constraint.
    levels=[thrust,.9*thrust,.8*thrust,.7*thrust,.6*thrust,
            hover,.8*hover,1.1*hover,4*engine.config.min_rotor_thrust]
    rates=[raw[1:],np.asarray(state[10:13]),np.zeros(3)]
    best=None;best_score=float('inf')
    for rate in rates:
        for level in levels:
            candidate=np.r_[level,rate]
            if np.allclose(candidate,raw,atol=1e-10):continue
            check,action=evaluate(candidate,0);attempts+=1
            if action is None or check[1]!=0:continue
            delta=(action-raw)/np.array([hover,1.,1.,1.])
            score=float(delta@delta)
            if score<best_score:best_score=score;best=(check,action)
    if best is None:return first_check,None,attempts,True
    return best[0],best[1],attempts,True
