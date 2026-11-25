"""Deterministic ideal/lagged closed-loop tests, not a PX4 substitute."""
from __future__ import annotations
import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import time
import numpy as np

from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from .planner import Config, Planner


def advance_gate(planner, gate, p0, p1, t0, t1):
    if gate >= len(planner.windows): return False, None
    w = planner.windows[gate]
    q0,z0 = w.world_to_local(p0,t0)
    q1,z1 = w.world_to_local(p1,t1)
    sign = planner.signs[gate]
    if sign*z0 >= 0 and sign*z1 < 0:
        alpha = abs(z0)/(abs(z0)+abs(z1))
        q = (1-alpha)*q0+alpha*q1
        margin = float(w.aperture.boundary_distance(q))-planner.cfg.radius
        return w.aperture.contains(q) and margin >= 0, {'gate':gate,'time':t0+alpha*(t1-t0),'margin':margin,'q':q.tolist()}
    return False,None


def run(outdir, phase=0., layout=0., lag=0., max_time=65., cfg=Config()):
    track,_ = build_track()
    for i,w in enumerate(track.windows):
        w.motion = replace(w.motion, phase=w.motion.phase+phase)
        w.center0 = w.center0+layout*np.array([np.sin(1.7*i),np.cos(2.1*i),.2*np.sin(i)])
    outdir = Path(outdir); outdir.mkdir(parents=True,exist_ok=False)
    build_started=time.perf_counter(); planner=Planner(track,cfg)
    build_s=time.perf_counter()-build_started
    state=np.r_[track.start,np.zeros(6)]; active=None; gate=0; crossings=[]
    dt=.02; now=0.; next_solve=0.; rows=[]; timings=[]; solves=[]; failed=None
    # No trajectory seed: repeated budgeted attempts while at the initial state.
    startup=time.perf_counter()
    for attempt in range(12):
        p=planner.solve(state,0.,0)
        timings.append(p.solve_ms)
        if p.violation < .003 and p.solve_ms < 100:
            active=p; break
    initial_s=time.perf_counter()-startup
    if active is None: failed='initial_feasible_plan_not_found'
    while failed is None and now < max_time:
        if now >= next_solve and (gate >= len(planner.windows) or active.first_gate != gate or active.crossing_times[0]-now > .25):
            tick=time.perf_counter()
            p=planner.solve(state,now,gate)
            clearance=planner.prefix_clearance(p,now)
            cycle_ms=1000*(time.perf_counter()-tick)
            timings.append(cycle_ms)
            accepted=p.violation<.003 and clearance>=cfg.tracking_margin and cycle_ms<100
            solves.append({'time':now,'ms':cycle_ms,'violation':p.violation,'accepted':accepted,'status':p.status,'gate':gate})
            if accepted: active=p
            next_solve=now+cfg.replan_period
        if now+dt >= active.end_time:
            if gate==len(planner.windows) and np.linalg.norm(state[:3]-track.goal)<.55 and np.linalg.norm(state[3:6])<1.: break
            failed='accepted_plan_expired'; break
        previous=state[:3].copy()
        if lag <= 0:
            state=np.concatenate([active.sample(now+dt,d) for d in range(3)])
        else:
            pr,vr,ar=[active.sample(now,d) for d in range(3)]
            desired=ar+cfg.response_time*active.sample(now,3)+4*(pr-state[:3])+3*(vr-state[3:6])
            acc=state[6:9]+min(1.,dt/lag)*(desired-state[6:9])
            state[:3]+=dt*state[3:6]+.5*dt*dt*acc
            state[3:6]+=dt*acc; state[6:9]=acc
        now+=dt
        passed,cross=advance_gate(planner,gate,previous,state[:3],now-dt,now)
        if passed:
            gate+=1; crossings.append(cross); next_solve=now
            print('pass',gate,round(now,2),flush=True)
        clearance=min(np.hypot(float(w.aperture.boundary_distance(w.world_to_local(state[:3],now)[0])),w.world_to_local(state[:3],now)[1])-cfg.radius-cfg.frame_thickness for w in planner.windows)
        rows.append(np.r_[now,state,clearance])
        if clearance < 0: failed='sampled_sphere_collision'
        if gate==len(planner.windows) and np.linalg.norm(state[:3]-track.goal)<.55 and np.linalg.norm(state[3:6])<1.: break
    if failed is None and now>=max_time: failed='timeout'
    data=np.array(rows)
    if rows: np.savez_compressed(outdir/'trace.npz',trace=data)
    result={'completed':failed is None,'failure':failed,'flight_time_s':now,'crossings':crossings,
            'config':asdict(cfg),'phase':phase,'layout':layout,'lag':lag,'solver_build_s':build_s,
            'initial_plan_wall_s':initial_s,'cycle_ms':{'max':max(timings),'mean':float(np.mean(timings)),'p99':float(np.percentile(timings,99))},
            'minimum_sampled_clearance':float(data[:,-1].min()) if rows else None,'solves':solves,
            'evidence':'20ms sampled ideal/acceleration-lag model; not PX4 or continuous certification'}
    (outdir/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='solves'},indent=2),flush=True)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('--outdir',required=True)
    parser.add_argument('--phase',type=float,default=0.); parser.add_argument('--layout',type=float,default=0.)
    parser.add_argument('--lag',type=float,default=0.)
    args=parser.parse_args(); run(args.outdir,args.phase,args.layout,args.lag)
