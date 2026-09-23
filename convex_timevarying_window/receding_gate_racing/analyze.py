"""Independent post-flight audits. These costs are not online control costs."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np

from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from .scenario import vary_track


def analyze(path):
    path=Path(path)
    result=json.loads((path/'result.json').read_text())
    data=np.genfromtxt(path/'telemetry.csv',delimiter=',',names=True)
    if data.size==0: return {'passed':False,'reason':'empty_telemetry'}
    track,_=build_track()
    vary_track(track,result.get('phase',0.),result.get('layout',0.),result.get('motion_rate',1.),result.get('translation_scale',1.),result.get('rotation_scale',1.))
    tt=data['t']; actual=np.column_stack([data['x'],data['y'],data['z']])
    # Resample the recorded states at 5 ms. This cannot repair estimator bias
    # or telemetry latency; physical contact sensors remain separate evidence.
    times=np.arange(tt[0],tt[-1],.005)
    positions=np.column_stack([np.interp(times,tt,actual[:,j]) for j in range(3)])
    radius=result['config']['radius']
    per_gate=[]; ordered=[]; next_start=0
    for i,w in enumerate(track.windows):
        local=[]; clearance=[]
        for t,p in zip(times,positions):
            q,z=w.world_to_local(p,float(t))
            local.append(np.r_[q,z])
            clearance.append(np.hypot(w.aperture.boundary_distance(q),z)-radius)
        local=np.array(local)
        per_gate.append({'gate':i,'minimum_sampled_thin_frame_sphere_clearance_m':float(min(clearance))})
        source=track.start if i==0 else track.windows[i-1].center0
        c,R,*_=w.state_at(0.)
        sign=float(np.sign(R[:,2]@(source-c)) or 1.)
        selected=None
        for j in range(next_start,len(times)-1):
            if sign*local[j,2]>=0 and sign*local[j+1,2]<0:
                alpha=abs(local[j,2])/(abs(local[j,2])+abs(local[j+1,2]))
                q=(1-alpha)*local[j,:2]+alpha*local[j+1,:2]
                if w.aperture.contains(q):
                    selected={'gate':i,'time_s':float((1-alpha)*times[j]+alpha*times[j+1]),'q':q.tolist()}
                    next_start=j+1; break
        ordered.append(selected)
    durations=[s['cycle_ms'] for s in result['solves']]
    good_order=all(c is not None for c in ordered)
    goal_error=float(result.get('goal_error_m',np.linalg.norm(actual[-1]-track.goal)))
    goal_speed=float(result.get('goal_speed_mps',np.linalg.norm([data['vx'][-1],data['vy'][-1],data['vz'][-1]])))
    # Historical traces can end one telemetry interval before the in-loop
    # completion event. Report that lack of an independently saved terminal
    # sample explicitly, rather than silently treating it as a verified goal.
    completed=bool(result['completed']) and goal_error<.55 and goal_speed<1.
    contact_free=not bool(result['collision_detected'])
    deadline_pass=bool(durations) and max(durations)<100
    audit={'passed':completed and contact_free and good_order and deadline_pass,
           'completed':completed,'contact_free':contact_free,'ordered_passages_passed':good_order,
           'deadline_passed':deadline_pass,'flight_time_s':result['flight_time_s'],
           'cycle_ms':{'max':max(durations) if durations else None,'p99':float(np.percentile(durations,99)) if durations else None,
                       'mean':float(np.mean(durations)) if durations else None},
           'accepted_replans':sum(s['accepted'] for s in result['solves']),
           'total_replans':len(durations),'ordered':ordered,'per_gate':per_gate,
           'goal_error_m':goal_error,
           'goal_speed_mps':goal_speed,
           'evidence':'5ms interpolated PX4 estimator states plus separate physical Gazebo contact logs; not continuous certification'}
    (path/'audit.json').write_text(json.dumps(audit,indent=2)+'\n')
    print(json.dumps(audit,indent=2))
    return audit


if __name__=='__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('run'); args=parser.parse_args()
    analyze(args.run)
