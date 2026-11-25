"""Independent native-rate ground-truth crossing/return audit, not certification."""
import json
from pathlib import Path
import numpy as np
from pyulog import ULog
from scipy.spatial.transform import Rotation
from convex_timevarying_window.three_phase_mppi.report import continuous_quaternions
from .events import Progress
from .scenario import Case,build_case
from .planner import route_signs


def audit(path):
    path=Path(path);r=json.loads((path/'result.json').read_text())
    track,_=build_case(Case(**r['case']),r.get('first_gate_sideways_shift_m',0.),
                       r.get('horizontal_spacing_scale',1.))
    u=ULog(str(path/'flight.ulg'));d=u.get_dataset('vehicle_local_position_groundtruth').data
    a=u.get_dataset('vehicle_attitude_groundtruth').data
    t=d['timestamp']*1e-6;selection=(t>=30)&(t<=30+r['last_time_s']);t=t[selection]
    q=continuous_quaternions(np.column_stack([a[k] for k in ('q[1]','q[2]','q[3]','q[0]')]))
    qs=np.column_stack([np.interp(t,a['timestamp']*1e-6,q[:,i]) for i in range(4)])
    position=np.column_stack([d[k][selection] for k in ('x','y','z')])
    position+=Rotation.from_quat(qs).apply(np.tile([0.,0.,-.24],(len(t),1)))
    position=position[:,[1,0,2]]*np.array([1.,1.,-1.]);times=t-30.
    progress=Progress(list(track.windows),route_signs(track),r['config']['radius'],r['config']['frame_thickness'])
    for at,pos in zip(times,position):progress.observe(pos,float(at))
    goal_error=float(np.linalg.norm(position[-1]-track.goal))
    signs=route_signs(track);all_events=[]
    for i,w in enumerate(track.windows):
        normal=w.state_at(0.)[1][:,2]
        z=signs[i]*(position-w.center0)@normal;events=[]
        for k in np.flatnonzero(z[:-1]*z[1:]<0):
            alpha=abs(z[k])/(abs(z[k])+abs(z[k+1]));at=times[k]+alpha*(times[k+1]-times[k])
            pos=(1-alpha)*position[k]+alpha*position[k+1];q,_=w.world_to_local(pos,float(at))
            if w.aperture.contains(q):events.append(dict(time_s=float(at),forward=bool(z[k]>0),point=q.tolist()))
        all_events.append(events)
    exactly_once=all(len(e)==1 and e[0]['forward'] for e in all_events)
    good=bool(r['completed'] and progress.gate==7 and goal_error<.55 and not r['collision_detected'] and exactly_once)
    last_exit=progress.exits[-1]['time'] if progress.gate==7 else None
    after_last=[s for s in r['solves'] if last_exit is not None and s['time']>last_exit+.02 and s['accepted']]
    result=dict(passed=good,completed=r['completed'],collision_detected=r['collision_detected'],
        failure=r['failure'],flight_time_s=r['flight_time_s'] if good else None,
        actual_gate_count=progress.gate,crossings=progress.crossings,body_exits=progress.exits,
        all_opening_events=all_events,exactly_once=exactly_once,
        truth_goal_error_m=goal_error,truth_step_median_s=float(np.median(np.diff(times))),
        accepted_after_last_exit=len(after_last),contacts=r['contacts'],
        planner_max_ms=max((s['cycle_ms'] for s in r['solves']),default=None),
        mpc_max_ms=max((s['cycle_ms'] for s in r['mpc_cycles']),default=None),
        accepted_plan_count=sum(s['accepted'] for s in r['solves']),
        evidence='native-rate Gazebo body-center truth + physical frame contacts; not continuous proof')
    np.savez_compressed(path/'truth_trace.npz',times=times,positions=position)
    (path/'physical_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    return result
