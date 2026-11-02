"""Independent ULog body-centre event audit; contacts decide frame collision."""
import json
from pathlib import Path
import numpy as np
from pyulog import ULog
from scipy.spatial.transform import Rotation
from convex_timevarying_window.three_phase_mppi.report import continuous_quaternions
from convex_timevarying_window.single_gate_stop_racing.events import Progress
from .scenario import Case,build_case


def audit(path):
    path=Path(path);r=json.loads((path/'result.json').read_text())
    track,_=build_case(Case(**r['case']));w=track.windows[0];R=w.state_at(0.)[1]
    u=ULog(str(path/'flight.ulg'));d=u.get_dataset('vehicle_local_position_groundtruth').data
    a=u.get_dataset('vehicle_attitude_groundtruth').data
    t=d['timestamp']*1e-6;selected=(t>=30)&(t<=30+r['last_time_s']);t=t[selected]
    q=continuous_quaternions(np.column_stack([a[k] for k in ('q[1]','q[2]','q[3]','q[0]')]))
    qs=np.column_stack([np.interp(t,a['timestamp']*1e-6,q[:,i]) for i in range(4)])
    pos=np.column_stack([d[k][selected] for k in ('x','y','z')])
    pos+=Rotation.from_quat(qs).apply(np.tile([0.,0.,-.24],(len(t),1)))
    pos=pos[:,[1,0,2]]*[1,1,-1];t=t-30.
    if len(t)<3:raise RuntimeError('insufficient truth samples')
    centers=np.array([w.state_at(float(at))[0] for at in t]);local=(pos-centers)@R
    sign=float(np.sign((track.start-w.center0)@R[:,2]))
    progress=Progress([w],[sign],r['config']['radius'],r['config']['frame_thickness'])
    for at,p in zip(t,pos):progress.observe(p,float(at))
    z=sign*local[:,2];events=[]
    velocity=np.gradient(pos,t,axis=0);relative=(velocity-w.motion.velocity)@R
    for k in np.flatnonzero(z[:-1]*z[1:]<0):
        alpha=abs(z[k])/(abs(z[k])+abs(z[k+1]));at=t[k]+alpha*(t[k+1]-t[k])
        point=(1-alpha)*local[k]+alpha*local[k+1]
        if w.aperture.contains(point[:2]):
            events.append(dict(time_s=float(at),forward=bool(z[k]>0),point_local_m=point[:2].tolist(),
                gate_speed_mps=float(np.linalg.norm(w.motion.velocity)),
                relative_lateral_speed_mps=float(np.interp(at,t,relative[:,1])),
                normal_speed_mps=float(-sign*np.interp(at,t,relative[:,2]))))
    once=len(events)==1 and events[0]['forward'];h=r['config']['radius']+r['config']['frame_thickness']
    entry=exit=None
    if once:
        at=events[0]['time_s'];before=np.flatnonzero((t<at)&(z>=h));after=np.flatnonzero((t>at)&(z<=-h))
        if len(before):entry=float(t[before[-1]])
        if len(after):exit=float(t[after[0]])
    goal_error=float(np.linalg.norm(pos[-1]-track.goal));goal_speed=float(np.linalg.norm(velocity[-1]))
    collision=r['collision_detected'];accepted=sum(s['accepted'] for s in r['solves'])
    passed=bool(r['completed'] and once and exit is not None and goal_error<.55 and not collision)
    verdict=('collision' if collision else 'completed_no_frame_contact' if passed else
             'online_no_accepted_plan' if accepted==0 and r['failure']=='timeout' else
             'missed_gate' if 'missed_opening' in str(r['failure']) else 'not_completed')
    mpc=[s['cycle_ms'] for s in r['mpc_cycles'] if s.get('time',0)>=0]
    planning=[s['cycle_ms'] for s in r['solves']]
    result=dict(speed_mps=r['case']['speed'],variant=r['comparison_variant'],verdict=verdict,
        passed=passed,completed=r['completed'],collision_detected=collision,contacts=r['contacts'],
        flight_time_s=r['flight_time_s'] if passed else None,raw_completion_time_s=r['flight_time_s'],
        failure=r['failure'],crossings=events,ordered_once=once,body_exit_confirmed=exit is not None,
        entry_time_s=entry,exit_time_s=exit,body_passage_duration_s=exit-entry if exit is not None and entry is not None else None,
        truth_goal_error_m=goal_error,truth_endpoint_speed_mps=goal_speed,
        truth_step_median_s=float(np.median(np.diff(t))),actual_distance_m=float(np.linalg.norm(np.diff(pos,axis=0),axis=1).sum()),
        maximum_speed_mps=float(np.linalg.norm(velocity,axis=1).max()),accepted_plan_count=accepted,
        planner_max_ms=max(planning,default=None),planner_over_100ms=sum(x>100 for x in planning),
        mpc_max_ms=max(mpc,default=None),mpc_over_100ms=sum(x>100 for x in mpc),
        evidence='native-rate ULog body-centre truth, Gazebo frame contacts; not a closed-loop safety certificate')
    np.savez_compressed(path/'mechanism_trace.npz',times=t,positions=pos,gate_centers=centers,
        local_positions=local,relative_velocity_local=relative,velocity=velocity)
    (path/'comparison_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    return result
