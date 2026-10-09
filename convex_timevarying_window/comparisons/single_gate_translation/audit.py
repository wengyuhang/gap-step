"""One-gate truth metrics; physical collision verdict comes from Gazebo contacts."""
import json
import numpy as np
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.audit import truth, inflated_frame
from .scenario import Case,build_case


def audit(path):
    raw=json.loads((path/'result.json').read_text())
    track,_=build_case(Case(**raw['case']))
    w=track.windows[0]
    ts,pos=truth(path,raw['last_time_s'])
    centers=np.array([w.state_at(float(t))[0] for t in ts])
    R=w.state_at(0.)[1]
    local=(pos-centers)@R
    sign=float(np.sign((track.start-w.center0)@R[:,2]))
    z=sign*local[:,2]
    h=raw['config']['radius']+raw['config']['frame_thickness']
    aperture=inflated_frame(w.aperture)
    distance=aperture.boundary_distance(local[:,:2])
    clearance=np.hypot(distance,local[:,2])-h
    crossing=[]
    for i in np.flatnonzero(z[:-1]*z[1:]<0):
        alpha=abs(z[i])/(abs(z[i])+abs(z[i+1]))
        q=(1-alpha)*local[i,:2]+alpha*local[i+1,:2]
        if w.aperture.contains(q):
            crossing.append(dict(time_s=float(ts[i]+alpha*(ts[i+1]-ts[i])),
                                 direction='forward' if z[i]>0 else 'backward',
                                 point_local_m=q.tolist(),
                                 center_offset_m=float(np.linalg.norm(q))))
    valid=len(crossing)==1 and crossing[0]['direction']=='forward'
    if valid:
        at=crossing[0]['time_s']
        before=np.flatnonzero((ts<at)&(z>=h))
        after=np.flatnonzero((ts>at)&(z<=-h))
        entry=int(before[-1]) if len(before) else None
        exit=int(after[0]) if len(after) else None
    else:entry=exit=None
    speed=np.gradient(pos,ts,axis=0)
    center_speed=np.gradient(centers,ts,axis=0)
    relative=(speed-center_speed)@R[:,1]
    where=slice(entry,exit+1) if entry is not None and exit is not None else slice(None)
    trace=np.genfromtxt(path/'telemetry_live.csv',delimiter=',',names=True)
    err=np.atleast_1d(trace['error'])
    result=dict(completed=raw['completed'],collision_detected=raw['collision_detected'],
                contacts=raw['contacts'],crossings=crossing,
                ordered_once=valid,body_exit_confirmed=exit is not None,
                entry_time_s=None if entry is None else float(ts[entry]),
                exit_time_s=None if exit is None else float(ts[exit]),
                crossing_min_proxy_clearance_m=float(np.min(clearance[where])),
                crossing_min_proxy_clearance_time_s=float(ts[(entry or 0)+np.argmin(clearance[where])]),
                crossing_relative_lateral_speed_mps=(None if not valid else
                    float(np.interp(crossing[0]['time_s'],ts,relative))),
                peak_crossing_relative_lateral_speed_mps=float(np.max(np.abs(relative[where]))),
                tracking_error_mean_m=float(np.nanmean(err)),
                tracking_error_max_m=float(np.nanmax(err)),
                actual_goal_error_m=float(np.linalg.norm(pos[-1]-track.goal)),
                flight_time_s=raw['flight_time_s'],
                passed=bool(raw['completed'] and valid and exit is not None and
                            not raw['collision_detected']),
                evidence='Gazebo contacts and 4 ms x500 body-center truth; sphere margin is a sampled proxy')
    np.savez_compressed(path/'mechanism_trace.npz',times=ts,positions=pos,gate_centers=centers,
                        local_positions=local,relative_lateral_speed=relative,proxy_clearance=clearance)
    return result
