"""Check recorded truth at contact times, including post-termination cleanup."""
import argparse
import json
from pathlib import Path
import numpy as np
from pyulog import ULog
from scipy.spatial.transform import Rotation
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.receding_gate_racing.scenario import vary_track
from convex_timevarying_window.conditional_dual_constraint_cem.safety_penalty import polygon_halfspaces


def main(root):
    root=Path(root);rows=json.loads((root/'summary.json').read_text());out=[]
    for row in rows:
        if not row.get('collision'):continue
        path=Path(row['run']);r=json.loads((path/'result.json').read_text());log=ULog(str(path/'flight.ulg'))
        truth=log.get_dataset('vehicle_local_position_groundtruth').data
        att=log.get_dataset('vehicle_attitude_groundtruth').data
        track,_=build_track();vary_track(track,r['phase'],r['layout'],r['motion_rate'],r['translation_scale'],r['rotation_scale'])
        for i,c in enumerate(r['contacts']):
            if not c['contact_pair_count']:continue
            stamp=c['first_contact_sim_time_s'];time_us=stamp*1e6
            if time_us>truth['timestamp'][-1] or time_us<truth['timestamp'][0]:
                out.append(dict(run=path.name,gate=i+1,missing_truth=True));continue
            p=np.array([np.interp(time_us,truth['timestamp'],truth[k]) for k in ['x','y','z']])
            q=np.array([np.interp(time_us,att['timestamp'],att[k]) for k in ['q[1]','q[2]','q[3]','q[0]']])
            p+=Rotation.from_quat(q).apply([0.,0.,-.24]);p=p[[1,0,2]]*np.array([1,1,-1])
            w=track.windows[i];uv,z=w.world_to_local(p,stamp-30.)
            if w.aperture.kind=='circle':distance=abs(np.linalg.norm(uv)-w.aperture.radius-.065)
            else:
                hs=polygon_halfspaces(w.aperture.physical_vertices)
                vs=np.array([np.linalg.solve(np.stack([hs.normals[j-1],hs.normals[j]]),np.array([hs.offsets[j-1],hs.offsets[j]])+.065) for j in range(len(hs.offsets))])
                distance=np.inf
                for a,b in zip(vs,np.roll(vs,-1,axis=0)):
                    e=b-a;alpha=np.clip((uv-a)@e/(e@e),0,1);distance=min(distance,np.linalg.norm(uv-a-alpha*e))
            out.append(dict(run=path.name,gate=i+1,first_contact_race_time_s=stamp-30,
                seconds_after_flight_termination=stamp-30-r['last_time_s'],
                interpolated_truth_sphere_clearance_m=float(np.hypot(distance,z)-r['config']['radius']-.085),
                note='Contact events remain authoritative; interpolated truth is a separate numerical diagnostic.'))
    (root/'contact_audit.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root');main(p.parse_args().root)
