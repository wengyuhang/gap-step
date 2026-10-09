"""Read-only live Gazebo scene probe; independent from both controllers."""
import argparse
import json
from pathlib import Path
import subprocess

import numpy as np
from scipy.spatial.transform import Rotation

from .scenario import Case, build_case, NORMALS
from convex_timevarying_window.comparisons.checked_soft_racing.run import save


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--frequency',type=float,required=True)
    parser.add_argument('--amplitude',type=float,required=True)
    parser.add_argument('--phase',type=float,required=True)
    parser.add_argument('--out',type=Path,required=True)
    a = parser.parse_args()
    if a.out.exists(): raise FileExistsError(a.out)
    case = Case(a.frequency,a.amplitude,a.phase)
    track,_ = build_case(case)
    raw = subprocess.check_output(['docker','exec','-e','GZ_PARTITION=convex_seven_togt_experiment_partition',
        'convex_seven_togt_experiment','gz','topic','-e','-n','120','--json-output',
        '-t','/world/convex_seven_dynamic_px4_togt/pose/info'],text=True,timeout=10)
    messages = [json.loads(s) for s in raw.splitlines() if s.startswith('{')]
    rows = []
    for message in messages:
        stamp = message['header']['stamp']
        t = float(stamp.get('sec',0))+float(stamp.get('nsec',0))*1e-9-30
        poses = sorted((g for g in message['pose'] if g['name'].startswith('gate_')),key=lambda g:g['name'])
        if len(poses)!=7: raise ValueError('Missing live gate models')
        for i,(w,g) in enumerate(zip(track.windows,poses)):
            pos = np.array([g['position'].get(k,0.) for k in 'xyz'])
            q = [g['orientation'].get(k,0.) for k in ('x','y','z','w')]
            c,R,*_ = w.state_at(max(t,0.))
            rows.append(dict(time=t,gate=i,position=pos.tolist(),
                position_error_m=float(np.linalg.norm(pos-c)),
                plane_error_m=float(abs(NORMALS[i]@(pos-w.center0))),
                rotation_error=float(np.max(np.abs(R-Rotation.from_quat(q).as_matrix())))))
    result = dict(case=vars(case),sample_count=len(messages),comparisons=rows,
        max_position_error_m=max(r['position_error_m'] for r in rows),
        max_plane_error_m=max(r['plane_error_m'] for r in rows),
        max_rotation_matrix_error=max(r['rotation_error'] for r in rows),
        evidence='Live Gazebo pose topic at its own simulation timestamps; read-only, never used for control')
    save(a.out,result)
    print(json.dumps({k:v for k,v in result.items() if k!='comparisons'}))


if __name__=='__main__': main()
