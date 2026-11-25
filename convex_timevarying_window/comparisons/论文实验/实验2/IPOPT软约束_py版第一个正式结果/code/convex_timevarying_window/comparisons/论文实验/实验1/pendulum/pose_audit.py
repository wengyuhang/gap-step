"""Independent native Gazebo gate-pose check against the shared ODE table."""
import json
import re
import numpy as np
from scipy.spatial.transform import Rotation

def fields(text):
    return {key:float(value) for key,value in re.findall(r'([xyzw]):\s*([-+0-9.eE]+)',text)}

def audit(path,window,last_time):
    records=[]
    for message in re.split(r'(?m)(?=^header \{)',(path/'gate_pose.pbtxt').read_text(errors='replace')):
        stamp=re.search(r'stamp\s*\{([^{}]*)\}',message)
        if stamp is None:continue
        sec=re.search(r'sec:\s*(\d+)',stamp[1]);nsec=re.search(r'nsec:\s*(\d+)',stamp[1]);absolute=(int(sec[1]) if sec else 0)+(int(nsec[1]) if nsec else 0)*1e-9
        t=absolute-30.
        if t<0 or t>last_time:continue
        for block in re.split(r'(?m)^pose \{',message)[1:]:
            if re.search(r'name:\s*"gate_01_pendulum"',block) is None:continue
            pos=re.search(r'position\s*\{([^{}]*)\}',block);quat=re.search(r'orientation\s*\{([^{}]*)\}',block)
            if pos is None or quat is None:continue
            a,b=fields(pos[1]),fields(quat[1]);position=np.array([a.get(k,0.) for k in ('x','y','z')]);q=np.array([b.get(k,0.) for k in ('x','y','z','w')])
            c,R,*_=window.state_at(t);rotation=Rotation.from_quat(q).as_matrix()
            records.append([t,float(np.linalg.norm(position-c)),float(Rotation.from_matrix(rotation.T@R).magnitude())])
    if len(records)<3:raise RuntimeError('insufficient independent gate-pose samples')
    records=np.asarray(records);speed=2*np.max(abs(window.motion.theta(window.motion.times,1)));rate=speed/2
    position_limit=2*speed*.001+1e-4;angle_limit=2*rate*.001+1e-4
    result=dict(samples=len(records),first_time_s=float(records[0,0]),last_time_s=float(records[-1,0]),
        maximum_position_error_m=float(records[:,1].max()),maximum_rotation_error_rad=float(records[:,2].max()),
        allowed_position_error_m=float(position_limit),allowed_rotation_error_rad=float(angle_limit),
        passed=bool(records[:,1].max()<position_limit and records[:,2].max()<angle_limit),
        evidence='native Gazebo pose/info, not commanded pose alone')
    (path/'gate_pose_audit.json').write_text(json.dumps(result,indent=2)+'\n');np.savez_compressed(path/'gate_pose_error.npz',records=records)
    if not result['passed']:raise RuntimeError('physical gate pose differs from planning query')
    return result
