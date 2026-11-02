import json
from pathlib import Path
import numpy as np
from convex_timevarying_window.comparisons.single_gate_swing.audit import audit as shared_audit
from .scenario import Case, build_case


def audit(path):
    path=Path(path);raw=json.loads((path/'result.json').read_text())
    case=Case(**raw['case']);track,_=build_case(case);w=track.windows[0]
    result=shared_audit(path,track=track,condition=dict(speed_mps=case.speed,
        direction='approaching' if case.speed>0 else 'receding' if case.speed<0 else 'static'))
    for event in result['crossings']:
        t=event['time_s'];delta=w.state_at(t)[0]-track.start
        derivative=float(delta@w.motion.velocity/np.linalg.norm(delta))
        event['distance_to_start_rate_mps']=derivative
        event['approaching_start_at_crossing']=derivative<0
        event['gate_velocity_enu_mps']=w.motion.velocity.tolist()
        accepted=next((s for s in raw['solves'] if s['accepted']),None)
        if accepted:
            event['planned_point_local_m']=accepted['variables'][2:4]
            event['planned_crossing_time_s']=accepted['made_at']+accepted['durations'][0]
    result['initial_lateral_offset_m']=case.offset
    result['closest_to_start_time_s']=case.offset/case.speed if case.speed>0 else None
    (path/'comparison_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    return result
