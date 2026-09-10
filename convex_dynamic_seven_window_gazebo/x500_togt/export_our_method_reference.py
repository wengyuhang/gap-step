#!/usr/bin/env python3
"""Export the first accepted x500 conditional-CEM candidate for PX4."""
from __future__ import annotations
import argparse, hashlib, json
from pathlib import Path
import numpy as np
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track, X500Objective

def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    p=argparse.ArgumentParser(); p.add_argument("--result",type=Path,required=True); p.add_argument("--candidates",type=Path,required=True); p.add_argument("--output",type=Path,required=True); a=p.parse_args()
    summary=json.loads(a.result.read_text()); candidate_id=summary["comparison"]["conditional_dual_constraint_cem"]["candidate_id"]
    candidate=None
    for line in a.candidates.read_text().splitlines():
        row=json.loads(line)
        if row["id"]==candidate_id: candidate=row; break
    if candidate is None: raise RuntimeError(f"candidate {candidate_id} not found")
    track,cfg=build_track(); f=X500Objective(track,cfg).forward(np.asarray(candidate["x"],float)); duration=f.trajectory.total_time
    t=np.linspace(0,duration,int(np.ceil(duration/.01))+1); p0=np.real(f.trajectory.evaluate(t,0)); v=np.real(f.trajectory.evaluate(t,1)); acc=np.real(f.trajectory.evaluate(t,2)); jerk=np.real(f.trajectory.evaluate(t,3)); snap=np.real(f.trajectory.evaluate(t,4))
    a.output.parent.mkdir(parents=True,exist_ok=True); np.savez_compressed(a.output,time=t,position_enu=p0,velocity_enu=v,acceleration_enu=acc,jerk_enu=jerk,snap_enu=snap,traversal_times=f.traversal_times,waypoints_enu=f.waypoints)
    meta={"construction":"first hard-audit-accepted x500 Conditional Dual-Constraint CEM candidate","candidate_id":candidate_id,
      "result":str(a.result.resolve()),"result_sha256":sha(a.result),"candidates":str(a.candidates.resolve()),"candidates_sha256":sha(a.candidates),
      "reference_file":a.output.name,"reference_sha256":sha(a.output),"flight_time_s":duration,"sample_count":len(t),
      "maximum_speed_mps":float(np.linalg.norm(v,axis=1).max()),"maximum_acceleration_mps2":float(np.linalg.norm(acc,axis=1).max()),
      "traversal_times_s":f.traversal_times.tolist(),"coordinate_frame":"Gazebo ENU"}
    a.output.with_suffix('.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2)+"\n"); print(json.dumps(meta,indent=2))
if __name__ == '__main__': main()
