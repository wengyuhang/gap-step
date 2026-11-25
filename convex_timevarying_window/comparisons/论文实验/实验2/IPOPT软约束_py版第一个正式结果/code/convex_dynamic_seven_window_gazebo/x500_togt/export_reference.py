#!/usr/bin/env python3
"""Export an x500-replanned result as a 100 Hz PX4 ENU reference."""
from __future__ import annotations
import argparse, hashlib, json
from pathlib import Path
import numpy as np
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track, X500Objective

def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    p=argparse.ArgumentParser(); p.add_argument("--result",type=Path,required=True); p.add_argument("--output",type=Path,required=True); a=p.parse_args()
    source=a.result.resolve(); result=json.loads(source.read_text()); track,cfg=build_track(); obj=X500Objective(track,cfg)
    f=obj.forward(np.asarray(result["decision_vector"],float)); duration=f.trajectory.total_time
    t=np.linspace(0,duration,int(np.ceil(duration/.01))+1)
    pos=np.real(f.trajectory.evaluate(t,0)); vel=np.real(f.trajectory.evaluate(t,1)); acc=np.real(f.trajectory.evaluate(t,2))
    jerk=np.real(f.trajectory.evaluate(t,3)); snap=np.real(f.trajectory.evaluate(t,4))
    a.output.parent.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(a.output,time=t,position_enu=pos,velocity_enu=vel,acceleration_enu=acc,jerk_enu=jerk,snap_enu=snap,
                        traversal_times=f.traversal_times,waypoints_enu=f.waypoints)
    meta={"construction":"joint x500 TOGT reoptimization of D and K","source_result":str(source),"source_sha256":sha(source),
          "reference_file":a.output.name,"reference_sha256":sha(a.output),"coordinate_frame":"Gazebo ENU",
          "sample_count":len(t),"nominal_sample_period_s":.01,"flight_time_s":duration,
          "maximum_speed_mps":float(np.linalg.norm(vel,axis=1).max()),
          "maximum_acceleration_mps2":float(np.linalg.norm(acc,axis=1).max()),
          "start_enu":pos[0].tolist(),"goal_enu":pos[-1].tolist(),"traversal_times_s":f.traversal_times.tolist()}
    a.output.with_suffix(".json").write_text(json.dumps(meta,ensure_ascii=False,indent=2)+"\n"); print(json.dumps(meta,indent=2))
if __name__ == "__main__": main()
