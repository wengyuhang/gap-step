"""Observe actual moving seven-gate poses without arming or flight."""
import json,subprocess,time
from pathlib import Path
from .scenario import Case,build_case,launch,ASSETS,manifest
from .audit import pose_audit

def main():
    out=Path('convex_timevarying_window/comparisons/论文实验/实验2/results/high_motion_scene_probe_v1_20261105').resolve();out.mkdir(parents=True,exist_ok=False)
    case=Case('slalom','high');track,_=build_case(case);launch(track,ASSETS,out)
    with (out/'gate_pose.pbtxt').open('w') as f:
        p=subprocess.Popen(['docker','exec','-e','GZ_PARTITION=convex_seven_togt_experiment_partition','convex_seven_togt_experiment','gz','topic','-e','-t','/world/convex_seven_dynamic_px4_togt/pose/info'],stdout=f,stderr=subprocess.DEVNULL)
        try:
            deadline=time.monotonic()+55
            while time.monotonic()<deadline:
                text=(out/'gate_pose.pbtxt').read_text(errors='replace')[-300000:]
                import re
                seconds=re.findall(r'(?m)^    sec: (\d+)',text)
                if seconds and int(seconds[-1])>=34:break
                time.sleep(.5)
        finally:
            subprocess.run(['docker','exec','-e','GZ_PARTITION=convex_seven_togt_experiment_partition','convex_seven_togt_experiment','gz','service','-s','/world/convex_seven_dynamic_px4_togt/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','2000','--req','pause: true'],capture_output=True,timeout=5)
            p.terminate();p.wait(timeout=5)
    result=pose_audit(out,track,5.);(out/'scene.json').write_text(json.dumps(manifest(case),indent=2)+'\n');print(json.dumps(result,indent=2),flush=True)
if __name__=='__main__':main()
