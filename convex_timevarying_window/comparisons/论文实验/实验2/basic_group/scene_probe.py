"""Check the common high-motion world against the exact scene query, without arming."""
import argparse
import json
import re
import subprocess
import time
from pathlib import Path
from . import scenario
from ..audit import pose_audit


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);a=p.parse_args();out=a.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    track,_=scenario.build_case(scenario.Case('original','high'));scenario.launch(track,scenario.ASSETS,out)
    with (out/'gate_pose.pbtxt').open('w') as f:
        reader=subprocess.Popen(['docker','exec','-e','GZ_PARTITION=convex_seven_togt_experiment_partition','convex_seven_togt_experiment','gz','topic','-e','-t','/world/convex_seven_dynamic_px4_togt/pose/info'],stdout=f,stderr=subprocess.DEVNULL)
        try:
            deadline=time.monotonic()+80
            while time.monotonic()<deadline:
                text=(out/'gate_pose.pbtxt').read_text(errors='replace')[-300000:];seconds=re.findall(r'(?m)^    sec: (\d+)',text)
                if seconds and int(seconds[-1])>=34:break
                time.sleep(.5)
            else:raise RuntimeError('scene_preflight_timeout')
        finally:
            subprocess.run(['docker','exec','-e','GZ_PARTITION=convex_seven_togt_experiment_partition','convex_seven_togt_experiment','gz','service','-s','/world/convex_seven_dynamic_px4_togt/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','2000','--req','pause: true'],capture_output=True,timeout=5)
            reader.terminate();reader.wait(timeout=5)
    result=pose_audit(out,track,5.)
    (out/'scene.json').write_text(json.dumps(scenario.manifest(scenario.Case('original','high')),ensure_ascii=False,indent=2)+'\n')
    print('SCENE_PREFLIGHT_PASSED',json.dumps(result),flush=True)


if __name__=='__main__':main()
