"""Three predeclared PX4 development cases; all attempts retained."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

import numpy as np

from convex_timevarying_window.comparisons.checked_soft_mpc_racing.run import inspect, source_hashes as parent_hashes
from convex_timevarying_window.comparisons.checked_soft_racing.run import save, ROOT

CASES = [(1,2,1,1),(1,1,1,0),(10,4,1,1)]


def source_hashes():
    hashes=parent_hashes()
    for path in Path(__file__).parent.glob('*.py'):
        hashes[str(path.relative_to(ROOT))]=hashlib.sha256(path.read_bytes()).hexdigest()
    for path in (Path(__file__).parent/'fast').glob('*.py'):
        hashes[str(path.relative_to(ROOT))]=hashlib.sha256(path.read_bytes()).hexdigest()
    return hashes


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True)
    p.add_argument('--condensed',action='store_true');a=p.parse_args()
    out=a.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    hashes=source_hashes()
    save(out/'protocol.json',dict(cases=CASES,source_sha256=hashes,created=time.time(),condensed=a.condensed,
        changes='Persistent independent planner process; unchanged MPC and planning constraints',
        acceptance='Same primary/contact/truth/deadline and separate strict 12/8/42 diagnostic',
        max_attempts=2,retry='Only independently confirmed preflight infrastructure failure'))
    for relative in hashes:
        dest=out/'source_snapshot'/relative;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/relative,dest)
    rows=[]
    for case,(rate,tr,rot,phase_id) in enumerate(CASES):
        for attempt in (1,2):
            if source_hashes()!=hashes:raise RuntimeError('Frozen optimization experiment sources changed')
            path=out/f'r{rate}_t{tr}_a{rot}_p{phase_id}_attempt{attempt}'
            save(out/'status.json',dict(state='running',case=case+1,total=len(CASES),run=str(path),pid=os.getpid()))
            print('START '+path.name,flush=True)
            command=[sys.executable,'-u','-m','convex_timevarying_window.checked_soft_mpc_isolated.px4',
                     '--outdir',str(path),'--motion-rate',str(rate),'--translation-scale',str(tr),
                     '--rotation-scale',str(rot),'--phase',str(phase_id*2*np.pi/3)]
            if a.condensed:command.append('--condensed')
            began=time.perf_counter()
            try:
                p=subprocess.run(command,cwd=ROOT,capture_output=True,text=True,timeout=240)
                console=p.stdout+'\n'+p.stderr;code=p.returncode
            except subprocess.TimeoutExpired as exc:
                console=repr(exc);code=None
            path.mkdir(parents=True,exist_ok=True);(path/'console.log').write_text(console)
            row=dict(method='checked_soft_mpc_isolated',condition_id=[rate,tr,rot,phase_id],attempt=attempt,
                     run=str(path),wall_s=time.perf_counter()-began,returncode=code)
            if (path/'result.json').exists():row.update(inspect(path))
            else:row.update(realtime_success=False,physical_success=False,infrastructure_failure=False,failure='missing_result')
            rows.append(row);save(out/'summary.json',rows)
            print('RESULT '+json.dumps(row,ensure_ascii=False),flush=True)
            if not row['infrastructure_failure']:break
    save(out/'status.json',dict(state='complete',conditions=len(CASES),attempts=len(rows)))


if __name__=='__main__':main()
