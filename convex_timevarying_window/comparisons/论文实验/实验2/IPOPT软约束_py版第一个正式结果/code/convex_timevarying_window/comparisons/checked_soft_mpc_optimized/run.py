"""Run all 38 conditions with the frozen optimized MPC; retain every attempt."""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time

import numpy as np

from convex_timevarying_window.comparisons.checked_soft_racing.run import ROOT, conditions, save
from convex_timevarying_window.comparisons.checked_soft_mpc_racing.run import (
    inspect, condition_rows, source_hashes as original_hashes)
from convex_timevarying_window.checked_soft_mpc_racing.mpc import Config


def source_hashes():
    result=original_hashes()
    method=ROOT/'convex_timevarying_window/checked_soft_mpc_isolated'
    paths=list(method.glob('*.py'))+list((method/'fast').glob('*.py'))
    paths += [Path(__file__),Path(__file__).with_name('PROTOCOL.md')]
    for path in paths: result[str(path.relative_to(ROOT))]=hashlib.sha256(path.read_bytes()).hexdigest()
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);p.add_argument('--resume',action='store_true')
    a=p.parse_args();out=a.outdir.resolve();out.mkdir(parents=True,exist_ok=a.resume)
    hashes=source_hashes()
    if a.resume:
        protocol=json.loads((out/'protocol.json').read_text())
        if protocol['source_sha256']!=hashes:raise RuntimeError('Frozen optimized sources changed')
        rows=json.loads((out/'summary.json').read_text()) if (out/'summary.json').exists() else []
    else:
        devpath=ROOT/'convex_timevarying_window/checked_soft_mpc_isolated/results/px4_v2_condensed_20261006/r10_t4_a1_p1_attempt1/result.json'
        dev=json.loads(devpath.read_text())
        if not dev.get('condensed'):raise RuntimeError('Development reference not condensed')
        for relative in ('checked_soft_mpc_isolated/px4.py','checked_soft_mpc_isolated/worker.py','checked_soft_mpc_isolated/fast/mpc.py'):
            key='convex_timevarying_window/'+relative
            if hashes[key]!=dev['source_sha256'][key]:raise RuntimeError('Different controller from declared development version')
        protocol=dict(method='checked_soft_mpc_optimized',source_sha256=hashes,conditions=conditions(),
            mpc_config=asdict(Config()),condensed=True,isolated=True,max_flight_time_s=60.,
            development_reference=str(devpath),python=platform.python_version(),numpy=np.__version__,platform=platform.platform(),
            environment={k:os.environ.get(k) for k in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','GZ_SHOW_GUI')},
            nominal_dynamics='Diagnostic, not primary rejection, per user clarification',created=time.time())
        save(out/'protocol.json',protocol);rows=[]
        for relative in hashes:
            dest=out/'source_snapshot'/relative;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/relative,dest)
    for case,key in enumerate(conditions()):
        prior=sorted([r for r in rows if tuple(r['condition_id'])==key],key=lambda r:r['attempt'])
        if prior and (not prior[-1]['infrastructure_failure'] or prior[-1]['attempt']==2):continue
        for attempt in range(max((r['attempt'] for r in prior),default=0)+1,3):
            if source_hashes()!=hashes:raise RuntimeError('Frozen optimized sources changed during matrix')
            rate,tr,rot,phase_id=key;phase=phase_id*2*math.pi/3
            path=out/f'r{rate}_t{tr}_a{rot}_p{phase_id}_attempt{attempt}'
            save(out/'status.json',dict(state='running',case=case+1,total=38,condition_id=key,attempt=attempt,
                                       pid=os.getpid(),run=str(path),started=time.time()))
            print(f'START {case+1}/38 {path.name}',flush=True)
            begin=time.monotonic();code=None;timed_out=False
            if not path.exists():
                command=[sys.executable,'-u','-m','convex_timevarying_window.checked_soft_mpc_isolated.px4','--condensed',
                         '--outdir',str(path),'--motion-rate',str(rate),'--translation-scale',str(tr),
                         '--rotation-scale',str(rot),'--phase',str(phase),'--max-flight-time','60']
                try:
                    process=subprocess.run(command,cwd=ROOT,capture_output=True,text=True,timeout=240)
                    output=process.stdout+'\n'+process.stderr;code=process.returncode
                except subprocess.TimeoutExpired as exc:output=repr(exc);timed_out=True
                path.mkdir(exist_ok=True);(path/'console.log').write_text(output)
            else:output=(path/'console.log').read_text() if (path/'console.log').exists() else ''
            row=dict(method='checked_soft_mpc_optimized',condition_id=key,rate=rate,translation_scale=tr,
                rotation_scale=rot,phase=phase,phase_id=phase_id,layout=0.,attempt=attempt,run=str(path),
                wall_s=time.monotonic()-begin,returncode=code)
            if (path/'result.json').exists():row.update(inspect(path))
            else:
                started=(path/'solves_live.jsonl').exists()
                infra=not started and any(s in output for s in ('PX4 startup failed','no heartbeat','missing initial telemetry'))
                row.update(physical_success=False,realtime_success=False,infrastructure_failure=infra,
                    flight_time_s=None,failure='process_timeout' if timed_out else 'missing_result')
            rows.append(row);save(out/'summary.json',rows)
            save(out/'first_attempt_summary.json',[r for r in rows if r['attempt']==1])
            save(out/'condition_summary.json',condition_rows(rows))
            print('RESULT '+json.dumps(row,ensure_ascii=False),flush=True)
            if not row['infrastructure_failure']:break
    save(out/'status.json',dict(state='complete',conditions=len(condition_rows(rows)),attempts=len(rows),finished=time.time()))
    print('DONE '+str(out),flush=True)


if __name__=='__main__':main()
