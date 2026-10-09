"""Frozen paired scene-only comparison; never select attempts by outcome."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import shutil
import signal
import subprocess
import sys
import time

import numpy as np

from .scenario import Case, description
from .audit import inspect_run
from convex_timevarying_window.comparisons.checked_soft_racing.run import ROOT, save
from convex_timevarying_window.wang_free_state_mpc.experiment import source_hashes as flight_hashes

DEVELOPMENT = ROOT/'convex_timevarying_window/wang_free_state_mpc/results/event_v4_20261007/protocol.json'


def conditions(pilot=False):
    return [(.8,1.6,.4)] if pilot else [(f,a,p*2*math.pi/3) for f in (.6,1.) for a in (1.2,2.) for p in range(3)]


def selected_rows(rows):
    grouped = {}
    for row in rows:
        grouped.setdefault((row['method'],tuple(row['condition_id'])),[]).append(row)
    selected = []
    for attempts in grouped.values():
        attempts = sorted(attempts,key=lambda r:r['attempt'])
        ids = [r['attempt'] for r in attempts]
        if ids not in ([1],[1,2]) or (len(ids)>1 and not attempts[0]['infrastructure_failure']):
            raise ValueError('Invalid retry history; algorithm failures cannot be retried')
        selected.append(attempts[-1])
    return selected


def source_hashes():
    result = flight_hashes()
    if result != json.loads(DEVELOPMENT.read_text())['source_sha256']:
        raise RuntimeError('Original algorithms differ from the event-v4 snapshot')
    here = Path(__file__).parent
    paths = [here/name for name in ('scenario.py','flight.py','audit.py','run.py','PROTOCOL.md')]
    paths += [ROOT/'nonconvex_timevarying_window/sc_dynatogt/environment.py',
              ROOT/'convex_timevarying_window/geometry.py',
              ROOT/'convex_timevarying_window/comparisons/wang_free_state_mpc/run.py',
              ROOT/'convex_dynamic_seven_window_gazebo/src/PeriodicGateMotion.cc',
              ROOT/'convex_dynamic_seven_window_gazebo/build/libPeriodicGateMotion.so']
    paths += list((ROOT/'convex_dynamic_seven_window_gazebo/meshes').glob('*.stl'))
    for path in paths:
        result[str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--outdir',type=Path,required=True)
    p.add_argument('--pilot',action='store_true')
    p.add_argument('--resume',action='store_true')
    a = p.parse_args(); out = a.outdir.resolve(); matrix = conditions(a.pilot)
    hashes = source_hashes()
    out.mkdir(parents=True,exist_ok=a.resume)
    if a.resume:
        protocol = json.loads((out/'protocol.json').read_text())
        if protocol['source_sha256'] != hashes or protocol['pilot'] != a.pilot:
            raise RuntimeError('Frozen experiment differs; do not mix versions')
        rows = json.loads((out/'summary.json').read_text()) if (out/'summary.json').exists() else []
        selected_rows(rows)
    else:
        rows = []
        save(out/'protocol.json',dict(source_sha256=hashes,development_reference=str(DEVELOPMENT),
            pilot=a.pilot,conditions=matrix,scenarios=[description(Case(*c)) for c in matrix],
            methods=['ours','wang'],paired_order='alternate first method by condition index',
            max_flight_time_s=60.,process_timeout_s=240.,created=time.time(),
            python=platform.python_version(),numpy=np.__version__,platform=platform.platform(),
            environment={k:os.environ.get(k) for k in
                ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS','GZ_SHOW_GUI')},
            comparison='Frozen whole methods, not official Wang implementation or free-point-only ablation',
            acceptance='Return + contacts + sampled whole-body clearance + exactly-once forward events + <100ms',
            retry='Once only after independently confirmed infrastructure fault'))
        for relative in hashes:
            dest = out/'source_snapshot'/relative
            dest.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(ROOT/relative,dest)
    for index,key in enumerate(matrix):
        methods = ('ours','wang') if index%2==0 else ('wang','ours')
        for method in methods:
            prior = sorted((r for r in rows if r['method']==method and tuple(r['condition_id'])==key),key=lambda r:r['attempt'])
            if prior and (not prior[-1]['infrastructure_failure'] or prior[-1]['attempt']==2):
                continue
            for attempt in range(max((r['attempt'] for r in prior),default=0)+1,3):
                if source_hashes()!=hashes:
                    raise RuntimeError('Frozen sources changed during experiment')
                f,amp,phase = key
                path = out/f'case{index:02d}_{method}_attempt{attempt}'
                save(out/'status.json',dict(state='running',condition=index+1,total_conditions=len(matrix),
                    method=method,attempt=attempt,run=str(path),pid=os.getpid(),started=time.time()))
                print(f'START {index+1}/{len(matrix)} {method} attempt={attempt}',flush=True)
                start = time.monotonic(); code = None; timeout = False; recovered = path.exists()
                if not recovered:
                    command = [sys.executable,'-u','-m','convex_timevarying_window.comparisons.fixed_plane_wang.flight',
                        '--method',method,'--frequency',str(f),'--amplitude',str(amp),
                        '--phase',str(phase),'--outdir',str(path)]
                    env = dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',GZ_SHOW_GUI='0')
                    with subprocess.Popen(command,cwd=ROOT,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                                          text=True,start_new_session=True) as proc:
                        try:
                            output,_ = proc.communicate(timeout=240)
                        except subprocess.TimeoutExpired:
                            timeout = True
                            os.killpg(proc.pid,signal.SIGTERM)
                            try:
                                output,_ = proc.communicate(timeout=5)
                            except subprocess.TimeoutExpired:
                                os.killpg(proc.pid,signal.SIGKILL)
                                output,_ = proc.communicate()
                        code = proc.returncode
                    path.mkdir(exist_ok=True)
                    (path/'console.log').write_text(output)
                else:
                    output = (path/'console.log').read_text() if (path/'console.log').exists() else ''
                row = dict(method=method,condition_id=key,case=index,frequency_hz=f,amplitude_m=amp,
                    phase=phase,attempt=attempt,run=str(path),wall_s=time.monotonic()-start,
                    returncode=code,recovered_existing_run=recovered)
                if (path/'result.json').exists():
                    try:
                        row.update(inspect_run(path,method))
                    except Exception as exc:
                        row.update(physical_success=False,realtime_success=False,infrastructure_failure=False,
                                   failure='scene_or_analysis_error',analysis_error=repr(exc),flight_time_s=None)
                else:
                    started = (path/'solves_live.jsonl').exists()
                    infra = not started and any(s in output for s in ('PX4 startup failed','no heartbeat','missing initial telemetry'))
                    row.update(physical_success=False,realtime_success=False,infrastructure_failure=infra,
                        failure='process_timeout' if timeout else 'missing_result',flight_time_s=None)
                rows.append(row)
                save(path/'batch_source_reference.json',dict(protocol=str(out/'protocol.json'),source_sha256=hashes))
                save(out/'summary.json',rows)
                save(out/'first_attempt_summary.json',[r for r in rows if r['attempt']==1])
                save(out/'condition_summary.json',selected_rows(rows))
                print('RESULT '+json.dumps(row,ensure_ascii=False),flush=True)
                if not row['infrastructure_failure']:
                    break
    if source_hashes()!=hashes:
        raise RuntimeError('Frozen source mismatch after experiment')
    chosen = selected_rows(rows)
    save(out/'status.json',dict(state='complete',conditions=len(matrix),method_conditions=len(chosen),
        attempts=len(rows),passed={m:sum(r['realtime_success'] for r in chosen if r['method']==m) for m in ('ours','wang')},
        finished=time.time()))
    print('DONE '+str(out),flush=True)


if __name__=='__main__':
    main()
