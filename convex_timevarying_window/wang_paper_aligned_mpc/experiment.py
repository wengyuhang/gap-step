"""Frozen paired Gazebo/PX4 flights, including an initialization ablation."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import numpy as np
from convex_timevarying_window.comparisons.fixed_plane_wang.run import source_hashes as old_hashes
from convex_timevarying_window.comparisons.fixed_plane_wang.audit import inspect_run
from convex_timevarying_window.comparisons.checked_soft_racing.run import ROOT, save

CASES=[(.8,1.6,.4),(.6,1.2,0.)]
METHODS=['wang_warm','free_warm','wang','free_cold']


def source_hashes():
    result=old_hashes()
    for directory in [Path(__file__).parent, ROOT/'convex_timevarying_window/wang_center_seed_mpc',
                      ROOT/'复现/polynomial_dynamic_racing_2023/paper_aligned']:
        for path in sorted(directory.rglob('*')):
            if 'results' not in path.parts and path.suffix in ('.py','.md'):
                result[str(path.relative_to(ROOT))]=hashlib.sha256(path.read_bytes()).hexdigest()
    return result


def mechanism_audit(path):
    raw=json.loads((path/'result.json').read_text()); failures=[]; warm=0; global_seed=0
    for s in raw['solves']:
        if 'initial_variables' not in s: continue
        if s['pieces_per_leg']!=4 or len(s['durations'])!=8 or s['gate_count']!=1:
            failures.append('N_M_gate_count')
        if len(s['variables'])!=28: failures.append('decision_dimension')
        if not np.all(np.array(s['initial_variables'][26:])==0): failures.append('noncenter_seed')
        if not raw['config']['free_points'] and np.max(np.abs(s['variables'][26:]))>1e-9:
            failures.append('baseline_not_center')
        if s['terminal_goal']!=(s['candidate_first_gate']==6): failures.append('terminal_goal_flag')
        for row in s['seed_provenance']:
            warm+=row['state_source']=='admitted';global_seed+=row['state_source']=='global'
    if raw['N']!=2 or raw['M']!=4 or raw['config']['horizon']!=1: failures.append('configuration')
    if not raw['config']['state_warm_start'] and warm: failures.append('unrequested_warm_start')
    result=dict(passed=not failures,failures=failures,warm_segments=warm,global_seed_segments=global_seed)
    save(path/'mechanism_audit.json',result);return result


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);a=p.parse_args()
    out=a.outdir.resolve();out.mkdir(parents=True,exist_ok=False);hashes=source_hashes()
    save(out/'protocol.json',dict(cases=CASES,methods=METHODS,source_sha256=hashes,created=time.time(),
        N=2,M=4,first_order=METHODS,second_order=list(reversed(METHODS)),race_timeout_s=60,
        process_timeout_s=240,retry='Once only after independently confirmed infrastructure fault',
        reference='ALIGNMENT.md; all three are independent x500-adapted implementations',
        old_matrix='Paused; no results replaced or reused'))
    for rel in hashes:
        dest=out/'source_snapshot'/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/rel,dest)
    rows=[]
    for i,case in enumerate(CASES):
        for method in (METHODS if i==0 else list(reversed(METHODS))):
            for attempt in (1,2):
                if source_hashes()!=hashes: raise RuntimeError('Sources changed during frozen flights')
                path=out/f'case{i:02d}_{method}_attempt{attempt}'
                save(out/'status.json',dict(state='running',case=i,method=method,attempt=attempt,pid=os.getpid(),run=str(path)))
                print('START '+path.name,flush=True);began=time.monotonic();timeout=False
                command=[sys.executable,'-u','-m','convex_timevarying_window.wang_paper_aligned_mpc.flight',
                    '--method',method,'--frequency',str(case[0]),'--amplitude',str(case[1]),
                    '--phase',str(case[2]),'--outdir',str(path)]
                with subprocess.Popen(command,cwd=ROOT,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                                      text=True,start_new_session=True) as proc:
                    try: output,_=proc.communicate(timeout=240)
                    except subprocess.TimeoutExpired:
                        timeout=True;os.killpg(proc.pid,signal.SIGTERM)
                        try: output,_=proc.communicate(timeout=5)
                        except subprocess.TimeoutExpired:
                            os.killpg(proc.pid,signal.SIGKILL);output,_=proc.communicate()
                    code=proc.returncode
                path.mkdir(exist_ok=True);(path/'console.log').write_text(output)
                save(path/'batch_source_reference.json',dict(protocol=str(out/'protocol.json'),source_sha256=hashes))
                row=dict(method=method,case=i,condition_id=case,attempt=attempt,run=str(path),
                         returncode=code,wall_s=time.monotonic()-began)
                if (path/'result.json').exists():
                    row.update(inspect_run(path,'ours'))  # All variants use identical CTBR MPC.
                    row['mechanism_audit']=mechanism_audit(path)
                    row['realtime_success'] &= row['mechanism_audit']['passed']
                else:
                    started=(path/'solves_live.jsonl').exists()
                    infra=not started and any(s in output for s in ('PX4 startup failed','no heartbeat','missing initial telemetry'))
                    row.update(physical_success=False,realtime_success=False,infrastructure_failure=infra,
                               failure='process_timeout' if timeout else 'missing_result',flight_time_s=None)
                rows.append(row);save(out/'summary.json',rows)
                print('RESULT '+json.dumps(row,ensure_ascii=False),flush=True)
                if not row['infrastructure_failure']:break
    if source_hashes()!=hashes:raise RuntimeError('Sources changed')
    save(out/'status.json',dict(state='complete',attempts=len(rows),finished=time.time()))
    print('COMPLETE',flush=True)


if __name__=='__main__':main()
