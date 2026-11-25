"""Frozen first-attempt pair: existing common-passage deadline method."""
import argparse
from dataclasses import asdict
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
from . import adapter
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import CASES,manifest
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.probe import probe
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.audit import audit,truth
from convex_timevarying_window.comparisons.original_fixed_plane_ours.run import source_hashes as old_hashes
from convex_timevarying_window.wang_progress_mpc.experiment import mechanism_audit
from convex_timevarying_window.comparisons.checked_soft_racing.adjudicate import assess
from convex_timevarying_window.gs_mppi.direct_ctbr.compare_targets import body_dynamics

ROOT=Path(__file__).resolve().parents[3]
CASES = tuple(c for c in CASES if c.phase == 0.)
BASELINE=ROOT/'复现/polynomial_dynamic_racing_2023/px4_fixed_plane/results/original_layout_v2_20261007'


def save(path,data): path.write_text(json.dumps(data,indent=2)+'\n')
def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def source_hashes():
    hashes=old_hashes()
    for directory in (Path(__file__).parent,ROOT/'复现/polynomial_dynamic_racing_2023/px4_fixed_plane', ROOT/'convex_timevarying_window/wang_progress_mpc'):
        for path in directory.rglob('*'):
            if 'results' not in path.parts and path.suffix in ('.py','.md'):
                hashes[str(path.relative_to(ROOT))]=digest(path)
    path=ROOT/'convex_dynamic_seven_window_gazebo/build/libPeriodicGateMotion.so'
    hashes[str(path.relative_to(ROOT))]=digest(path)
    return hashes


def strict_truth(path,result):
    times,positions=truth(path,result['last_time_s'])
    velocity=np.gradient(positions,times,axis=0)
    events=result['events']; exits=result['body_exits']
    entry={e['gate']:e['time'] for e in events if e['inside'] and e['direction']=='forward'}
    exit_at={e['gate']:e['time'] for e in exits}
    exit_order=all(exit_at.get(i,np.inf)<entry.get(i+1,-np.inf) for i in range(6))
    no_reverse=not any(e['direction']=='backward' and e['time']>entry.get(e['gate'],np.inf) for e in events)
    goal_error=float(np.linalg.norm(positions[-1]-[-16,4,3.2])); goal_speed=float(np.linalg.norm(velocity[-1]))
    room=float(min(np.min(positions-[-28,-30,.65]),np.min([28,22,12]-positions)))
    returned=goal_error<.55 and goal_speed<1.
    realtime=bool(result.get('planner_cycle_ms') and result['planner_cycle_ms']['max']<100 and result['controller_deadline_passed'])
    passed=bool(result['physical_success'] and result['sampled_envelope_passed'] and room>=0 and exit_order and no_reverse and returned and realtime)
    return dict(main_passed=passed,truth_returned=returned,truth_goal_error_m=goal_error,truth_goal_speed_mps=goal_speed,
                workspace_center_margin_m=room,exit_before_next_entry=exit_order,no_reverse_after_entry=no_reverse,
                seventh_body_exit_s=exit_at.get(6),realtime_passed=realtime)


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);a=p.parse_args()
    out=a.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
    hashes=source_hashes(); rows=[]
    save(out/'protocol.json',dict(created=time.time(),cases=[manifest(c) for c in CASES],method='existing_common_passage_deadline_v1',
        source_sha256=hashes,baseline=str(BASELINE),protocol=Path(__file__).with_name('PROTOCOL.md').read_text(),
        baseline_result_sha256={c.name:digest(BASELINE/c.name/'result.json') for c in CASES}))
    for rel in hashes:
        target=out/'source_snapshot'/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/rel,target)
    for case in CASES:
        if source_hashes()!=hashes: raise RuntimeError('Frozen sources changed')
        path=out/case.name; print('START',case.name,flush=True)
        save(out/'status.json',dict(state='running',case=case.name,pid=os.getpid()))
        command=[sys.executable,'-u','-m','convex_timevarying_window.comparisons.original_fixed_plane_deadline.flight',
                 '--frequency',str(case.frequency),'--amplitude',str(case.amplitude),'--phase',str(case.phase),'--outdir',str(path)]
        failure=None
        with (out/f'{case.name}.log').open('w') as log:
            with subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True) as proc:
                try: code=proc.wait(timeout=300)
                except subprocess.TimeoutExpired:
                    failure='process_timeout';os.killpg(proc.pid,signal.SIGTERM)
                    try: proc.wait(timeout=5)
                    except subprocess.TimeoutExpired: os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                    code=proc.returncode
        row=dict(case=asdict(case),name=case.name,directory=str(path),returncode=code,process_error=failure)
        if (path/'result.json').exists():
            raw=json.loads((path/'result.json').read_text()); old=json.loads((BASELINE/case.name/'result.json').read_text())
            row.update({k:raw[k] for k in ('completed','failure','flight_time_s','last_time_s','collision_detected')})
            row['online_body_exit_count']=len(raw['body_exits'])
            row['same_world_bytes']=digest(path/'world.sdf')==digest(BASELINE/case.name/'world.sdf')
            row['same_scene_manifest']=json.loads((path/'scene.json').read_text())==json.loads((BASELINE/case.name/'scene.json').read_text())
            row['same_mpc_config']=raw['mpc_config']==old['mpc_config']
            row['same_px4_parameters']=raw['parameters']==old['parameters']
            try:
                result=audit(path);row.update(result);save(path/'audit.json',result)
                row.update(strict_truth(path,row)); row['mechanism_audit']=mechanism_audit(path)
                row['main_passed'] &= row['mechanism_audit']['passed']
                row['preparation_assessment']=assess(path)
                row['dynamics_diagnostic']=body_dynamics(path,raw['last_time_s'])
            except Exception as exc: row.update(main_passed=False,audit_error=repr(exc))
            try:
                scene=probe(case);save(path/'scene_probe.json',scene)
                row['scene_probe']={k:v for k,v in scene.items() if k!='comparisons'}
            except Exception as exc: row['scene_probe_error']=repr(exc)
        else: row.update(main_passed=False,failure='missing_result')
        rows.append(row);save(out/'summary.json',rows)
        print('RESULT',json.dumps({k:v for k,v in row.items() if k in ('name','failure','completed','flight_time_s',
            'online_body_exit_count','main_passed','audit_error','same_world_bytes')},ensure_ascii=False),flush=True)
    if source_hashes()!=hashes: raise RuntimeError('Frozen sources changed at end')
    save(out/'status.json',dict(state='complete',finished=time.time(),cases=len(rows)))
    print('COMPLETE',out,flush=True)


if __name__=='__main__':main()

