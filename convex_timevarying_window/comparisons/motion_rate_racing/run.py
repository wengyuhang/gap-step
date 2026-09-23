"""Frozen frequency/phase PX4 batch; preserves every attempt and failure."""
import argparse
import contextlib
import hashlib
import io
import json
import math
from pathlib import Path
import subprocess
import sys
import time

from convex_timevarying_window.receding_gate_racing.analyze import analyze
from convex_timevarying_window.receding_gate_racing.localization_audit import audit


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--outdir',type=Path,required=True)
    p.add_argument('--method',choices=['ours','gp'],default='ours')
    p.add_argument('--local-sweep',action='store_true')
    p.add_argument('--legacy-crossing',action='store_true')
    p.add_argument('--rates',nargs='+',type=float,default=[1,3,6,10])
    p.add_argument('--phases',nargs='+',type=float,default=[0,2*math.pi/3,4*math.pi/3])
    args=p.parse_args(); args.outdir.mkdir(parents=True,exist_ok=False)
    if args.local_sweep and args.legacy_crossing:p.error('crossing modes are mutually exclusive')
    args.local_sweep=not args.legacy_crossing
    planner=(Path('convex_timevarying_window/receding_gate_racing/planner.py') if args.method=='ours' else Path(__file__).parent/'gp_rollout.cpp')
    frozen=hashlib.sha256(planner.read_bytes()).hexdigest()
    (args.outdir/'protocol.json').write_text(json.dumps(dict(rates=args.rates,phases=args.phases,
        planner_sha256=frozen,method=args.method,local_sweep=args.local_sweep,max_flight_time_s=60),indent=2))
    rows=[]
    for rate in args.rates:
        for phase_id,phase in enumerate(args.phases):
            for attempt in range(2):
                assert hashlib.sha256(planner.read_bytes()).hexdigest()==frozen
                path=args.outdir/f'{args.method}_rate{rate:g}_phase{phase_id}_attempt{attempt+1}'
                print('START',path.name,flush=True)
                start=time.monotonic()
                cmd=[sys.executable,'-u','-m','convex_timevarying_window.receding_gate_racing.px4_experiment',
                     '--launch','--motion-rate',str(rate),'--phase',str(phase),'--outdir',str(path)]
                if args.method=='gp':
                    cmd=[sys.executable,'-u','-m','convex_timevarying_window.comparisons.motion_rate_racing.gp_px4',
                         '--motor-telemetry','--motion-rate',str(rate),'--phase',str(phase),'--seed',str(phase_id),'--outdir',str(path)]
                elif args.local_sweep:cmd.append('--local-sweep')
                else:cmd.append('--legacy-crossing')
                proc=subprocess.run(cmd,text=True,capture_output=True,timeout=240)
                path.mkdir(exist_ok=True)
                (path/'console.log').write_text(proc.stdout+'\n'+proc.stderr)
                row=dict(method=args.method,rate=rate,phase_id=phase_id,phase=phase,attempt=attempt+1,
                         run=str(path.resolve()),wall_s=time.monotonic()-start)
                if not (path/'result.json').exists():
                    row.update(infrastructure_failure=True,error=proc.stderr[-1500:])
                else:
                    result=json.loads((path/'result.json').read_text())
                    row.update(infrastructure_failure=result.get('failure')=="RuntimeError('arm failed')")
                    if not row['infrastructure_failure']:
                        try:
                            with contextlib.redirect_stdout(io.StringIO()) as output:
                                a=analyze(path); b=audit(path)
                            (path/'analysis.log').write_text(output.getvalue())
                            geometry=b.get('room_clearance_passed',False) and bool(b.get('groundtruth_sampled_sphere_sleeve_clearance')) and all(
                                g['minimum_sampled_sphere_sleeve_clearance_m']>=0 for g in b['groundtruth_sampled_sphere_sleeve_clearance'])
                            physical=(a['completed'] and a['contact_free'] and a['ordered_passages_passed']
                                      and a['goal_error_m']<.55 and a['goal_speed_mps']<1 and geometry)
                            row.update(physical_success=physical,realtime_success=physical and a['deadline_passed'],
                                flight_time_s=result['flight_time_s'],cycle_ms=a['cycle_ms'],
                                gates=sum(g is not None for g in a['ordered']),collision=result['collision_detected'],failure=result['failure'],
                                min_clearance_m=min(g['minimum_sampled_sphere_sleeve_clearance_m'] for g in b['groundtruth_sampled_sphere_sleeve_clearance']))
                        except Exception as exc:
                            row.update(physical_success=False,realtime_success=False,analysis_error=repr(exc),
                                       failure=result['failure'],gates=len(result['crossings']))
                rows.append(row)
                (args.outdir/'summary.json').write_text(json.dumps(rows,indent=2)+'\n')
                print('RESULT',json.dumps(row),flush=True)
                if not row['infrastructure_failure']: break
    print('DONE',args.outdir,flush=True)


if __name__=='__main__': main()
