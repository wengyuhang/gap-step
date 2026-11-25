"""Frozen matched-condition paper baseline runs; all negative results retained.

This driver does not tune controllers, change courses after failure, or rank
unvalidated policy transfers as successful paper reproductions.
"""
import argparse
import contextlib
import hashlib
import io
import json
import math
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time
import numpy as np
from convex_timevarying_window.receding_gate_racing.analyze import analyze
from convex_timevarying_window.receding_gate_racing.localization_audit import audit


def summarize(path):
    r=json.loads((path/'result.json').read_text())
    row={'method':r.get('method','proposed'),'run':str(path.resolve()),'completed':r['completed'],
         'failure':r['failure'],'flight_time_s':r['flight_time_s'],'last_time_s':r['last_time_s'],
         'collision':r['collision_detected'],'gates':len(r['crossings']),
         'motion_rate':r['motion_rate'],'translation_scale':r.get('translation_scale',1),
         'rotation_scale':r.get('rotation_scale',1),'phase':r['phase']}
    ms=[s['cycle_ms'] for s in r['solves']]
    row['cycle_ms']={'max':max(ms),'p99':float(np.percentile(ms,99)),'mean':float(np.mean(ms))} if ms else None
    row['physical_success']=False;row['realtime_success']=False
    try:
        with contextlib.redirect_stdout(io.StringIO()) as captured:
            checked=analyze(path);truth=audit(path)
        (path/'analysis.log').write_text(captured.getvalue())
        clearance=min(g['minimum_sampled_sphere_sleeve_clearance_m'] for g in truth['groundtruth_sampled_sphere_sleeve_clearance'])
        room=bool(np.all(np.array(truth['world_body_position_min'])>=np.array(r['config']['workspace_lower'])-.01)
                  and np.all(np.array(truth['world_body_position_max'])<=np.array(r['config']['workspace_upper'])+.01))
        physical=bool(checked['completed'] and checked['contact_free'] and checked['ordered_passages_passed'] and room and clearance>=0)
        data=np.atleast_1d(np.genfromtxt(path/'telemetry.csv',delimiter=',',names=True))
        speed=np.sqrt(data['vx']**2+data['vy']**2+data['vz']**2);duration=data['t'][-1]-data['t'][0]
        row.update(physical_success=physical,realtime_success=physical and checked['deadline_passed'],
                   min_clearance_m=clearance,room_passed=room,accepted_replans=checked['accepted_replans'],
                   total_replans=checked['total_replans'],mean_speed_mps=float(np.trapz(speed,data['t'])/duration),
                   max_speed_mps=float(speed.max()),slow_time_s=float(np.sum(np.diff(data['t'])*(speed[:-1]<1))))
    except Exception as exc:row['audit_error']=repr(exc)
    return row


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True)
    p.add_argument('--method',choices=['polynomial','proposed','rl'],required=True)
    p.add_argument('--stage',choices=['static','development','pilot','matrix','center_motion'],default='matrix')
    p.add_argument('--checkpoint',type=Path);p.add_argument('--max-flight-time',type=float,default=60.)
    a=p.parse_args()
    if a.method=='rl' and a.checkpoint is None:p.error('RL requires a trained checkpoint')
    a.outdir.mkdir(parents=True,exist_ok=False)
    if a.stage=='static':conditions=[(1,0,0,.4)]
    elif a.stage=='development':conditions=[(1,1,1,.4),(10,4,4,.4)]
    elif a.stage=='center_motion':
        if a.method!='polynomial':p.error('center_motion currently matches the 2023 polynomial baseline only')
        conditions=[(rate,translation,1,phase)
                    for rate in (1,3,6,10) for translation in (1,2,4)
                    for phase in (0.,2*math.pi/3)]
    else:
        base=[(r,s,s) for r in (1,10) for s in (1,2,4)]+[(10,4,1),(10,1,4)]
        if a.stage=='pilot':base=[(1,1,1),(10,1,1),(10,4,4)]
        conditions=[(*b,phase) for b in base for phase in (0.,2*math.pi/3)]
    roots=[Path('convex_timevarying_window/receding_gate_racing'),Path('convex_timevarying_window/continuation_racing'),
           Path('复现/agile_generalist_rl_2026' if a.method=='rl' else '复现/polynomial_dynamic_racing_2023'),
           Path(__file__).resolve().parent.relative_to(Path.cwd())]
    sources=[f for root in roots for f in root.glob('*.py')]
    hashes={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources}
    snapshot=a.outdir/'source_snapshot'
    for f in sources:
        dest=snapshot/f;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(f,dest)
    checkpoint_hash=hashlib.sha256(a.checkpoint.read_bytes()).hexdigest() if a.checkpoint else None
    protocol={'method':a.method,'stage':a.stage,'conditions':conditions,'source_sha256':hashes,
              'checkpoint':str(a.checkpoint) if a.checkpoint else None,'checkpoint_sha256':checkpoint_hash,
              'python':platform.python_version(),'numpy':np.__version__,'max_flight_time_s':a.max_flight_time,
              'note':'Small fixed-condition scan, not population success probability. RL transfer validity must be reported separately.'}
    (a.outdir/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    rows=[]
    for index,(rate,translation,rotation,phase) in enumerate(conditions):
        for attempt in ((1,2) if a.stage=='center_motion' else (1,)):
            for f in sources:
                if hashes[str(f)]!=hashlib.sha256(f.read_bytes()).hexdigest():raise RuntimeError(f'Frozen source changed: {f}')
            if a.checkpoint and checkpoint_hash!=hashlib.sha256(a.checkpoint.read_bytes()).hexdigest():raise RuntimeError('Frozen checkpoint changed')
            suffix=f'_p{0 if phase==0. else 1}_attempt{attempt}' if a.stage=='center_motion' else ''
            path=a.outdir/f'case_{index:02d}_r{rate}_t{translation}_a{rotation}{suffix}'
            common=['--outdir',str(path),'--motion-rate',str(rate),'--translation-scale',str(translation),
                    '--rotation-scale',str(rotation),'--phase',str(phase),'--max-flight-time',str(a.max_flight_time)]
            if a.method=='rl':
                cmd=[sys.executable,'-u','-m','复现.agile_generalist_rl_2026.px4','--checkpoint',str(a.checkpoint),*common]
            else:
                cmd=[sys.executable,'-u','-m','convex_timevarying_window.receding_gate_racing.px4_experiment',
                     '--launch','--tracking-preview','.05',('--paper-polynomial' if a.method=='polynomial' else '--checked'),*common]
            print('START',index,attempt,cmd,flush=True);tick=time.perf_counter()
            console_path=a.outdir/(f'case_{index:02d}{suffix}_console.log')
            with console_path.open('w') as console:
                try:
                    process=subprocess.run(cmd,stdout=console,stderr=subprocess.STDOUT,timeout=240)
                    returncode=process.returncode
                except subprocess.TimeoutExpired:returncode=None
            if (path/'result.json').exists():
                result=json.loads((path/'result.json').read_text())
                infrastructure_failure=result.get('failure')=="RuntimeError('arm failed')"
                row=summarize(path) if not infrastructure_failure else {
                    'method':a.method,'run':str(path),'physical_success':False,
                    'realtime_success':False,'failure':result['failure']}
            else:
                infrastructure_failure=False
                row={'method':a.method,'run':str(path),'physical_success':False,
                     'realtime_success':False,'failure':'missing_result'}
            row.update(index=index,attempt=attempt,phase_id=0 if phase==0. else 1,
                       infrastructure_failure=infrastructure_failure,returncode=returncode,
                       wall_s=time.perf_counter()-tick)
            rows.append(row)
            (a.outdir/'summary.json').write_text(json.dumps(rows,indent=2)+'\n')
            print('RESULT',json.dumps(row),flush=True)
            if not infrastructure_failure:break
    print('DONE',a.outdir,flush=True)


if __name__=='__main__':main()
