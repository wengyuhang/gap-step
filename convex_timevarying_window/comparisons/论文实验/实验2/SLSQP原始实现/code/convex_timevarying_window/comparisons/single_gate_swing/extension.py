"""Paired half-step frequency/amplitude staircases with no solve wall-clock cap."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import numpy as np
from pyulog import ULog
from .run import MODULE, report, pair_check
from .planner import Config
from convex_timevarying_window.comparisons.single_gate_constant_velocity.run import sources, save


def unflown_reason(path, raw, audit, log):
    if raw is None:
        if not (path/'telemetry_live.csv').exists() and any(s in log for s in ('missing initial telemetry','no heartbeat')):
            return 'No initial telemetry/heartbeat before task'
        return None
    if not raw['solves'] and raw['last_time_s']==0 and 'arm failed' in str(raw['failure']):
        return 'PX4 arm failed before task'
    if audit is None or audit['actual_distance_m']>=1e-6 or audit['truth_start_position_m'][2]>=0 or audit['accepted_plan_count'] or audit['crossings']:
        return None
    u=ULog(str(path/'flight.ulg'))
    d=u.get_dataset('vehicle_local_position_groundtruth').data
    pre=(d['timestamp']*1e-6>=14)&(d['timestamp']*1e-6<30)
    xyz=np.column_stack([d[k] for k in ('x','y','z')])
    a=u.get_dataset('actuator_outputs').data
    active=(a['timestamp']*1e-6>=14)&(a['timestamp']*1e-6<30)
    if pre.sum()>100 and np.ptp(xyz[pre],axis=0).max()<1e-4 and active.any() and np.mean(a['output[0]'][active])>300:
        return 'Gazebo body exactly stationary on ground throughout preparation despite nonzero PX4 actuator output'
    return None


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--outdir',type=Path,required=True)
    parser.add_argument('--start-frequency',type=float,default=4.5)
    parser.add_argument('--start-amplitude',type=float,default=4.5)
    parser.add_argument('--stop-on',choices=('either','ours'),default='either')
    args=parser.parse_args();root=Path(__file__).resolve().parents[3];out=args.outdir.resolve()
    out.mkdir(parents=True,exist_ok=False)
    assert Config().max_solve_ms is None
    hashes=sources(root);rows=[];cases=[];boundaries={}
    save(out/'protocol.json',dict(online_budget_ms=None,offline_diagnostic_budget_ms=None,
        max_task_s=40,frequency_staircase=dict(amplitude_m=4,start_hz=args.start_frequency,step_hz=.5),
        amplitude_staircase=dict(frequency_hz=4,start_m=args.start_amplitude,step_m=.5),
        stop_rule='Complete paired condition; stop at first non-success of '+args.stop_on,
        repetitions=1,phase=0,unflown_facility_retries=1,source_sha256=hashes,
        implementation_protocol=Path(__file__).with_name('README.md').read_text()))
    for rel in hashes:
        target=out/'source_snapshot'/rel;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(root/rel,target)
    save(out/'summary.json',rows)
    try:
        for route in ('frequency','amplitude'):
            step=0
            while True:
                frequency,amplitude=(args.start_frequency+.5*step,4.) if route=='frequency' else (4.,args.start_amplitude+.5*step)
                cases.append((frequency,amplitude))
                for variant in (('ours','center') if len(cases)%2 else ('center','ours')):
                    assert sources(root)==hashes, 'frozen_source_changed'
                    name=f'a{amplitude:g}_f{frequency:g}_{variant}'
                    for attempt in range(2):
                        path=out/(name if not attempt else name+'_facility_retry')
                        save(out/'status.json',dict(state='running',pid=os.getpid(),completed=len(rows),route=route,
                            frequency_hz=frequency,amplitude_m=amplitude,variant=variant,attempt=attempt,
                            directory=str(path),boundaries=boundaries))
                        print('START',path.name,flush=True)
                        command=[sys.executable,'-u','-m',MODULE,'--frequency',str(frequency),
                            '--amplitude',str(amplitude),'--method',variant,'--outdir',str(path)]
                        with (out/(path.name+'.log')).open('w') as stream:
                            with subprocess.Popen(command,cwd=root,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True) as proc:
                                code=proc.wait()
                        raw=json.loads((path/'result.json').read_text()) if (path/'result.json').exists() else None
                        audit=json.loads((path/'comparison_audit.json').read_text()) if (path/'comparison_audit.json').exists() else None
                        reason=unflown_reason(path,raw,audit,(out/(path.name+'.log')).read_text())
                        if reason:
                            save(path/'facility_retry_reason.json',dict(unflown=True,reason=reason,attempt=attempt,original_audit=audit))
                            if not attempt:
                                print('UNFLOWN_FACILITY_RETRY',path.name,reason,flush=True);continue
                            raise RuntimeError('repeated_unflown_facility_failure:'+path.name)
                        if code or audit is None:raise RuntimeError('infrastructure_or_audit_failure:'+path.name)
                        audit.update(directory=str(path),process_returncode=code,planning_requests=len(raw['solves']),
                            offline_found=raw['planner_process']['offline_diagnostic']['found'],route=route)
                        break
                    rows.append(audit);save(out/'summary.json',rows);report(out,rows,cases)
                    print('RESULT',name,audit['verdict'],audit.get('flight_time_s'),flush=True)
                pair_check(out,rows,frequency,amplitude)
                if any(not r['passed'] and (args.stop_on=='either' or r['variant']=='ours') for r in rows[-2:]):
                    boundaries[route]=dict(frequency_hz=frequency,amplitude_m=amplitude,
                        outcomes={r['variant']:r['verdict'] for r in rows[-2:]},
                        meaning='first observed paired failure, not monotonic infeasibility threshold')
                    save(out/'boundaries.json',boundaries)
                    break
                step+=1
        assert sources(root)==hashes,'frozen_source_changed_at_end'
        save(out/'status.json',dict(state='complete',pid=os.getpid(),completed=len(rows),sources_unchanged=True,boundaries=boundaries))
    except Exception as exc:
        save(out/'status.json',dict(state='stopped',pid=os.getpid(),completed=len(rows),error=repr(exc),boundaries=boundaries))
        raise


if __name__=='__main__':main()
