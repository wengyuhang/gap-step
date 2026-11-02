"""Freeze and fly all eight already computed approach plans exactly once."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from .run import report
from convex_timevarying_window.comparisons.single_gate_constant_velocity.run import sources,save,digest
from convex_timevarying_window.comparisons.single_gate_swing.extension import unflown_reason


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);a=p.parse_args()
    out=a.outdir.resolve();root=Path(__file__).resolve().parents[3]
    if (out/'protocol.json').exists():raise FileExistsError('Do not resume or overwrite a scored batch')
    hashes=sources(root);plans={};rows=[]
    for v in (5,10,15,20):
        for variant in ('ours','center'):
            path=out/f'v{v}_{variant}_plan/plan.json'
            record=json.loads(path.read_text())
            if not record['found']:raise RuntimeError('Missing admissible offline plan:'+str(path))
            if record['made_at']+record['durations'][0]>=record['case'].get('offset',30.)/v:
                raise RuntimeError('Nominal crossing is not in the approaching phase:'+str(path))
            plans[str(path)]=digest(path)
    save(out/'protocol.json',dict(mode='offline precomputed, no online solve',speeds_mps=[5,10,15,20],
        directions=['approaching at crossing; checked before execution'],nominal_head='identical start P and zero VAJ',made_at_s=0,
        source_sha256=hashes,plan_sha256=plans,repetitions=1,unflown_facility_retries=1,
        max_task_s=40,planner_wall_limit_ms=None,online_replanning=False,
        physics_step_s=.001,protocol=Path(__file__).with_name('OFFLINE_PROTOCOL.md').read_text()))
    for rel in hashes:
        dest=out/'source_snapshot'/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(root/rel,dest)
    save(out/'summary.json',rows)
    try:
        for i,v in enumerate((5,10,15,20)):
            for variant in (('ours','center') if i%2==0 else ('center','ours')):
                assert sources(root)==hashes,'frozen_source_changed'
                assert all(digest(Path(p))==h for p,h in plans.items()),'frozen_plan_changed'
                name=f'v{v}_{variant}_flight';src=out/f'v{v}_{variant}_plan/plan.json'
                for attempt in range(2):
                    path=out/(name if not attempt else name+'_facility_retry')
                    save(out/'status.json',dict(state='running',pid=os.getpid(),completed=len(rows),total=8,
                        speed_mps=v,variant=variant,attempt=attempt,directory=str(path),mode='offline_frozen'))
                    print('START',path.name,flush=True)
                    with (out/(path.name+'.log')).open('w') as stream:
                        with subprocess.Popen([sys.executable,'-u','-m',
                            'convex_timevarying_window.comparisons.single_gate_linear_approach.offline_flight',
                            '--plan',str(src),'--outdir',str(path)],cwd=root,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True) as proc:
                            code=proc.wait()
                    raw=json.loads((path/'result.json').read_text()) if (path/'result.json').exists() else None
                    audit=json.loads((path/'comparison_audit.json').read_text()) if (path/'comparison_audit.json').exists() else None
                    reason=unflown_reason(path,raw,audit,(out/(path.name+'.log')).read_text())
                    if reason:
                        save(path/'facility_retry_reason.json',dict(reason=reason,unflown=True,attempt=attempt,original_audit=audit))
                        if not attempt:print('UNFLOWN_FACILITY_RETRY',path.name,reason,flush=True);continue
                        raise RuntimeError('repeated_facility_failure:'+path.name)
                    if code or audit is None:raise RuntimeError('infrastructure_or_audit_failure:'+path.name)
                    audit.update(directory=str(path),process_returncode=code,planning_requests=0,offline_found=True)
                    break
                rows.append(audit);save(out/'summary.json',rows);report(out,rows)
                print('RESULT',name,audit['verdict'],audit.get('flight_time_s'),flush=True)
            pair=rows[-2:];records=[json.loads((Path(r['directory'])/'result.json').read_text()) for r in pair]
            configs=[dict(r['config']) for r in records]
            for cfg in configs:cfg.pop('free_points')
            same=(configs[0]==configs[1] and records[0]['mpc_config']==records[1]['mpc_config']
                and records[0]['parameters']==records[1]['parameters']
                and digest(Path(pair[0]['directory'])/'world.sdf')==digest(Path(pair[1]['directory'])/'world.sdf'))
            planconfigs=[dict(r['offline_plan']['config']) for r in records]
            for cfg in planconfigs:cfg.pop('free_points')
            same &= planconfigs[0]==planconfigs[1]
            save(out/f'pair_v{v}.json',dict(equal_except_crossing_point=same,online_planning_requests=0))
            assert same,'pair_configuration_mismatch'
        assert sources(root)==hashes,'frozen_source_changed_at_end'
        save(out/'status.json',dict(state='complete',pid=os.getpid(),completed=len(rows),total=8,sources_unchanged=True))
    except Exception as exc:
        save(out/'status.json',dict(state='stopped',pid=os.getpid(),completed=len(rows),total=8,error=repr(exc)))
        raise


if __name__=='__main__':main()
