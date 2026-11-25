"""Correct only author numeric-bound encoding; retain unchanged flown methods."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from convex_timevarying_window.comparisons.single_gate_constant_velocity.run import sources,save,digest
from . import scenario
from .run import report,METHODS,MODULE

def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);a=p.parse_args();out=a.outdir.resolve();out.mkdir(exist_ok=False)
    old=scenario.HERE.parent/'results/pendulum_v1_20261105'
    state=json.loads((old/'status.json').read_text())
    if state['state'] not in ('complete','stopped'):raise RuntimeError('original coordinator still active')
    old_protocol=json.loads((old/'protocol.json').read_text());old_hash=old_protocol['source_sha256']
    changed=[rel for rel,h in old_hash.items() if digest(scenario.ROOT/rel)!=h]
    expected=str((scenario.HERE/'high_mpc_adapter.py').relative_to(scenario.ROOT))
    if changed!=[expected] and set(changed)!={expected}:raise RuntimeError('unexpected change to frozen methods:'+repr(changed))
    old_rows=json.loads((old/'summary.json').read_text());available={(r['angle_deg'],r['variant']):r for r in old_rows if r['variant'] in ('ours','center')}
    hashes=sources(scenario.ROOT)
    for rel in old_hash:
        if not rel.endswith('.py'):hashes[rel]=digest(scenario.ROOT/rel)
    save(out/'protocol.json',dict(angles_deg=[0.,60.,120.],methods=METHODS,repetitions=1,source_sha256=hashes,
        original_batch=str(old),only_changed_existing_source=expected,
        correction='project only <=1e-5 input-box floating-point solver residual; larger residual remains error',
        unchanged_free_and_center_reused=True,all_author_cases_refly=True,source_equivalence_verified=True,
        excluded_author_v1_reason='CTBR encoder rejected Ipopt default bound-relaxation residual 3.28e-7 N; not algorithm safety performance'))
    for rel in hashes:
        target=out/'source_snapshot'/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(scenario.ROOT/rel,target)
    rows=[];save(out/'summary.json',rows);report(out,rows)
    try:
        for i,angle in enumerate((0.,60.,120.)):
            for method in METHODS[i%3:]+METHODS[:i%3]:
                if any(digest(scenario.ROOT/rel)!=h for rel,h in hashes.items()):raise RuntimeError('frozen corrected source changed')
                if (angle,method) in available:
                    row=dict(available[(angle,method)]);path=Path(row['directory'])
                    if row['process_returncode'] or row['verdict']=='evidence_failure' or not row['gate_pose_audit']['passed']:raise RuntimeError('invalid reused evidence')
                    row['reuse_reason']='unchanged method; existing flown condition, no repeated attempt'
                    print('RETAIN',angle,method,row['verdict'],row.get('flight_time_s'),flush=True)
                else:
                    name=f'angle{angle:g}_{method}'
                    for attempt in range(2):
                        path=out/(name if not attempt else name+'_facility_retry')
                        save(out/'status.json',dict(state='running',pid=os.getpid(),completed=len(rows),total=9,angle_deg=angle,variant=method,directory=str(path)))
                        print('START',path.name,flush=True)
                        with (out/(path.name+'.log')).open('w') as stream:
                            completed=subprocess.run([sys.executable,'-u','-m',MODULE,'--angle',str(angle),'--method',method,'--outdir',str(path)],cwd=scenario.ROOT,stdout=stream,stderr=subprocess.STDOUT,timeout=600)
                        raw=json.loads((path/'result.json').read_text()) if (path/'result.json').exists() else None
                        unflown=raw is not None and raw['last_time_s']==0 and not raw['solves'] and 'arm failed' in str(raw['failure'])
                        if unflown and attempt==0:
                            save(path/'facility_retry_reason.json',dict(unflown=True,failure=raw['failure'],retained=True));continue
                        if not (path/'comparison_audit.json').exists():raise RuntimeError('missing flight audit:'+name)
                        row=json.loads((path/'comparison_audit.json').read_text());row.update(directory=str(path),process_returncode=completed.returncode)
                        if completed.returncode:row.update(passed=False,verdict='evidence_failure',flight_time_s=None)
                        print('RESULT',name,row['verdict'],row.get('flight_time_s'),flush=True)
                        break
                rows.append(row);save(out/'summary.json',rows);report(out,rows)
                if row['process_returncode']:raise RuntimeError('flight/evidence failure')
            group=[r for r in rows if r['angle_deg']==angle];paths=[Path(r['directory']) for r in group]
            if len({digest(path/'world.sdf') for path in paths})!=1 or len({digest(path/'pendulum_table.txt') for path in paths})!=1:raise RuntimeError('unmatched physical scene')
            raw=[json.loads((path/'result.json').read_text()) for path in paths];pair=[r for r in raw if r['comparison_variant'] in ('ours','center')]
            if [{k:v for k,v in r['config'].items() if k!='free_points'} for r in pair][0]!=[{k:v for k,v in r['config'].items() if k!='free_points'} for r in pair][1] or pair[0]['mpc_config']!=pair[1]['mpc_config']:raise RuntimeError('unmatched free/centre configuration')
            save(out/f'pair_angle{angle:g}.json',dict(same_physical_scene_and_motion=True,free_center_same_except_point=True,source_unchanged_except_author_roundoff=True))
        unchanged=all(digest(scenario.ROOT/rel)==h for rel,h in hashes.items())
        if not unchanged:raise RuntimeError('source changed at end')
        save(out/'status.json',dict(state='complete',completed=9,total=9,sources_unchanged=True,reused_unchanged_methods=len(available),author_all_cases_corrected=True))
    except Exception as exc:save(out/'status.json',dict(state='stopped',completed=len(rows),total=9,error=repr(exc)));raise

if __name__=='__main__':main()
