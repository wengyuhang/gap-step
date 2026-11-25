"""Backtracked periodic-time globalization of one independent trajectory solve."""
from pathlib import Path
import os,sys,time,json,subprocess,argparse
from bootstrap import load,HERE
scene,higher,common,_,_=load()
from cycle_run import advance
from repair import explicit_signs
import numpy as np

def save(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n')
def main():
 p=argparse.ArgumentParser();p.add_argument('--seed',type=int,required=True);p.add_argument('--outdir',type=Path,required=True);p.add_argument('--source',type=Path);p.add_argument('--max-attempts',type=int,default=20);p.add_argument('--exact',action='store_true');a=p.parse_args();a.outdir.mkdir(parents=True,exist_ok=False);level='2026111301_high';tr,_=scene.build_case(scene.Case(level));freq=[w.motion.frequency for w in tr.windows]
 initial=common.read(HERE/'results/pv_transit_initials_v1_20261113'/f'{level}.json');incumbent=common.read(a.source) if a.source else None;x=np.array(incumbent['variables'] if incumbent else initial['seeds'][a.seed]['variables']);best_cost=incumbent['objective'] if incumbent else float(np.exp(x[:8]).sum());records=[];factor=.95;began=time.perf_counter()
 save(a.outdir/'protocol.json',dict(seed=a.seed,source=str(a.source) if a.source else None,independent_original_initial=a.source is None,maximum_attempts=a.max_attempts,initial_hessian='exact' if a.exact else 'limited-memory',initial_compression=.95,backtracking='halve compression after unsuccessful attempt, reset after improvement',worker='sampled limited-memory Hessian, physical PVAJ',full_safety_required=True,all_attempts_preserved=True))
 for attempt in range(a.max_attempts):
  trial=x.copy() if incumbent is None and attempt==0 else advance(x,8,freq,factor);old=np.cumsum(np.exp(x[:8]))[:-1];new=np.cumsum(np.exp(trial[:8]))[:-1]
  for w,t0,t1 in zip(tr.windows,old,new):np.testing.assert_allclose(w.state_at(t0)[0],w.state_at(t1)[0],atol=1e-8)
  d=a.outdir/f'attempt_{attempt:02d}';(d/'jobs').mkdir(parents=True);job=dict(id='ours_0',method='ours',seed=0,level=level);save(d/'jobs/ours_0.json',job);save(d/'initial.json',dict(explicit_signs=list(map(float,explicit_signs(tr)[0])),seeds=[dict(variables=trial.tolist())]))
  with (d/'worker.log').open('w') as log:r=subprocess.run([sys.executable,'-u',str(HERE/'sampled_cycle_worker.py'),'--outdir',str(d),'--job','ours_0','--initial',str(d/'initial.json')]+(['--exact'] if a.exact else []),stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy())
  f=d/'plans/ours_0/result.json';raw=common.read(f) if f.exists() else {};qualified=bool(r.returncode==0 and raw.get('found') and raw.get('independent_frame_clear') and raw.get('independent_task_passed'));improved=qualified and (incumbent is None or raw['objective']<best_cost-1e-5)
  row=dict(attempt=attempt,factor=factor,qualified=qualified,improved=improved,nominal_time_s=raw.get('nominal_time_s'),objective=raw.get('objective'),solve_s=raw.get('solve_s'),exit_code=r.returncode,finite_calls=len(raw.get('optimizer_calls',[])),finite_statuses=[c['status'] for c in raw.get('optimizer_calls',[])]);records.append(row);save(a.outdir/'attempts.json',records);print(a.seed,row,flush=True)
  if improved:incumbent=raw;x=np.array(raw['variables']);best_cost=raw['objective'];factor=.95;save(a.outdir/'best_result.json',raw)
  else:factor=(1+factor)/2
  save(a.outdir/'status.json',dict(state='running',attempts=len(records),best_time_s=incumbent.get('nominal_time_s') if incumbent else None,elapsed_s=time.perf_counter()-began))
  if factor>.995:break
 save(a.outdir/'status.json',dict(state='complete',attempts=len(records),best_time_s=incumbent.get('nominal_time_s') if incumbent else None,elapsed_s=time.perf_counter()-began,qualified=incumbent is not None))
if __name__=='__main__':main()
