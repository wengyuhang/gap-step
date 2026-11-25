"""Validate the same repaired model on medium/high, 8 starts per method."""
import sys,os,time,json,subprocess
from pathlib import Path
from collections import deque
HERE=Path(__file__).resolve().parent
OUT=HERE/'results/repair_medium_high_eight_v1_20261113'
def save(p,obj):p.write_text(json.dumps(obj,ensure_ascii=False,indent=2)+'\n')
def main():
 OUT.mkdir(exist_ok=False);(OUT/'jobs').mkdir();(OUT/'logs').mkdir();q=deque();active=[];rows=[];began=time.perf_counter()
 for level in ['2026111301_medium','2026111301_high']:
  for method in ['ours','center']:
   for i in range(8):
    job=dict(id=f'{level}_{method}_{i}',method=method,seed=i,level=level);save(OUT/'jobs'/f"{job['id']}.json",job);q.append(job)
 save(OUT/'protocol.json',dict(levels=['2026111301_medium','2026111301_high'],methods=['ours','center'],seeds=list(range(8)),concurrency=16,initials='repair_initials_v3_20261113',direction='incoming/outgoing unit-chord bisector projected on fixed normal',original_objective_and_limits=True,cross_method_warm_start=False,flights=False))
 while q or active:
  for item in list(active):
   code=item['p'].poll()
   if code is None:continue
   item['log'].close();active.remove(item);f=OUT/'plans'/item['id']/'result.json';r=json.loads(f.read_text()) if f.exists() else {};row=dict(job=item['id'],level=item['level'],method=item['method'],seed=item['seed'],exit_code=code,found=r.get('found',False),time_s=r.get('nominal_time_s'),solve_s=r.get('solve_s'),finite_statuses=[c['status'] for c in r.get('optimizer_calls',[])],frame_clear=r.get('independent_frame_clear',False),task_passed=r.get('independent_task_passed',False));rows.append(row);save(OUT/'summary.json',rows);print(row,flush=True)
  while q and len(active)<16:
   job=q.popleft();log=(OUT/'logs'/f"{job['id']}.log").open('w');p=subprocess.Popen([sys.executable,'-u',str(HERE/'repair_worker.py'),'--outdir',str(OUT),'--job',job['id']],stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy());active.append(dict(job,p=p,log=log))
  save(OUT/'status.json',dict(state='running',complete=len(rows),queued=len(q),active=[dict(job=x['id'],pid=x['p'].pid) for x in active],elapsed_s=time.perf_counter()-began));time.sleep(1)
 save(OUT/'status.json',dict(state='complete',complete=len(rows),queued=0,active=[],elapsed_s=time.perf_counter()-began))
if __name__=='__main__':main()
