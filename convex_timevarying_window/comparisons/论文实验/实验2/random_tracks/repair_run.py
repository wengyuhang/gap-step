"""Small paired eight-initial validation, without baseline tuning or flights."""
import os,sys,json,time,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent
OUT=HERE/'results/repair_low_eight_v2_20261113'
def save(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2)+'\n')
def main():
 OUT.mkdir(exist_ok=False);(OUT/'jobs').mkdir();(OUT/'logs').mkdir();active=[];began=time.perf_counter();rows=[]
 for method in ['ours','center']:
  for i in range(8):
   job=dict(id=f'{method}_{i}',method=method,seed=i,level='2026111301_low');save(OUT/'jobs'/f"{job['id']}.json",job);f=(OUT/'logs'/f"{job['id']}.log").open('w');p=subprocess.Popen([sys.executable,'-u',str(HERE/'repair_worker.py'),'--outdir',str(OUT),'--job',job['id']],stdout=f,stderr=subprocess.STDOUT,env=os.environ.copy());active.append(dict(job,process=p,log=f))
 save(OUT/'protocol.json',dict(level='2026111301_low',methods=['ours','center'],seeds=list(range(8)),initials='repair_initials_v3_20261113',original_objective_and_limits=True,concurrency=16,explicit_directions='incoming/outgoing unit-chord bisector projected on fixed gate normals; common across starts',cross_method_warm_start=False,optimization_language='Python/CasADi + original C++ penalty kernels',task_constraint_repair_added=False))
 while active:
  for item in list(active):
   code=item['process'].poll()
   if code is None:continue
   item['log'].close();active.remove(item);p=OUT/'plans'/item['id']/'result.json';raw=json.loads(p.read_text()) if p.exists() else {};r=dict(job=item['id'],method=item['method'],seed=item['seed'],exit_code=code,found=raw.get('found',False),time_s=raw.get('nominal_time_s'),solve_s=raw.get('solve_s'),finite_statuses=[x['status'] for x in raw.get('optimizer_calls',[])],frame_clear=raw.get('independent_frame_clear',False),task_passed=raw.get('independent_task_passed',False));rows.append(r);save(OUT/'summary.json',rows);print(r,flush=True)
  save(OUT/'status.json',dict(state='running',complete=len(rows),elapsed_s=time.perf_counter()-began,active=[dict(job=i['id'],pid=i['process'].pid) for i in active]));time.sleep(1)
 save(OUT/'status.json',dict(state='complete',complete=len(rows),elapsed_s=time.perf_counter()-began,active=[]))
if __name__=='__main__':main()
