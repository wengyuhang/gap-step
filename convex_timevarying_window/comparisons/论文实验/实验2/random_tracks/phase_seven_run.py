from pathlib import Path
import os,sys,time,json,subprocess
HERE=Path(__file__).resolve().parent
OUT=HERE/'results/repair_phase_eight_v2_20261113'
def save(p,x):p.write_text(json.dumps(x,indent=2)+'\n')
def main():
 OUT.mkdir(exist_ok=False);active=[];records=[];began=time.perf_counter()
 for seed in range(8):
  log=(OUT/f'seed_{seed}.log').open('w');p=subprocess.Popen([sys.executable,'-u',str(HERE/'backtracked_phase_run.py'),'--seed',str(seed),'--outdir',str(OUT/f'ours_{seed}')],stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy());active.append(dict(seed=seed,p=p,log=log))
 while active:
  for a in list(active):
   code=a['p'].poll()
   if code is not None:
    a['log'].close();active.remove(a);p=OUT/f"ours_{a['seed']}"/'status.json';r=json.loads(p.read_text()) if p.exists() else {};records.append(dict(seed=a['seed'],exit_code=code,status=r));save(OUT/'summary.json',records)
  save(OUT/'status.json',dict(state='running',complete=len(records),elapsed_s=time.perf_counter()-began,active=[dict(seed=a['seed'],pid=a['p'].pid) for a in active]));time.sleep(2)
 save(OUT/'status.json',dict(state='complete',complete=len(records),elapsed_s=time.perf_counter()-began,active=[]))
if __name__=='__main__':main()
