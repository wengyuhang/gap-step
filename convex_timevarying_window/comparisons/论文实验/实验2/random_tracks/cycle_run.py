"""Independent same-track periodic-arrival globalization, original objective."""
from pathlib import Path
import json,sys,os,time,subprocess,argparse
from bootstrap import load,HERE
scene,higher,common,best,rollout=load()
from repair import explicit_signs
import numpy as np

def save(p,r):p.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
def advance(x,n,freq,factor):
 T=np.exp(x[:n]);arr=np.cumsum(T);new=arr.copy();previous=0.
 for i,f in enumerate(freq):
  period=1/f;k=max(0,int(round((1-factor)*arr[i]/period)))
  while k>0 and arr[i]-k*period<previous+.3:k-=1
  new[i]=arr[i]-k*period;previous=new[i]
 new[-1]=max(factor*arr[-1],previous+.3);result=x.copy();result[:n]=np.log(np.diff(np.r_[0.,new]));return result

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--seed',type=int,required=True);ap.add_argument('--outdir',type=Path,required=True);ap.add_argument('--method',default='ours');a=ap.parse_args();a.outdir.mkdir(parents=True,exist_ok=False)
 init=common.read(HERE/'results/pv_transit_initials_v1_20261113/2026111301_high.json');x=np.array(init['seeds'][a.seed]['variables']);track,_=scene.build_case(scene.Case('2026111301_high'));n=8
 if a.method=='center':x=np.r_[x[:n],x[-63:]]
 best_raw=None;stages=[];began=time.perf_counter();periods=[w.motion.frequency for w in track.windows]
 for stage in range(25):
  folder=a.outdir/f'stage_{stage:02d}';(folder/'jobs').mkdir(parents=True);job=dict(id=f'{a.method}_0',method=a.method,seed=0,level='2026111301_high');save(folder/'jobs'/f"{job['id']}.json",job)
  # Transport the center branch through the shared free-layout seed adapter.
  free_x=x if a.method=='ours' else np.r_[x[:8],np.zeros(25),x[8:]]
  save(folder/'initial.json',dict(explicit_signs=list(map(float,explicit_signs(track)[0])),seeds=[dict(variables=free_x.tolist())]))
  with (folder/'worker.log').open('w') as log:
   process=subprocess.run([sys.executable,'-u',str(HERE/'cycle_worker.py'),'--outdir',str(folder),'--job',job['id'],'--initial',str(folder/'initial.json')],stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy())
  path=folder/'plans'/job['id']/'result.json';r=common.read(path) if path.exists() else {};qualified=bool(process.returncode==0 and r.get('found') and r.get('independent_frame_clear') and r.get('independent_task_passed'))
  row=dict(stage=stage,qualified=qualified,exit_code=process.returncode,nominal_time_s=r.get('nominal_time_s'),objective=r.get('objective'),solve_s=r.get('solve_s'));stages.append(row);save(a.outdir/'stages.json',stages);print(a.seed,a.method,row,flush=True)
  if not qualified:break
  if best_raw is not None and r['objective']>=best_raw['objective']-1e-5:break
  best_raw=r;save(a.outdir/'best_result.json',r)
  next_x=advance(np.array(r['variables']),n,periods,.95)
  old=np.cumsum(r['durations'])[:-1];new=np.cumsum(np.exp(next_x[:n]))[:-1]
  for w,t0,t1 in zip(track.windows,old,new):np.testing.assert_allclose(w.state_at(t0)[0],w.state_at(t1)[0],atol=1e-8)
  x=next_x
 save(a.outdir/'summary.json',dict(seed=a.seed,method=a.method,qualified=best_raw is not None,best_time_s=best_raw.get('nominal_time_s') if best_raw else None,best_objective=best_raw.get('objective') if best_raw else None,stages=stages,wall_s=time.perf_counter()-began))
if __name__=='__main__':main()
