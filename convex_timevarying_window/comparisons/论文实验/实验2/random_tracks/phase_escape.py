"""Try the adjacent opposite-motion phase of a dynamically critical segment."""
from pathlib import Path
import sys,os,time,json,subprocess,argparse
import numpy as np
from bootstrap import load,HERE
scene,higher,common,_,_=load()
from repair import Planner,explicit_signs
from cycle_run import advance
from convex_timevarying_window.single_gate_stop_racing import point_mapping
from convex_timevarying_window.single_gate_stop_racing.geometry import halfplanes
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics import kernels

def save(p,r):p.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
def main():
 a=argparse.ArgumentParser();a.add_argument('--source',required=True,type=Path);a.add_argument('--outdir',required=True,type=Path);a.add_argument('--seed',required=True,type=int);q=a.parse_args();q.outdir.mkdir(parents=True,exist_ok=False);raw=common.read(q.source);save(q.outdir/'source_result.json',raw);tr,_=scene.build_case(scene.Case(raw['case']['level']));p=Planner(tr,True,pose_function=scene.pose);x=np.array(raw['variables']);clocks=np.cumsum(np.exp(x[:8]));test=advance(x,8,[w.motion.frequency for w in tr.windows],.95);T=np.exp(test[:8]);C=np.array(p.curve_values(test)).ravel().reshape(8,3,8).transpose(0,2,1);cost=[]
 for c,t in zip(C,T):
  residual=np.array(kernels.dynamic_residuals('togt',.95)(np.r_[c.T.ravel(),t])).ravel();cost.append(float(kernels.slack_integral('togt',.95)(np.maximum(residual,0),t)))
 critical=int(np.argmax(cost));gates=sorted(set(i for i in [critical-1,critical] if 0<=i<7));active=[];rows=[];began=time.perf_counter()
 for gate in gates:
  for side in [-1,1]:
   z=x.copy();stamp=clocks.copy();w=tr.windows[gate];stamp[gate]+=side*.5/w.motion.frequency;dur=np.diff(np.r_[0,stamp])
   if min(dur)<.3 or max(dur)>15:continue
   z[:8]=np.log(dur);d=w.R@np.array(w.motion.direction_local) if w.motion.kind=='translation' else w.R[:,1];d=d/np.linalg.norm(d);M=np.eye(3)-2*np.outer(d,d);ap=p.point_apertures[gate];sl=p.slices[gate];loc=point_mapping.numeric(ap,x[sl]);loc=(w.R.T@M@w.R@np.r_[loc,0])[:2]
   if ap.kind=='circle':loc*=min(1.,ap.radius*.999999/max(np.linalg.norm(loc),1e-15))
   else:
    normals,bounds=halfplanes(ap);dot=normals@loc;ids=dot>0;loc*=min(1.,float(np.min(bounds[ids]/dot[ids]))*.999999) if ids.any() else 1.
   z[sl]=point_mapping.inverse(ap,loc)
   for order in range(3):
    s=slice(p.velocity_start+21*order+3*gate,p.velocity_start+21*order+3*gate+3);z[s]=M@x[s]
   name=f'gate{gate+1}_{side:+d}';folder=q.outdir/name;(folder/'jobs').mkdir(parents=True);save(folder/'jobs/ours_0.json',dict(id='ours_0',method='ours',seed=0,level=raw['case']['level']));save(folder/'initial.json',dict(explicit_signs=list(map(float,explicit_signs(tr)[0])),seeds=[dict(variables=z.tolist())]));log=(folder/'worker.log').open('w');process=subprocess.Popen([sys.executable,'-u',str(HERE/'sampled_cycle_worker.py'),'--outdir',str(folder),'--job','ours_0','--initial',str(folder/'initial.json')],stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy());active.append(dict(name=name,p=process,log=log,folder=folder))
 save(q.outdir/'protocol.json',dict(original_seed=q.seed,source=str(q.source),source_time_s=raw['nominal_time_s'],critical_piece=critical+1,predictor_segment_penalties=cost,trial_gates=[i+1 for i in gates],phase_step='plus/minus half period',derivatives='motion-axis reflection of own state',objective_unchanged=True,selection='qualified converged results only',borrowed_other_seed=False))
 while active:
  for a in list(active):
   code=a['p'].poll()
   if code is None:continue
   a['log'].close();active.remove(a);f=a['folder']/'plans/ours_0/result.json';r=common.read(f) if f.exists() else {};valid=bool(code==0 and r.get('found') and r.get('independent_frame_clear') and r.get('independent_task_passed'));row=dict(trial=a['name'],qualified=valid,exit_code=code,time_s=r.get('nominal_time_s'),objective=r.get('objective'),solve_s=r.get('solve_s'),finite_statuses=[c['status'] for c in r.get('optimizer_calls',[])]);rows.append(row);save(q.outdir/'summary.json',rows);print(q.seed,row,flush=True)
  save(q.outdir/'status.json',dict(state='running',completed=len(rows),elapsed_s=time.perf_counter()-began,active=[dict(trial=a['name'],pid=a['p'].pid) for a in active]));time.sleep(1)
 best=min([r for r in rows if r['qualified']],key=lambda r:r['objective'],default=None)
 if best:save(q.outdir/'best_result.json',common.read(q.outdir/best['trial']/'plans/ours_0/result.json'))
 save(q.outdir/'status.json',dict(state='complete',completed=len(rows),elapsed_s=time.perf_counter()-began,best_trial=best))
if __name__=='__main__':main()
