"""Replay an unchanged failed finite NLP with fresh primal/slack/dual initialization."""
from pathlib import Path
import argparse,json,time
import numpy as np
from bootstrap import HERE,load
scene,*_=load()
from sampled_safe_model import Planner
from convex_timevarying_window.global_gate_minco_probe.planner import Plan

def save(p,r):p.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
def main():
 a=argparse.ArgumentParser();a.add_argument('--hessian',choices=['limited-memory','exact'],required=True);a.add_argument('--outdir',type=Path,required=True);q=a.parse_args();q.outdir.mkdir(parents=True,exist_ok=False)
 source=HERE/'results/repair_phase_eight_v2_20261113/ours_2/attempt_08';initial=json.loads((source/'initial.json').read_text());seed=np.array(initial['seeds'][0]['variables']);track,_=scene.build_case(scene.Case('2026111301_high'));p=Planner(track,True,pose_function=scene.pose);p.initial_for_witnesses=seed;witnesses=p.witnesses();rows=[json.loads(s) for s in (source/'plans/ours_0/progress.jsonl').read_text().splitlines()];calls=[r for r in rows if r.get('stage')=='factorized_finite_ipopt']
 # Reconstruct exactly the original exchange-set history, without inventing points.
 for call in calls:
  x=np.array(call['original_variables']);g,_,C=p.evaluate(x,witnesses);T=np.exp(x[:p.n]);checks,passed=p.verify(Plan(0.,0,T,C,x,0.,0.,'replay-finite-history',7))
  if passed:continue
  for item in checks:
   if item['passed'] or item['witness_u'] is None:continue
   key=(item['gate'],item['piece']);target=witnesses.setdefault(key,dict(kind='normalized',values=[]));u=item['witness_u'];value=(1-u)*T[key[1]] if target['kind']=='before' else u*T[key[1]] if target['kind']=='after' else u
   if all(abs(v-value)>1e-5 for v in target['values']):target['values'].append(float(value))
 x=np.array(calls[-1]['original_variables']);p._fast_time_origin=np.cumsum(np.exp(seed[:p.n]));p._fast_normalization=float(np.exp(seed[:p.n]).sum())
 if q.hessian=='exact':from finite_direct_exact import build_problem,coordinate_parameters
 else:from finite_direct import build_problem,coordinate_parameters
 progress_file=(q.outdir/'progress.jsonl').open('w')
 def progress(row):progress_file.write(json.dumps(row)+'\n');progress_file.flush()
 problem=build_problem(p,witnesses,progress,np.inf,False);parameters=coordinate_parameters(p);z=np.r_[(np.cumsum(np.exp(x[:p.n]))-parameters[1:p.n+1])/parameters[p.n+1:],x[p.n:p.velocity_start],x[p.velocity_start:]/problem['state_scale']];residual=np.asarray(problem['residuals'](z,parameters)).ravel();starting=np.r_[z,np.maximum(residual,0)/problem['slack_scale']]
 assert len(problem['lower'])==calls[-1]['constraint_rows'],(len(problem['lower']),calls[-1]['constraint_rows'])
 save(q.outdir/'problem.json',dict(source=str(source),source_finite_calls=len(calls),initial_from='own last returned physical trajectory',rows=len(problem['lower']),variables=len(starting),hessian=q.hessian,warm_multipliers=False,witnesses=[dict(gate=k[0],piece=k[1],**v) for k,v in sorted(witnesses.items())],normalization=p._fast_normalization))
 began=time.perf_counter();answer=problem['solver'](x0=starting,p=parameters,lbx=np.r_[np.full(p.nvar,-np.inf),np.zeros(len(starting)-p.nvar)],ubx=np.full(len(starting),np.inf),lbg=problem['lower'],ubg=problem['upper']);stats=problem['solver'].stats();elapsed=time.perf_counter()-began;returned=np.asarray(answer['x']).ravel();original=np.asarray(problem['unpack'](returned[:p.nvar],parameters)).ravel();g=np.asarray(answer['g']).ravel();grad=np.asarray(problem['gradient'](returned,parameters)).ravel()+np.asarray(problem['jacobian'](returned,parameters)).T@np.asarray(answer['lam_g']).ravel()+np.asarray(answer['lam_x']).ravel();native,_,pen=p.objective(original);_,_,C=p.evaluate(original,witnesses);T=np.exp(original[:p.n]);checks,passed=p.verify(Plan(0.,0,T,C,original,0.,0.,'finite-recovery',7));result=dict(success=bool(stats['success']),status=stats['return_status'],iterations=int(stats['iter_count']),solver_s=elapsed,build_s=problem['build_s'],variables=original.tolist(),nominal_time_s=float(T.sum()),objective=native,dynamic_soft_integral=pen,full_safety_passed=passed,checks=checks,constraint_violation_inf=float(np.maximum(np.maximum(problem['lower']-g,g-problem['upper']),0).max()),stationarity_inf=float(abs(grad).max()),top_stationarity=[dict(index=int(i),value=float(grad[i])) for i in np.argsort(abs(grad))[-10:][::-1]],epigraph_gap=float(answer['f'])*p._fast_normalization-native,coefficients=C.tolist(),durations=T.tolist());save(q.outdir/'result.json',result);print({k:v for k,v in result.items() if k not in ['variables','checks','coefficients','durations']},flush=True)
if __name__=='__main__':main()
