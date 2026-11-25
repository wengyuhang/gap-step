"""Restart the saved free finite NLP, fixing witnesses, physical curve and scaling."""
from pathlib import Path
import sys,argparse,json
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import load,HERE
scene,higher,common,_,_=load()
from repair import Planner as Base,explicit_signs
from convex_timevarying_window.global_gate_minco_probe.planner import Plan

a=argparse.ArgumentParser();a.add_argument('--exact',action='store_true');a.add_argument('--reset-witnesses',action='store_true');a.add_argument('--method',choices=['ours','center'],default='ours');q=a.parse_args();mode=('exact' if q.exact else 'lbfgs')+('_base' if q.reset_witnesses else '')+('_center' if q.method=='center' else '');out=HERE/f'results/theory_audit_v1_20261114/n10_compact/restart_{mode}';out.mkdir(exist_ok=False);(out/'jobs').mkdir();level='n10_compact';summary=common.read(HERE/'results/paired6_v1_20261114/summary.json');best=next(r for r in summary['pairs'] if r['level']==level)['methods'][q.method]['best'];raw=common.read(Path(best['source']));old_initial=common.read(Path(best['source']).with_name('initial.json'));origin=np.cumsum(np.exp(np.array(old_initial['variables'])[:11]));normalization=float(origin[-1])
class Planner(Base):
 def witnesses(self):return getattr(self,'fixed_witnesses',None) or super().witnesses()
 def constrained_minimize(self,fun,initial,**kwargs):
  self._fast_time_origin=origin.copy();self._fast_normalization=normalization
  if q.exact:import finite_exact as module
  else:from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics import fast_ipopt as module
  result=module.minimize(self,fun,initial,**kwargs);result.verification_ready=bool(result.success);return result

def factory(track,free):
 p=Planner(track,free,pose_function=scene.pose);wit=p.witnesses()
 for call in raw['optimizer_calls']:
  assert call['status']=='Solve_Succeeded';x=np.array(call['original_variables']);T=np.exp(x[:p.n]);_,_,C=p.evaluate(x,wit);checks,passed=p.verify(Plan(0.,0,T,C,x,0.,0.,'reconstruct-exchange',p.m))
  if passed:continue
  for item in checks:
   if item['passed'] or item['witness_u'] is None:continue
   key=(item['gate'],item['piece']);target=wit.setdefault(key,dict(kind='normalized',values=[]));u=item['witness_u'];v=(1-u)*T[key[1]] if target['kind']=='before' else u*T[key[1]] if target['kind']=='after' else u
   if all(abs(old-v)>1e-5 for old in target['values']):target['values'].append(float(v))
 p.fixed_witnesses=Base.witnesses(p) if q.reset_witnesses else wit;p.seeds=lambda:[np.array(raw['variables'])];common.save(out/'protocol.json',dict(diagnostic_only=True,source=best['source'],same_witness_set=not q.reset_witnesses,same_time_coordinate_origin=origin.tolist(),same_objective_normalization=normalization,hessian=mode,witnesses=[dict(gate=k[0],piece=k[1],**v) for k,v in sorted(wit.items())]));return p
higher.make_planner=factory;job=dict(id='finite_restart',method=q.method,seed=0,level=level);common.save(out/'jobs/finite_restart.json',job);higher.worker(out,job['id']);folder=out/'plans/finite_restart';r=common.read(folder/'result.json');r.update(diagnostic_only=True,formal_comparison_candidate=False,source_free=best['source'],qualified=False)
if r['found']:
 from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.trajectory_only import collision
 collision.route_signs=lambda tr:explicit_signs(tr)[0];tr,_=scene.build_case(scene.Case(level));g=collision.evaluate(r['coefficients'],r['durations'],tr,scene.ASSETS/'meshes');common.save(folder/'assessment.json',g);r['qualified']=g['passed']
common.save(folder/'result.json',r);print(mode,r['found'],r.get('nominal_time_s'),r['qualified'],flush=True)
