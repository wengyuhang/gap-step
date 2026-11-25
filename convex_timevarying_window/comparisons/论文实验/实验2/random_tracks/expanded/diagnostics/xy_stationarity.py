"""Diagnostic only: same physical free domain in explicit xy coordinates."""
from pathlib import Path
import sys,json,importlib.util,hashlib,argparse
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import load,HERE,PARENT
scene,higher,common,_,_=load()
from repair import Planner as Base,explicit_signs
from convex_timevarying_window.single_gate_stop_racing import point_mapping

cli=argparse.ArgumentParser();cli.add_argument('--saved-witnesses',action='store_true');args=cli.parse_args();out=HERE/('results/theory_audit_v1_20261114/n10_compact/xy_fixed_witness' if args.saved_witnesses else 'results/theory_audit_v1_20261114/n10_compact/xy_diagnostic');out.mkdir(exist_ok=False);(out/'jobs').mkdir()
# Keep an exact copy of the executed diagnostic solver, outside formal sources.
source=(PARENT/'finite_xy.py').read_text().replace("'ipopt.hessian_approximation': 'limited-memory', 'ipopt.limited_memory_max_history': 6,","'ipopt.hessian_approximation': 'exact',").replace("hessian='limited-memory', history=6","hessian='exact', history=None");solverfile=out/'finite_xy_exact.py';solverfile.write_text(source);spec=importlib.util.spec_from_file_location('diagnostic_xy_exact',solverfile);finite=importlib.util.module_from_spec(spec);spec.loader.exec_module(finite)
point_mapping.dimension=lambda ap:2
point_mapping.symbolic=lambda ap,d:d
class Planner(Base):
 def witnesses(self):return getattr(self,'fixed_witnesses',None) or super().witnesses()
 def constrained_minimize(self,fun,initial,**kwargs):
  
  if args.saved_witnesses:
   self._fast_time_origin=np.array(control['same_time_coordinate_origin']);self._fast_normalization=control['same_objective_normalization']
  result=finite.minimize(self,fun,initial,**kwargs);result.verification_ready=bool(result.success);return result

control=common.read(HERE/'results/theory_audit_v1_20261114/n10_compact/restart_exact/protocol.json') if args.saved_witnesses else None
level='n10_compact';summary=json.loads((HERE/'results/paired6_v1_20261114/summary.json').read_text());best=next(r for r in summary['pairs'] if r['level']==level)['methods']['ours']['best'];raw=common.read(Path(best['source']));track,_=scene.build_case(scene.Case(level));points=np.array(raw['crossing_points']);times=np.cumsum(raw['durations'])[:-1];q=np.array([w.world_to_local(p,t)[0] for w,p,t in zip(track.windows,points,times)]);x=np.r_[raw['variables'][:len(track.windows)+1],q.ravel(),raw['variables'][-9*len(track.windows):]]
def factory(track,free):
 p=Planner(track,True,pose_function=scene.pose);
 if args.saved_witnesses:p.fixed_witnesses={(r['gate'],r['piece']):dict(kind=r['kind'],values=r['values']) for r in control['witnesses']}
 C=np.asarray(p.curve_values(x)).ravel().reshape(p.n,3,8).transpose(0,2,1);cost=p.objective(x)[0];err=float(abs(C-np.array(raw['coefficients'])).max());assert err<1e-7 and abs(cost-raw['objective'])<1e-6;common.save(out/'initial_equivalence.json',dict(coefficient_max_error=err,objective_difference=cost-raw['objective'],source=best['source'],same_inset_region=True,new_constraints='circle disk or original inset polygon halfplanes',diagnostic_solver_sha256=hashlib.sha256(source.encode()).hexdigest()));p.seeds=lambda:[x.copy()];return p
higher.make_planner=factory;job=dict(id='xy_diagnostic',method='ours',seed=0,level=level);common.save(out/'jobs/xy_diagnostic.json',job);higher.worker(out,job['id']);folder=out/'plans/xy_diagnostic';r=common.read(folder/'result.json');r.update(diagnostic_only=True,formal_comparison_candidate=False,optimized_initial_solution_used=True,source_free=best['source'],point_parameterization='physical xy with exactly original inset constraints',hessian='exact',qualified=False)
if r['found']:
 from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.trajectory_only import collision
 collision.route_signs=lambda tr:explicit_signs(tr)[0];g=collision.evaluate(r['coefficients'],r['durations'],track,scene.ASSETS/'meshes');common.save(folder/'assessment.json',g);r.update(qualified=g['passed'],independent_verdict=g['verdict'],trajectory_distance_m=g['trajectory_distance_m'])
common.save(folder/'result.json',r);print(r['found'],r.get('nominal_time_s'),r.get('qualified'),flush=True)
