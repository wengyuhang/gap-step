"""Test feasible-set embedding and mapped-coordinate conditioning; no optimization."""
from pathlib import Path
import sys,json
import numpy as np
import casadi as ca
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import load,HERE
scene,*_=load()
from repair import Planner,explicit_signs
from convex_timevarying_window.single_gate_stop_racing import point_mapping
from convex_timevarying_window.comparisons.论文实验.实验2.safety import frame_aperture
from convex_timevarying_window.single_gate_stop_racing.geometry import halfplanes
from convex_timevarying_window.global_gate_minco_probe.planner import Plan

def main():
 run=HERE/'results/paired6_v1_20261114';out=HERE/'results/theory_audit_v1_20261114';out.mkdir(exist_ok=False);summary=json.loads((run/'summary.json').read_text());candidates=json.loads((run/'candidates.json').read_text());records=[]
 for pair in summary['pairs']:
  if not pair['level'].startswith('n10_'):continue
  tr,_=scene.build_case(scene.Case(pair['level']));p=Planner(tr,True,pose_function=scene.pose);c=Planner(tr,False,pose_function=scene.pose);params=np.concatenate([point_mapping.inverse(ap,np.zeros(2)) for ap in p.point_apertures]);embedding=[]
  for row in candidates:
   if row['level']!=pair['level'] or row['method']!='center' or not row['qualified']:continue
   raw=json.loads(Path(row['source']).read_text());x=np.r_[raw['variables'][:p.n],params,raw['variables'][p.n:]];C=np.asarray(p.curve_values(x)).ravel().reshape(p.n,3,8).transpose(0,2,1);old=np.array(raw['coefficients']);g=p.evaluate(x,p.witnesses())[0];gc=c.evaluate(np.array(raw['variables']),c.witnesses())[0];cost=p.objective(x)[0];rowtest=dict(seed=row['seed'],coefficient_max_error=float(abs(C-old).max()),objective_difference=cost-raw['objective'],finite_constraint_max_difference=float(abs(g-gc).max()),finite_minimum=float(g.min()),polygon_gauge_max_error=max([abs(float(params[sl.start-p.n:sl.stop-p.n]@params[sl.start-p.n:sl.stop-p.n])-1) for w,sl in zip(tr.windows,p.slices) if w.aperture.kind!='circle'],default=0.));assert rowtest['coefficient_max_error']<1e-7 and abs(rowtest['objective_difference'])<1e-6 and rowtest['finite_constraint_max_difference']<1e-7;embedding.append(rowtest)
  center=pair['methods']['center']['best'];rawc=json.loads(Path(center['source']).read_text());x=np.r_[rawc['variables'][:p.n],params,rawc['variables'][p.n:]];C=np.array(rawc['coefficients']);T=np.array(rawc['durations']);checks,passed=p.verify(Plan(0.,0,T,C,x,0.,0.,'center-embedding-audit',p.m));assert passed
  caseout=out/pair['level'];caseout.mkdir();(caseout/'release_initial.json').write_text(json.dumps(dict(explicit_signs=list(map(float,explicit_signs(tr)[0])),seeds=[dict(variables=x.tolist())],provenance='diagnostic only: best qualified center embedded unchanged in free-point variables',source_center=center['source']),indent=2)+'\n')
  free=pair['methods']['ours']['best'];rawf=json.loads(Path(free['source']).read_text());xf=np.array(rawf['variables']);ranks=[]
  for i,(w,ap,sl) in enumerate(zip(tr.windows,p.point_apertures,p.slices)):
   d=ca.MX.sym('mapped_point',sl.stop-sl.start);fun=ca.Function('mapping_jacobian',[d],[ca.jacobian(point_mapping.symbolic(ap,d),d)]);J=np.array(fun(xf[sl]));sv=np.linalg.svd(J,compute_uv=False);ranks.append(dict(gate=i+1,shape=w.shape_name,parameters=xf[sl].tolist(),singular_values=sv.tolist(),circle_parameter_norm=float(np.linalg.norm(xf[sl])) if ap.kind=='circle' else None,min_abs_normalized_polygon_parameter=float(np.min(abs(xf[sl]))/np.linalg.norm(xf[sl])) if ap.kind!='circle' else None))
  linear=[];body=float(rawc['verification_safety_radius_m'])
  for i,(w,at) in enumerate(zip(tr.windows,np.cumsum(T)[:-1])):
   V=np.polynomial.polynomial.polyval(1,np.polynomial.polynomial.polyder(C[i]))/T[i];relative=w.R.T@(V-w.state_at(at)[2]);vn=abs(relative[2]);ap=frame_aperture(w.aperture)
   if ap.kind=='circle':need=body*np.sqrt(1+np.linalg.norm(relative[:2])**2/vn**2);ratio=need/ap.radius
   else:
    normals,bounds=halfplanes(ap);need=body*np.sqrt(1+(normals@relative[:2]/vn)**2);ratio=float(np.max(need/bounds))
   linear.append(dict(gate=i+1,relative_tangent_speed=float(np.linalg.norm(relative[:2])),normal_speed=float(vn),linear_swept_body_fraction=float(ratio),note='first-order diagnostic, not a safety certificate'))
  calls=rawf['optimizer_calls'];record=dict(level=pair['level'],embedding_all_center_starts=embedding,best_center_full_verification_passed=passed,free_mapping_jacobians=ranks,center_relative_motion=linear,free_last_finite={k:calls[-1].get(k) for k in ['status','iterations','constraint_violation_inf','lagrangian_stationarity_inf','epigraph_gap']},free_time=rawf['nominal_time_s'],center_time=rawc['nominal_time_s']);records.append(record);print(pair['level'],'embedding passed',len(embedding),'max coefficient error',max(r['coefficient_max_error'] for r in embedding),'map min singular',min(r['singular_values'][-1] for r in ranks),'linear fractions',[round(r['linear_swept_body_fraction'],3) for r in linear],flush=True)
 (out/'AUDIT.json').write_text(json.dumps(records,ensure_ascii=False,indent=2)+'\n')
if __name__=='__main__':main()
