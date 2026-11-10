"""Check assembled native finite problems after adaptive constraint insertion."""
import argparse,json,subprocess
from pathlib import Path
import casadi as ca
import numpy as np
from convex_timevarying_window.comparisons.论文实验.实验2.higher_swing import scenario,run
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.fast_ipopt import build_problem,coordinate_parameters
p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--outdir',type=Path,required=True);p.add_argument('--graph-root',type=Path,default=Path(__file__).resolve().parents[1]/'results/graphs_original_exchange_v2_20261110');a=p.parse_args();a.outdir.mkdir(parents=True,exist_ok=False)
raw=json.loads(a.source.read_text());level=raw['level'];method=raw['method'];index=raw['initial_sample_index']
track,_=scenario.build_case(scenario.Case(level=level));planner=run.make_planner(track,method=='ours');seed=planner.seeds()[index]
planner._fast_time_origin=np.cumsum(np.exp(seed[:planner.n]));planner._fast_normalization=float(np.exp(seed[:planner.n]).sum());parameters=coordinate_parameters(planner)
witnesses=planner.witnesses()
records=raw.get('records')
if records is None:records=raw['candidates'][0]['statistics']['records']
for record in records[:3]:
 durations=np.exp(raw['optimizer_calls'][record['round']]['original_variables'][:planner.n])
 for check in record['checks']:
  if check['passed'] or check['witness_u'] is None:continue
  key=check['gate'],check['piece'];target=witnesses.setdefault(key,dict(kind='normalized',values=[]));u=check['witness_u']
  offset=(1-u)*durations[key[1]] if target['kind']=='before' else u*durations[key[1]] if target['kind']=='after' else u
  if all(abs(v-offset)>1e-5 for v in target['values']):target['values'].append(offset)
rows=[dict(gate=gate,piece=piece,**value) for (gate,piece),value in sorted(witnesses.items())]
(a.outdir/'witnesses.json').write_text(json.dumps(rows)+'\n')
problem=build_problem(planner,witnesses,lambda row:None,np.inf,False)
original=np.array(raw['optimizer_calls'][min(3,len(raw['optimizer_calls'])-1)]['original_variables'])
point=np.r_[(np.cumsum(np.exp(original[:planner.n]))-parameters[1:1+planner.n])/parameters[1+planner.n:],
 original[planner.n:planner.velocity_start],original[planner.velocity_start:]/problem['state_scale']]
point[planner.velocity_start:]-=np.asarray(problem['baseline'](point,parameters)).ravel()/problem['state_scale']
residual=np.asarray(problem['residuals'](point,parameters)).ravel();variables=np.r_[point,np.maximum(residual,0.)/problem['slack_scale']]
f=problem['formulation'];x,param=f.mx_in()
gradient=problem['solver'].get_function('nlp_grad_f')(x,param)
jacobian=problem['solver'].get_function('nlp_jac_g')(x,param)
checks=ca.Function('expected_checks',[x,param],[gradient[0],jacobian[0],gradient[1],jacobian[1]])
root=Path(__file__).resolve().parents[1]
subprocess.run(['/tmp/window_cpp_runtime_build/sip_probe',str(a.graph_root/f'{level}_{method}'),str(a.outdir/'witnesses.json'),str(a.outdir)],check=True)
native=ca.Function.load(str(a.outdir/'checks.casadi'));expected=checks(variables,parameters);actual=native(variables,parameters)
errors=[float(np.max(abs(np.asarray(a)-np.asarray(e))/(1+abs(np.asarray(e))))) for a,e in zip(actual,expected)]
result=dict(passed=max(errors)<1e-8,bitwise_equal=max(errors)==0,relative_max_errors=errors,witnesses=len(rows),source=str(a.source))
(a.outdir/'validation.json').write_text(json.dumps(result,indent=2)+'\n');print(result,flush=True)
