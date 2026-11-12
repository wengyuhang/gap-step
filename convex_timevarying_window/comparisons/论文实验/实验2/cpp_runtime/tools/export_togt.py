"""Export exact moving safe maps, penalty and frozen original TOGT starts."""
import argparse,json
from pathlib import Path
import casadi as ca
import numpy as np
from convex_timevarying_window.comparisons.论文实验.实验2.higher_swing import scenario,run
from convex_timevarying_window.comparisons.论文实验.实验2.matched_full_matrix.togt import matched_objective,initial_vector
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.kernels import PolynomialPenalty

parser=argparse.ArgumentParser();parser.add_argument('--level',choices=scenario.LEVELS,required=True)
parser.add_argument('--method',choices=('dynamic_togt','dynamic_togt_center'),required=True)
parser.add_argument('--outdir',type=Path,required=True)
parser.add_argument('--dynamics-weight',type=float,default=1000.)
args=parser.parse_args()
if not np.isfinite(args.dynamics_weight) or args.dynamics_weight < 0.:
 parser.error('dynamics weight must be finite and nonnegative')
out=args.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
track,_=scenario.build_case(scenario.Case(level=args.level));initializer=run.make_planner(track,True)
objective=matched_objective(track,args.method.endswith('_center'),initializer)
x=ca.MX.sym('togt_variables',objective.dimension);n=objective.temporal_dimension
k=x[:n];times=ca.if_else(k>0.,.5*k*k+k+1.,1./(.5*k*k-k+1.));arrivals=ca.cumsum(times)
points=[];offset=n
for gate,window in enumerate(objective.track.windows):
 ap=window.aperture;d=ca.DM(objective.center_parameters[gate]) if objective.center else x[offset:offset+ap.dimension]
 if ap.kind=='circle':q=2.*(ap.radius-ap.margin/2.)*d/(1.+ca.sumsqr(d))
 else:
  unit=d/ca.norm_2(d);q=ca.DM(ap.safe_vertices[0])+ca.DM((ap.safe_vertices[1:]-ap.safe_vertices[0]).T)@(unit[:-1]**2)
 center,rotation=scenario.pose(window,arrivals[gate]);points.append(center+rotation[:,:2]@q);offset+=ap.dimension
values=ca.vertcat(times,ca.vec(ca.horzcat(*points)))
geometry=ca.Function('togt_geometry',[x],[values,ca.jacobian(values,x)])
geometry.save(str(out/'geometry.casadi'))
PolynomialPenalty('togt',.95).segment.save(str(out/'penalty.casadi'))
initials=[]
for index in range(8):
 seed=initial_vector(objective,initializer,index);cost,gradient=objective.value_and_gradient(seed)
 shape,jacobian=geometry(seed)
 total=float(np.asarray(shape).ravel()[:n].sum())
 time_gradient=np.asarray(jacobian)[:n].sum(axis=0)
 cost=total+(cost-total)*args.dynamics_weight/1000.
 gradient=time_gradient+(gradient-time_gradient)*args.dynamics_weight/1000.
 forward=objective.forward(seed);C=forward.trajectory.coefficients*forward.durations[:,None,None]**np.arange(8)[None,:,None]
 initials.append(dict(index=index,variables=seed.tolist(),expected_cost=cost,expected_gradient=gradient.tolist(),expected_coefficients=C.tolist()))
(out/'manifest.json').write_text(json.dumps(dict(level=args.level,method=args.method,n=n,m=n-1,
 start=track.start.tolist(),goal=track.goal.tolist(),initials=initials,dynamics_weight=args.dynamics_weight,invalid_trial_cost=objective.config.invalid_trial_cost,
 memory=objective.config.memory_size,max_linesearch=objective.config.max_line_search_steps,past=objective.config.past_iterations,
 past_tolerance=objective.config.function_tolerance,gtol=objective.config.gradient_tolerance,
 max_iterations=2147483647 if objective.config.max_iterations==0 else objective.config.max_iterations,optimization_run=False),indent=2)+'\n')
print('TOGT_EXPORTED',args.level,args.method,flush=True)
