"""Offline export for C++ parity checks. No trajectory optimization is performed."""
import argparse
import json
from pathlib import Path
import casadi as ca
import numpy as np
from convex_timevarying_window.comparisons.论文实验.实验2.higher_swing import scenario, run
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.fast_ipopt import build_problem,coordinate_parameters

parser=argparse.ArgumentParser()
parser.add_argument('--level',choices=scenario.LEVELS,required=True)
parser.add_argument('--method',choices=('ours','center'),required=True)
parser.add_argument('--outdir',type=Path,required=True)
args=parser.parse_args()
track,_=scenario.build_case(scenario.Case(level=args.level))
planner=run.make_planner(track,args.method=='ours')
seed=planner.seeds()[0]
planner._fast_time_origin=np.cumsum(np.exp(seed[:planner.n]))
planner._fast_normalization=float(np.exp(seed[:planner.n]).sum())
problem=build_problem(planner,planner.witnesses(),lambda row:None,np.inf,False)
parameters=coordinate_parameters(planner)
point=np.r_[np.zeros(planner.n),seed[planner.n:planner.velocity_start],
    seed[planner.velocity_start:]/problem['state_scale']]
point[planner.velocity_start:]-=np.asarray(problem['baseline'](point,parameters)).ravel()/problem['state_scale']
residuals=np.asarray(problem['residuals'](point,parameters)).ravel()
initial=np.r_[point,np.maximum(residuals,0.)/problem['slack_scale']]
out=args.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
formulation=problem['formulation'];formulation.save(str(out/'nlp.casadi'))
x,p=formulation.mx_in();f,g=formulation.call([x,p])
checks=ca.Function('finite_checks',[x,p],[f,g,ca.gradient(f,x),ca.jacobian(g,x)])
checks.save(str(out/'checks.casadi'))
pack=dict(initial=initial,parameters=parameters,
    lower_x=np.r_[np.full(planner.nvar,-np.inf),np.zeros(len(initial)-planner.nvar)],
    upper_x=np.full(len(initial),np.inf),lower_g=problem['lower'],upper_g=problem['upper'])
for name,value in pack.items():np.savetxt(out/(name+'.txt'),value,fmt='%.17e')
for index,value in enumerate(checks(initial,parameters)):
    np.savetxt(out/f'expected_{index}.txt',np.asarray(value.nonzeros()),fmt='%.17e')
with (out/'solver_options.txt').open('w') as stream:
    for key,value in problem['solver_options'].items():
        kind='bool' if isinstance(value,bool) else 'int' if isinstance(value,int) else 'double' if isinstance(value,float) else 'string'
        stream.write(f'{key} {kind} {str(value).lower() if kind=="bool" else value}\n')
(out/'manifest.json').write_text(json.dumps(dict(level=args.level,method=args.method,
    variables=len(initial),constraints=len(problem['lower']),optimization_run=False),indent=2)+'\n')
