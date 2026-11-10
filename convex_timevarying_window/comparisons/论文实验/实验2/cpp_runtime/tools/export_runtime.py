"""Offline mathematical graph export for the native, unchanged SIP algorithm."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import casadi as ca
import numpy as np
from convex_timevarying_window.comparisons.论文实验.实验2.higher_swing import scenario,run
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics import kernels
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.fast_ipopt import build_problem,coordinate_parameters
from convex_timevarying_window.single_gate_stop_racing import point_mapping
from convex_timevarying_window.comparisons.论文实验.实验2.safety import symbolic_constraints

parser=argparse.ArgumentParser()
parser.add_argument('--level',choices=scenario.LEVELS,required=True)
parser.add_argument('--method',choices=('ours','center'),required=True)
parser.add_argument('--outdir',type=Path,required=True)
args=parser.parse_args()
out=args.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
track,_=scenario.build_case(scenario.Case(level=args.level))
planner=run.make_planner(track,args.method=='ours')
seeds=planner.seeds();first=seeds[0]
planner._fast_time_origin=np.cumsum(np.exp(first[:planner.n]))
planner._fast_normalization=float(np.exp(first[:planner.n]).sum())
problem=build_problem(planner,{},lambda row:None,np.inf,False)
formulation=problem['formulation'];formulation.save(str(out/'base.casadi'))
v,p=formulation.mx_in();z=v[:planner.nvar]
x=problem['unpack'](z,p);times=ca.exp(x[:planner.n])
coefficients=kernels.curve_coefficients(planner,x)
points=planner.point_function(x)
penalty=0.
for piece in range(planner.n):
 state=ca.vertcat(coefficients[24*piece:24*(piece+1)],times[piece])
 residuals=kernels.dynamic_residuals(planner.dynamics_model,planner.dynamic_limit_fraction)(state)
 penalty+=kernels.slack_integral(planner.dynamics_model,planner.dynamic_limit_fraction)(ca.fmax(residuals,0.),times[piece])
ca.Function('native_trajectory',[v,p],[x,times,coefficients,points,ca.sum1(times)+penalty,penalty]).save(str(out/'trajectory.casadi'))
problem['baseline'].save(str(out/'baseline.casadi'))
problem['residuals'].save(str(out/'residuals.casadi'))
radius=planner.cfg.radius+planner.cfg.frame_thickness+planner.cfg.passage_margin+planner.cfg.witness_slack
windows=[];slices=[];knot_expressions=[]
for gate,window in enumerate(track.windows):
 circle=window.aperture.kind=='circle'
 windows.append(dict(center=window.center0.tolist(),rotation=window.R.tolist(),circle=circle,
  swing=window.motion.kind=='swing',frequency=window.motion.frequency,amplitude=window.motion.amplitude,
  length=window.motion.length,phase=window.motion.phase,opening_radius=window.aperture.radius if circle else None,
  vertices=None if circle else window.aperture.physical_vertices.tolist()))
 for kind in ('before','after','normalized'):
  kernels.frame_constraints(window,planner.pose_function,kind,radius).save(str(out/f'frame_{gate}_{kind}.casadi'))
 local=point_mapping.symbolic(planner.point_apertures[gate],x[planner.slices[gate]]) if planner.free_points else ca.DM.zeros(2)
 knot_value=ca.vertcat(*symbolic_constraints(window.aperture,ca.vertcat(local,0.),radius))
 ca.Function('native_knot',[v,p],[knot_value]).save(str(out/f'knot_{gate}.casadi'))
 raw_x=problem['native_expressions'][2]
 raw_local=point_mapping.symbolic(planner.point_apertures[gate],raw_x[planner.slices[gate]]) if planner.free_points else ca.DM.zeros(2)
 knot_expressions.append(ca.vertcat(*symbolic_constraints(window.aperture,ca.vertcat(raw_local,0.),radius)))
 if planner.free_points:slices.append(dict(begin=planner.slices[gate].start,end=planner.slices[gate].stop,circle=circle))
serializer=ca.StringSerializer()
serializer.pack(problem['native_expressions']+knot_expressions)
(out/'symbolic.mx').write_text(serializer.encode())
planner.curve.save(str(out/'physical_curve.casadi'))
initials=[]
for index,seed in enumerate(seeds):
 planner._fast_time_origin=np.cumsum(np.exp(seed[:planner.n]))
 planner._fast_normalization=float(np.exp(seed[:planner.n]).sum())
 parameters=coordinate_parameters(planner)
 point=np.r_[np.zeros(planner.n),seed[planner.n:planner.velocity_start],seed[planner.velocity_start:]/problem['state_scale']]
 point[planner.velocity_start:]-=np.asarray(problem['baseline'](point,parameters)).ravel()/problem['state_scale']
 residuals=np.asarray(problem['residuals'](point,parameters)).ravel()
 initials.append(dict(index=index,original=seed.tolist(),parameters=parameters.tolist(),
  finite=np.r_[point,np.maximum(residuals,0.)/problem['slack_scale']].tolist()))
manifest=dict(schema_version=2,legacy_outer_omega=2*np.pi/planner.track.windows[0].motion.translation_period,level=args.level,method=args.method,n=planner.n,m=planner.m,nvar=planner.nvar,
 velocity_start=planner.velocity_start,workspace_rows=planner.n*(planner.cfg.nodes_per_piece+1)*3,
 state_scale=problem['state_scale'].tolist(),slack_scale=problem['slack_scale'].tolist(),
 lower_g=problem['lower'].tolist(),upper_g=[None if np.isinf(a) else float(a) for a in problem['upper']],
 solver_options=problem['solver_options'],windows=windows,slices=slices,initials=initials,
 config=asdict(planner.cfg),verification_radius=radius-planner.cfg.witness_slack,
 finite_radius=radius,free_points=planner.free_points,optimization_run=False)
(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print('EXPORTED',args.level,args.method,len(initials),flush=True)
