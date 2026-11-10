"""Offline graphs and route initializer for the unchanged Wang reproduction."""
import argparse,json
from dataclasses import asdict,replace
from pathlib import Path
import numpy as np
from convex_timevarying_window.comparisons.论文实验.实验2.higher_swing import scenario
from convex_timevarying_window.comparisons.论文实验.实验2.wang2023 import rollout
from 复现.wang2023_verified import Planner
from 复现.wang2023_verified import x500
parser=argparse.ArgumentParser();parser.add_argument('--level',choices=scenario.LEVELS,required=True)
parser.add_argument('--outdir',type=Path,required=True);args=parser.parse_args()
out=args.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
track,_=scenario.build_case(scenario.Case(level=args.level));settings=rollout.Settings();cfg=replace(rollout.model_config(settings),compile_functions=False)
reference=rollout.Reference(track,settings,scenario.ASSETS/'meshes')
checks=[]
for gate,window in enumerate(track.windows):
 planner=Planner([lambda at,w=window:scenario.pose(w,at)[0]],cfg,flatness_function=x500.flatness,yaw=lambda at:0*at+np.pi/2)
 planner.fg.save(str(out/f'fg_{gate}.casadi'));planner.details.save(str(out/f'details_{gate}.casadi'))
 head=np.zeros((3,settings.order));head[:,0]=track.start
 seed=reference.seed(planner,head,0.,gate,settings.temporal_scales[0]);params=planner.parameters(head,reference.tails[gate],0.)
 f,g=planner.fg(seed,params)
 checks.append(dict(seed=seed.tolist(),parameters=params.tolist(),objective=float(f),gradient=np.asarray(g).ravel().tolist()))
(out/'manifest.json').write_text(json.dumps(dict(level=args.level,settings=asdict(settings),config=asdict(cfg),
 dimension=planner.dimension,state_end=planner.state_end,start=track.start.tolist(),goal=track.goal.tolist(),
 reference=dict(durations=reference.curve.durations.tolist(),coefficients=reference.curve.coefficients.tolist(),
 node_times=reference.node_times.tolist(),tail_times=reference.tail_times,tail_states=[s.tolist() for s in reference.tails],
 tail_choices=reference.tail_choices,normals=[n.tolist() for n in reference.normals],body_exit_depths=reference.body_depths),checks=checks,
 windows=[dict(center=w.center0.tolist(),rotation=w.R.tolist(),circle=w.aperture.kind=='circle',
  opening_radius=w.aperture.radius if w.aperture.kind=='circle' else None,
  vertices=None if w.aperture.kind=='circle' else w.aperture.physical_vertices.tolist(),
  swing=w.motion.kind=='swing',frequency=w.motion.frequency,amplitude=w.motion.amplitude,length=w.motion.length,phase=w.motion.phase) for w in track.windows]),indent=2)+'\n')
print('WANG_EXPORTED',args.level,flush=True)
