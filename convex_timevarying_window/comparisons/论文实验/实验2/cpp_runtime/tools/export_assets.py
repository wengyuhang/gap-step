"""Freeze existing mesh triangles and world overlays without changing geometry."""
import argparse,json
from pathlib import Path
import trimesh
from convex_timevarying_window.comparisons.论文实验.实验2.higher_swing import scenario
from convex_timevarying_window.single_gate_stop_racing.planner import route_signs
from convex_timevarying_window.single_gate_stop_racing.metric_session import metric_scale
from convex_timevarying_window.comparisons.论文实验.实验2.basic_group import scenario as world
parser=argparse.ArgumentParser();parser.add_argument('--outdir',type=Path,required=True);a=parser.parse_args()
out=a.outdir.resolve();out.mkdir(parents=True,exist_ok=False)
for level in scenario.LEVELS:
 track,_=scenario.build_case(scenario.Case(level=level));folder=out/level;folder.mkdir()
 target=scenario.write_world(track,scenario.ASSETS,folder)
 rows=[]
 for window,sign,topics in zip(track.windows,route_signs(track),world.contact_groups(track)):
  mesh=trimesh.load_mesh(scenario.ASSETS/'meshes'/f'gate_{window.name}_sleeve.stl')
  ap=window.aperture;circle=ap.kind=='circle'
  rows.append(dict(name=window.name,center=window.center0.tolist(),rotation=window.R.tolist(),circle=circle,
   opening_radius=ap.radius if circle else None,vertices=None if circle else ap.physical_vertices.tolist(),
   swing=window.motion.kind=='swing',frequency=window.motion.frequency,amplitude=window.motion.amplitude,
   length=window.motion.length,phase=window.motion.phase,sign=float(sign),mesh=mesh.triangles.tolist(),contact_topics=topics))
 (folder/'scene.json').write_text(json.dumps(dict(level=level,windows=rows,start=track.start.tolist(),goal=track.goal.tolist(),
  contact_topics=world.contact_topics(track),assets=str(scenario.ASSETS),world=str(target),
  scene=scenario.manifest(scenario.Case(level=level)),metric_scale=metric_scale(47.397971057728974).tolist()),indent=2)+'\n')
 print('ASSETS',level,flush=True)
