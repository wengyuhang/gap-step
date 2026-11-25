"""Diagnostic only: release one known center curve, never add it to formal scores."""
from pathlib import Path
import argparse,json,sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from bootstrap import load
scene,higher,common,_,_=load()
from repair import Planner,explicit_signs

def main():
 a=argparse.ArgumentParser();a.add_argument('--outdir',type=Path,required=True);a.add_argument('--initial',type=Path,required=True);a.add_argument('--level',required=True);q=a.parse_args();q.outdir.mkdir(exist_ok=False);(q.outdir/'jobs').mkdir();job=dict(id='release_diagnostic',method='ours',seed=0,level=q.level);common.save(q.outdir/'jobs/release_diagnostic.json',job);initial=common.read(q.initial)
 def factory(track,free):
  p=Planner(track,free,pose_function=scene.pose);p.seeds=lambda:[np.array(initial['seeds'][0]['variables'])];return p
 higher.make_planner=factory;higher.worker(q.outdir,job['id']);folder=q.outdir/'plans'/job['id'];raw=common.read(folder/'result.json');raw.update(diagnostic_only=True,formal_comparison_candidate=False,optimized_initial_solution_used=True,center_solution_used=True,center_result_initialization=True,source_center=initial['source_center'],case=dict(layout='expanded_serpentine',level=q.level),qualified=False)
 if raw['found']:
  from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.trajectory_only import collision
  collision.route_signs=lambda tr:explicit_signs(tr)[0];tr,_=scene.build_case(scene.Case(q.level));g=collision.evaluate(raw['coefficients'],raw['durations'],tr,scene.ASSETS/'meshes');common.save(folder/'assessment.json',g);raw.update(qualified=g['passed'],independent_verdict=g['verdict'],trajectory_distance_m=g['trajectory_distance_m'])
 common.save(folder/'result.json',raw);print(q.level,raw['found'],raw.get('nominal_time_s'),raw.get('qualified'),flush=True)
if __name__=='__main__':main()
