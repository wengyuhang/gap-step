"""One unchanged repaired-planner candidate on a frozen expanded track."""
import argparse,json
from pathlib import Path
import numpy as np
from bootstrap import load
scene,higher,common,_,_=load()
from repair import Planner,explicit_signs

def main():
 a=argparse.ArgumentParser();a.add_argument('--outdir',type=Path,required=True);a.add_argument('--job',required=True);a.add_argument('--initial',type=Path,required=True);q=a.parse_args();job=common.read(q.outdir/'jobs'/f'{q.job}.json');initials=common.read(q.initial)
 def factory(track,free):
  p=Planner(track,free,pose_function=scene.pose);vectors=[]
  for row in initials['seeds']:
   x=np.array(row['variables']);vectors.append(x if free else np.r_[x[:p.n],x[-9*p.m:]])
  p.seeds=lambda:vectors
  return p
 higher.make_planner=factory;higher.worker(q.outdir,q.job);folder=q.outdir/'plans'/q.job;raw=common.read(folder/'result.json');tr,_=scene.build_case(scene.Case(job['level']));raw.update(case=dict(layout='expanded_serpentine',level=job['level']),gate_count=len(tr.windows),all_seven_joint=len(tr.windows)==7,all_windows_joint=True,initials_file_sha256=common.previous.digest(q.initial),scenario_manifest_sha256=common.previous.digest(scene.HERE/'results/cases_v1'/f"{job['level']}.json"),explicit_signs=initials['explicit_signs'],independent_frame_clear=False,independent_ground_clear=False,independent_task_passed=False,qualified=False)
 if raw['found']:
  assert len(raw['durations'])==len(tr.windows)+1
  from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.trajectory_only import collision
  collision.route_signs=lambda tr:explicit_signs(tr)[0]
  g=collision.evaluate(raw['coefficients'],raw['durations'],tr,scene.ASSETS/'meshes');common.save(folder/'assessment.json',g);raw.update(independent_frame_clear=all(c['outcome']=='clear' for c in g['checks']),independent_ground_clear=all(c['outcome']=='clear' for c in g['ground']),independent_task_passed=g['ordered_exactly_once'] and g['endpoints_passed'],qualified=g['passed'],independent_verdict=g['verdict'],trajectory_distance_m=g['trajectory_distance_m'],independent_check_s=g['check_s'])
 common.save(folder/'result.json',raw)
if __name__=='__main__':main()
