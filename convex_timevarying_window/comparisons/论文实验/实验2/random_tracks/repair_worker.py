import argparse,json,time
from pathlib import Path
from bootstrap import load,HERE
scene,higher,common,best,rollout=load()
from repair import Planner,explicit_signs
import numpy as np

def main():
    a=argparse.ArgumentParser();a.add_argument('--outdir',required=True,type=Path);a.add_argument('--job',required=True);a.add_argument('--initial',type=Path);args=a.parse_args();job=common.read(args.outdir/'jobs'/f'{args.job}.json')
    initial_file=args.initial or HERE/'results/repair_initials_v3_20261113'/f"{job['level']}.json"
    initials=common.read(initial_file)
    def factory(track,free):
        p=Planner(track,free,pose_function=scene.pose);vectors=[]
        for row in initials['seeds']:
            x=np.array(row['variables'])
            if not free:x=np.r_[x[:p.n],x[-9*p.m:]]
            vectors.append(x)
        p.seeds=lambda:vectors
        return p
    higher.make_planner=factory
    higher.worker(args.outdir,args.job)
    folder=args.outdir/'plans'/args.job;raw=common.read(folder/'result.json');raw.update(explicit_signs=initials['explicit_signs'],initial_source='independent direction/workspace min-snap QP, then dynamics feasibility time conditioning',initials_file_sha256=common.previous.digest(initial_file))
    if raw['found']:
        from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.trajectory_only import collision
        collision.route_signs=lambda tr:explicit_signs(tr)[0]
        tr,_=scene.build_case(scene.Case(job['level']));g=collision.evaluate(raw['coefficients'],raw['durations'],tr,scene.ASSETS/'meshes');common.save(folder/'assessment.json',g);raw['independent_frame_clear']=all(x['outcome']=='clear' for x in g['checks']);raw['independent_task_passed']=g['ordered_exactly_once'] and g['endpoints_passed']
    common.save(folder/'result.json',raw)
if __name__=='__main__':main()
