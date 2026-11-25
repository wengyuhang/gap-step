import argparse,json,time
from pathlib import Path
from bootstrap import load,HERE
scene,higher,common,best,rollout=load()
from xy_model import Planner
from repair import explicit_signs
import numpy as np

def main():
    a=argparse.ArgumentParser();a.add_argument('--outdir',required=True,type=Path);a.add_argument('--job',required=True);args=a.parse_args();job=common.read(args.outdir/'jobs'/f'{args.job}.json')
    initials=common.read(HERE/'results/repair_initials_v3_20261113'/f"{job['level']}.json")
    def factory(track,free):
        p=Planner(track,free,pose_function=scene.pose);vectors=[]
        for row in initials['seeds']:
            x=np.array(row['variables'])
            if free:x=np.r_[x[:p.n],np.zeros(2*p.m),x[-9*p.m:]]
            else:x=np.r_[x[:p.n],x[-9*p.m:]]
            vectors.append(x)
        p.seeds=lambda:vectors
        return p
    higher.make_planner=factory
    higher.worker(args.outdir,args.job)
    folder=args.outdir/'plans'/args.job;raw=common.read(folder/'result.json');raw.update(point_coordinates='direct physical xy in unchanged inset',finite_state_coordinates='direct physical VAJ',explicit_signs=initials['explicit_signs'],initial_source='independent direction/workspace min-snap QP, then dynamics feasibility time conditioning',initials_file_sha256=common.previous.digest(HERE/'results/repair_initials_v3_20261113'/f"{job['level']}.json"))
    if raw['found']:
        from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.trajectory_only import collision
        collision.route_signs=lambda tr:explicit_signs(tr)[0]
        tr,_=scene.build_case(scene.Case(job['level']));g=collision.evaluate(raw['coefficients'],raw['durations'],tr,scene.ASSETS/'meshes');common.save(folder/'assessment.json',g);raw['independent_frame_clear']=all(x['outcome']=='clear' for x in g['checks']);raw['independent_task_passed']=g['ordered_exactly_once'] and g['endpoints_passed']
    common.save(folder/'result.json',raw)
if __name__=='__main__':main()
