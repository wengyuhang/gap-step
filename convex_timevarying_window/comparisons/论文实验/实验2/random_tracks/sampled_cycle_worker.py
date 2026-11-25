import argparse,json,time
from pathlib import Path
from bootstrap import load,HERE
scene,higher,common,best,rollout=load()
from sampled_safe_model import Planner
from repair import explicit_signs
import numpy as np

def main():
    a=argparse.ArgumentParser();a.add_argument('--outdir',required=True,type=Path);a.add_argument('--job',required=True);a.add_argument('--initial',required=True,type=Path);a.add_argument('--exact',action='store_true');args=a.parse_args();job=common.read(args.outdir/'jobs'/f'{args.job}.json')
    initials=common.read(args.initial)
    def factory(track,free):
        p=Planner(track,free,pose_function=scene.pose);vectors=[]
        p._prefer_exact_hessian=args.exact
        for row in initials['seeds']:
            x=np.array(row['variables'])
            if not free:x=np.r_[x[:p.n],x[-9*p.m:]]
            vectors.append(x)
        p.initial_for_witnesses=vectors[job['seed']].copy()
        p.seeds=lambda:vectors
        return p
    higher.make_planner=factory
    higher.worker(args.outdir,args.job)
    folder=args.outdir/'plans'/args.job;raw=common.read(folder/'result.json');raw.update(explicit_signs=initials['explicit_signs'],initial_source=initials.get('provenance','independent direction/workspace min-snap QP, then dynamics feasibility time conditioning'),finite_recovery_strategy='failed finite NLP: cold exact Hessian retry; do not repeat nonconverged geometry-safe NLP',initial_finite_hessian='exact' if args.exact else 'limited-memory',initials_file_sha256=common.previous.digest(args.initial))
    if raw['found']:
        from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.trajectory_only import collision
        collision.route_signs=lambda tr:explicit_signs(tr)[0]
        tr,_=scene.build_case(scene.Case(job['level']));g=collision.evaluate(raw['coefficients'],raw['durations'],tr,scene.ASSETS/'meshes');common.save(folder/'assessment.json',g);raw['independent_frame_clear']=all(x['outcome']=='clear' for x in g['checks']);raw['independent_task_passed']=g['ordered_exactly_once'] and g['endpoints_passed']
    common.save(folder/'result.json',raw)
if __name__=='__main__':main()
