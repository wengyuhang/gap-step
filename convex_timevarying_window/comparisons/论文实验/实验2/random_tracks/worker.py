import argparse
from pathlib import Path
from bootstrap import load

def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);p.add_argument('--job',required=True);a=p.parse_args()
    scene,higher,common,best,rollout=load()
    from convex_timevarying_window.comparisons.论文实验.实验2.matched_full_matrix import togt
    from convex_timevarying_window.comparisons.论文实验.实验2.independent_repair.run import attitude_audit,frozen_plan
    job=common.read(a.outdir/'jobs'/f'{a.job}.json')
    if job['method']=='wang2023':
        import json
        folder=a.outdir/'plans'/a.job;folder.mkdir(parents=True)
        with (folder/'progress.jsonl').open('w') as f:
            def progress(row):f.write(json.dumps(row)+'\n');f.flush()
            raw=rollout.solve(job['level'],progress=progress)
        original=raw['config'];raw.update(method='wang2023',job=job,wang_config=original,config=dict(original,radius=.3859447197013662),variables=[],made_at=0.)
        if raw['found']:common.save(folder/'attitude_audit.json',attitude_audit(frozen_plan(raw)))
        common.save(folder/'result.json',raw)
    elif job['method'].startswith('dynamic_togt'):togt.worker(a.outdir,a.job)
    else:higher.worker(a.outdir,a.job)
    path=a.outdir/'plans'/a.job/'result.json';raw=common.read(path);raw.update(scenario_variant=scene.VARIANT,case=dict(layout='random_v2',level=job['level']));common.save(path,raw)
    if job['method'] in ('ours','center','release'):
        track,_=scene.build_case(scene.Case(job['level']))
        planner=higher.make_planner(track,job['method']!='center')
        row=best.assess(a.outdir,job,0,0.,planner,track)
    else:
        row=dict(job=job['id'],method=job['method'],seed=job['seed'],found=raw['found'],passed=raw['found'],source=str(path),objective=raw.get('objective'),nominal_time_s=raw.get('nominal_time_s'),build_s=raw['build_s'],solve_s=raw['solve_s'])
    row.update(level=job['level'])
    common.save(path.parent/'candidate.json',row)
if __name__=='__main__':main()
