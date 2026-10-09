"""Original current method or single-mechanism ablation, same physical scene."""
import argparse
from dataclasses import asdict,replace
import json
import os
from pathlib import Path
import sys
from .worker import Worker
from .planner import Config
from convex_timevarying_window.race_to_go_mpc.reference import Reference
from convex_timevarying_window.comparisons.original_fixed_plane_ours.flight import neutral_vary
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case,build_case,launch,manifest


def variant_config(variant):
    if variant not in ('ours','center','no_soft'):raise ValueError(variant)
    return replace(Config(),soft_clearance_weight=0.) if variant=='no_soft' else Config()


def main():
    p=argparse.ArgumentParser();p.add_argument('--method',choices=('ours','center','no_soft'),required=True)
    for k in ('frequency','amplitude','phase'):p.add_argument('--'+k,type=float,required=True)
    p.add_argument('--outdir',type=Path,required=True);a=p.parse_args();out=a.outdir.resolve()
    if out.exists():raise FileExistsError(out)
    case=Case(a.frequency,a.amplitude,a.phase);os.environ['EXPERIMENT_VARIANT']=a.method
    from . import px4 as entry
    from convex_timevarying_window.dynamic_gate_mppi_v3 import ctbr_probe
    refs=[]
    def reference(start):
        ref=Reference(start);refs.append(ref);return ref
    entry.Reference=reference;entry.Worker=Worker;entry.PlannerConfig=lambda:variant_config(a.method)
    entry.build_track=lambda:build_case(case);entry.vary_track=neutral_vary;ctbr_probe.launch_variant=launch
    sys.argv=[entry.__name__,'--outdir',str(out),'--max-flight-time','60','--condensed',
        '--motion-rate','1','--translation-scale','1','--rotation-scale','1','--phase','0','--layout','0']
    if a.method=='center':sys.argv.append('--center-only')
    try:entry.main()
    finally:
        if out.exists():
            (out/'scene.json').write_text(json.dumps(manifest(case),indent=2)+'\n')
            if refs:(out/'shared_passage_contracts.json').write_text(json.dumps(refs[0].contract_log,indent=2)+'\n')
            if (out/'result.json').exists():
                r=json.loads((out/'result.json').read_text());r.update(case=asdict(case),comparison_variant=a.method,
                    method='center_continuation_'+a.method,radius=r['config']['radius'],frame_thickness=r['config']['frame_thickness'],
                    post_trajectory_validation_enabled=False)
                (out/'result.json').write_text(json.dumps(r,indent=2)+'\n')


if __name__=='__main__':main()



