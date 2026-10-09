"""Original deadline solver/reference and MPC, with the exact current world."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import shutil
import sys
from .adapter import Worker
from convex_timevarying_window.wang_progress_mpc.reference import Reference
from convex_timevarying_window.comparisons.original_fixed_plane_ours.flight import neutral_vary
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case, build_case, launch, manifest


def main():
    p=argparse.ArgumentParser()
    for name in ('frequency','amplitude','phase'):p.add_argument('--'+name,type=float,required=True)
    p.add_argument('--outdir',type=Path,required=True)
    a=p.parse_args();case=Case(a.frequency,a.amplitude,a.phase);out=a.outdir.resolve()
    if out.exists():raise FileExistsError(out)
    from convex_timevarying_window.wang_free_state_mpc import px4 as entry
    from convex_timevarying_window.dynamic_gate_mppi_v3 import ctbr_probe
    references=[]
    def reference_factory(start):
        ref=Reference(start);references.append(ref);return ref
    entry.Reference=reference_factory;entry.Worker=Worker
    entry.build_track=lambda:build_case(case);entry.vary_track=neutral_vary
    ctbr_probe.launch_variant=launch
    sys.argv=[entry.__name__,'--outdir',str(out),'--max-flight-time','60','--condensed',
              '--motion-rate','1','--translation-scale','1','--rotation-scale','1','--phase','0','--layout','0']
    try:entry.main()
    finally:
        if out.exists():
            (out/'scene.json').write_text(json.dumps(manifest(case),indent=2)+'\n')
            if references:
                (out/'shared_passage_contracts.json').write_text(json.dumps(references[0].contract_log,indent=2)+'\n')
            if (out/'result.json').exists():
                shutil.copyfile(out/'result.json',out/'native_result.json')
                r=json.loads((out/'result.json').read_text())
                r.update(case=asdict(case),method='wang_progress_mpc',algorithm_adapter='common_passage_deadline_v1',
                    scenario_adapter='original_fixed_plane_deadline',radius=r['config']['radius'],
                    frame_thickness=r['config']['frame_thickness'],neutral_legacy_motion_fields=True,
                    tested_variant='existing deadline MINCO MPC; only scene queries adapted')
                (out/'result.json').write_text(json.dumps(r,indent=2)+'\n')


if __name__=='__main__':main()
