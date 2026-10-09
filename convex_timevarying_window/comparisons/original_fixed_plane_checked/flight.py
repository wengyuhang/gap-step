"""Run the original --checked --tracking-preview .05, not Checked-Soft or MPC."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import shutil
import sys
from .adapter import install
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case, build_case, launch, manifest
from convex_timevarying_window.comparisons.original_fixed_plane_ours.flight import neutral_vary


def main():
    parser=argparse.ArgumentParser()
    for name in ('frequency','amplitude','phase'): parser.add_argument('--'+name,type=float,required=True)
    parser.add_argument('--outdir',type=Path,required=True)
    a=parser.parse_args();case=Case(a.frequency,a.amplitude,a.phase);out=a.outdir.resolve()
    if out.exists():raise FileExistsError(out)
    install()
    from convex_timevarying_window.receding_gate_racing import px4_experiment as entry
    entry.build_track=lambda:build_case(case);entry.vary_track=neutral_vary;entry.launch_variant=launch
    sys.argv=[entry.__name__,'--launch','--checked','--tracking-preview','.05',
              '--max-flight-time','60','--outdir',str(out)]
    try:entry.main()
    finally:
        if out.exists():
            (out/'scene.json').write_text(json.dumps(manifest(case),indent=2)+'\n')
            if (out/'result.json').exists():
                shutil.copyfile(out/'result.json',out/'native_result.json')
                r=json.loads((out/'result.json').read_text())
                r.update(case=asdict(case),tested_variant='original Checked; original PVA transport; scene queries only',
                         radius=r['config']['radius'],frame_thickness=r['config']['frame_thickness'])
                (out/'result.json').write_text(json.dumps(r,indent=2)+'\n')


if __name__=='__main__':main()
