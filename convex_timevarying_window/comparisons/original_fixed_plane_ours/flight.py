"""Use existing free_warm PX4 entry with the previous turn's exact world."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import shutil
import sys
from .adapter import Worker
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case, build_case, launch, manifest
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.reference import Reference


def neutral_vary(track, phase, layout, rate, translation, rotation):
    if (phase,layout,rate,translation,rotation)!=(0.,0.,1.,1.,1.):
        raise ValueError('Frozen physical scene must not receive legacy motion scaling')
    return track


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--frequency', type=float, required=True)
    p.add_argument('--amplitude', type=float, required=True)
    p.add_argument('--phase', type=float, required=True)
    p.add_argument('--outdir', type=Path, required=True)
    a = p.parse_args(); case = Case(a.frequency,a.amplitude,a.phase); out = a.outdir.resolve()
    if out.exists(): raise FileExistsError(out)
    from convex_timevarying_window.wang_paper_aligned_mpc import px4 as entry
    from convex_timevarying_window.dynamic_gate_mppi_v3 import ctbr_probe
    entry.Worker = Worker; entry.Reference = Reference
    entry.build_track = lambda: build_case(case); entry.vary_track = neutral_vary
    ctbr_probe.launch_variant = launch
    sys.argv = [entry.__name__, '--outdir',str(out),'--max-flight-time','60','--condensed',
                '--motion-rate','1','--translation-scale','1','--rotation-scale','1','--phase','0','--layout','0']
    try: entry.main()
    finally:
        if out.exists():
            (out/'scene.json').write_text(json.dumps(manifest(case),indent=2)+'\n')
            if (out/'result.json').exists():
                shutil.copyfile(out/'result.json',out/'native_result.json')
                r = json.loads((out/'result.json').read_text())
                r.update(case=asdict(case), scenario_adapter='original_fixed_plane_ours',
                         radius=r['config']['radius'],frame_thickness=r['config']['frame_thickness'],
                         neutral_legacy_motion_fields=True,
                         tested_variant='existing N2 M4 free_warm, scene queries adapted, common timestamp-correct reference')
                (out/'result.json').write_text(json.dumps(r,indent=2)+'\n')


if __name__=='__main__': main()
