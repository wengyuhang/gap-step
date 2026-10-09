"""Fixed-plane adapter for the independent no-local-stop planner."""
import argparse
import importlib
import json
from pathlib import Path
import shutil
import sys

from convex_timevarying_window.comparisons.fixed_plane_wang.scenario import Case,build_case,neutral_vary,launch,description,check_world
from convex_timevarying_window.comparisons.checked_soft_racing.run import save


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--method',choices=('ours',),default='ours')
    p.add_argument('--frequency',type=float,required=True)
    p.add_argument('--amplitude',type=float,required=True)
    p.add_argument('--phase',type=float,required=True)
    p.add_argument('--outdir',type=Path,required=True)
    a = p.parse_args();case = Case(a.frequency,a.amplitude,a.phase)
    out = a.outdir.resolve()
    if out.exists():
        raise FileExistsError(out)
    if a.method=='ours':
        entry = importlib.import_module('convex_timevarying_window.wang_center_seed_mpc.px4')
        session = importlib.import_module('convex_timevarying_window.dynamic_gate_mppi_v3.ctbr_probe')
        session.launch_variant = launch
        flags = ['--condensed']
    else:
        entry = importlib.import_module('convex_timevarying_window.receding_gate_racing.px4_experiment')
        entry.launch_variant = launch
        flags = ['--launch','--paper-polynomial','--tracking-preview','.05']
    entry.build_track = lambda: build_case(case)
    entry.vary_track = neutral_vary
    # Ours historically defaults to a hard rotating case; explicitly neutralize
    # those CLI fields because the new input is already in physical units.
    sys.argv = [entry.__name__,'--outdir',str(out),'--max-flight-time','60',
                '--motion-rate','1','--translation-scale','1','--rotation-scale','1','--phase','0',*flags]
    try:
        entry.main()
    finally:
        if out.exists():
            save(out/'scenario.json',description(case))
            if (out/'world.sdf').exists():
                save(out/'scene_parity.json',check_world(build_case(case)[0],out/'world.sdf'))
            if (out/'result.json').exists():
                shutil.copyfile(out/'result.json',out/'native_result.json')
                raw = json.loads((out/'result.json').read_text())
                raw.update(scenario_adapter='fixed_plane_wang',scenario=description(case),
                    neutral_legacy_motion_fields=True,
                    scene_audit_entry='convex_timevarying_window.comparisons.fixed_plane_wang.audit')
                save(out/'result.json',raw)


if __name__=='__main__':
    main()
