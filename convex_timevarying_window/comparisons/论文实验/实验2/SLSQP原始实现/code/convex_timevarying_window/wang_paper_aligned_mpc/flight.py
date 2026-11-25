"""Identical fixed-plane course, PX4 and MPC for all matched variants."""
import argparse
import json
from pathlib import Path
import shutil
import sys
from convex_timevarying_window.comparisons.fixed_plane_wang.scenario import (
    Case, build_case, neutral_vary, launch, description, check_world)
from convex_timevarying_window.comparisons.checked_soft_racing.run import save


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--method', choices=('wang', 'wang_warm', 'free_cold', 'free_warm'), required=True)
    p.add_argument('--frequency', type=float, required=True)
    p.add_argument('--amplitude', type=float, required=True)
    p.add_argument('--phase', type=float, required=True)
    p.add_argument('--outdir', type=Path, required=True)
    a = p.parse_args(); case = Case(a.frequency, a.amplitude, a.phase)
    out = a.outdir.resolve()
    if out.exists(): raise FileExistsError(out)
    from . import px4 as entry
    from convex_timevarying_window.dynamic_gate_mppi_v3 import ctbr_probe
    ctbr_probe.launch_variant = launch
    entry.build_track = lambda: build_case(case)
    entry.vary_track = neutral_vary
    flags = (['--center-only', '--warm-center'] if a.method == 'wang_warm' else
             ['--center-only'] if a.method == 'wang' else (['--cold-state'] if a.method == 'free_cold' else []))
    if a.method.startswith('wang'):
        from 复现.polynomial_dynamic_racing_2023.paper_aligned.planner import Config
        entry.PlannerConfig = Config
    sys.argv = [entry.__name__, '--outdir', str(out), '--max-flight-time', '60',
                '--motion-rate', '1', '--translation-scale', '1', '--rotation-scale', '1',
                '--phase', '0', '--condensed', *flags]
    try:
        entry.main()
    finally:
        if out.exists():
            save(out/'scenario.json', description(case))
            if (out/'world.sdf').exists(): save(out/'scene_parity.json', check_world(build_case(case)[0], out/'world.sdf'))
            if (out/'result.json').exists():
                shutil.copyfile(out/'result.json', out/'native_result.json')
                raw = json.loads((out/'result.json').read_text())
                raw.update(scenario_adapter='fixed_plane_wang', scenario=description(case),
                           comparison_variant=a.method, neutral_legacy_motion_fields=True,
                           scene_audit_entry='convex_timevarying_window.comparisons.fixed_plane_wang.audit')
                save(out/'result.json', raw)


if __name__ == '__main__': main()
