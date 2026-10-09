from pathlib import Path
from convex_timevarying_window.comparisons.large_window_sweep import run as paired
from 复现.polynomial_dynamic_racing_2023.px4_original_track_legacy.run import ROOT,digest
from .scenario import build_case,write_world,layout_check
from .audit import audit


def sources():
    values=paired.sources()
    for p in Path(__file__).parent.rglob('*'):
        if 'results' not in p.parts and p.suffix in ('.py','.md'):
            values[str(p.relative_to(ROOT))]=digest(p)
    return values


def main():
    assert layout_check()['minimum_gate_swept_sphere_gap_m']>0
    assert layout_check()['straight_return_vs_other_gates_min_gap_m']>0
    paired.main(scene_build=build_case,world_writer=write_world,source_fn=sources,audit_fn=audit,
        flight_module='convex_timevarying_window.comparisons.spaced_large_window_sweep.flight',
        protocol_path=Path(__file__).with_name('PROTOCOL.md'))


if __name__=='__main__':main()
