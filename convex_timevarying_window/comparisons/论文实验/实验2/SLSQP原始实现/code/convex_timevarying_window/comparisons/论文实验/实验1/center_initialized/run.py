"""Six paired flights for the three predeclared old negative cases."""
import argparse
import json
from pathlib import Path
import sys
from convex_timevarying_window.comparisons.single_gate_swing import run as original

HERE=Path(__file__).resolve().parent
CASES=[(2.5,2.),(3.,2.),(1.5,4.)]


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);a=p.parse_args()
    out=a.outdir.resolve()
    if not out.is_relative_to(HERE.parent/'results'):raise ValueError('Keep new results inside experiment1/results')
    original.MODULE='convex_timevarying_window.center_initialized_minco.flight'
    sys.argv=[original.__name__,'--outdir',str(out),'--cases',*[f'{f}:{amp}' for f,amp in CASES]]
    original.main()
    (out/'method_protocol.md').write_text((HERE/'README.md').read_text())
    from .analyze import analyze
    analyze(out)


if __name__=='__main__':main()
