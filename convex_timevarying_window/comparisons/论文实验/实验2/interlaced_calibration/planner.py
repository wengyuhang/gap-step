"""Only switch the shared calibration planner's scene; objective is unchanged."""
import argparse
from pathlib import Path
from ..static_calibration import planner as shared
from . import scenario


def solve(case,method,out):
    shared.build_case=scenario.build_case;shared.manifest=scenario.manifest
    return shared.solve(case,method,out)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--layout',required=True);p.add_argument('--method',required=True,choices=('dynamic_togt','dynamic_togt_center'));p.add_argument('--outdir',type=Path,required=True);a=p.parse_args()
    solve(scenario.Case(a.layout,'static'),a.method,a.outdir)
