"""Scene-only injection into the existing four-start global solvers."""
import argparse
from pathlib import Path
import numpy as np
from . import scenario


def nominal_audit(forward,track):
    curve=forward.trajectory;tt=np.linspace(0,curve.total_time,int(np.ceil(curve.total_time/.001))+1);P=curve.evaluate(tt,0);rows=[]
    for w in track.windows:
        centers=w.center0+w.motion.translation(tt)[0];q=(P-centers)@w.R
        distance=np.hypot(w.aperture.boundary_distance(q[:,:2]),q[:,2])-.3859447197013662
        rows.append(dict(name=w.name,min_zero_thickness_sphere_clearance_m=float(distance.min()),at_s=float(tt[distance.argmin()])))
    return dict(evidence='1ms moving-frame nominal sphere proxy, not a physical verdict or certificate',per_window=rows)


def main():
    p=argparse.ArgumentParser();p.add_argument('--layout',choices=['original'],required=True);p.add_argument('--level',choices=list(scenario.LEVELS),required=True)
    p.add_argument('--method',choices=['ours','center','dynamic_togt','dynamic_togt_center'],required=True);p.add_argument('--outdir',type=Path,required=True)
    p.add_argument('--seconds-per-seed',type=float,default=180.);a=p.parse_args()
    if a.method in ('ours','center'):
        from .. import plan as entry,planner
        entry.build_case=scenario.build_case;entry.manifest=scenario.manifest;planner.pose=scenario.pose
        entry.main()
    else:
        from ..static_calibration import planner
        planner.build_case=scenario.build_case;planner.manifest=scenario.manifest;planner.nominal_audit=nominal_audit
        planner.solve(scenario.Case(a.layout,a.level),a.method,a.outdir)


if __name__=='__main__':main()
