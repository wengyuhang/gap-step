"""Isolated native contact geometry probe; teleporting here is NOT race flight."""
import argparse
import json
from pathlib import Path
import numpy as np
from .rpc import Client
from .scene import pose,vector


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',required=True);args=p.parse_args()
    out=Path(args.outdir);out.mkdir(parents=True,exist_ok=False)
    c=Client();c.call('simPause',True);records=[]
    import uuid
    session=uuid.uuid4().hex[:8]
    for axis in (0,1):
        for gap in (.2,.3,.35,.4,.45,.5,.55,.6,.7):
            c.call('reset');c.call('simPause',True)
            center=np.array([100.,100.,-20.]);rod=center.copy();rod[axis]+=gap
            name=c.call('simSpawnObject',f'BodyContactProbe_{axis}_{int(gap*1000)}_{session}','Cylinder',pose(rod),vector([.02,.02,2]),False,False)
            outside=center.copy();outside[axis]-=2.
            # Ignore collisions only for relocation to the isolated test site;
            # sweep toward the test rod over a short, otherwise empty segment.
            c.call('simSetVehiclePose',pose(outside),True,'RaceDrone')
            c.call('simContinueForTime',.003)
            c.call('simSetVehiclePose',pose(center),False,'RaceDrone')
            c.call('simContinueForTime',.03)
            contact=c.call('simGetCollisionInfo','RaceDrone')
            records.append(dict(axis=axis,rod_center_distance_m=gap,rod_radius_m=.01,collision=contact))
            (out/'result.json').write_text(json.dumps(dict(status='probe_running_not_flight',records=records),indent=2))
            print(axis,gap,contact['has_collided'],contact['object_name'],flush=True)
            c.call('simDestroyObject',name)
    c.call('reset');c.call('simPause',True)
    (out/'result.json').write_text(json.dumps(dict(status='isolated_native_contact_probe_not_flight',records=records),indent=2))


if __name__=='__main__':main()
