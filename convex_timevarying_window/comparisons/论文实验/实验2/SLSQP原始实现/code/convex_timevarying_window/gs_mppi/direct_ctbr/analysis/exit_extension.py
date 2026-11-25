"""Read-only, matched geometric diagnostic; not a race-time objective.

For every measured centre crossing, report the maximum forward distance from
the moving gate plane during the next two seconds. This is not extra path
length, nor proof that all of it can be eliminated under the vehicle limits.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from pyulog import ULog
from scipy.spatial.transform import Rotation
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.receding_gate_racing.scenario import vary_track
from convex_timevarying_window.three_phase_mppi.report import continuous_quaternions
from convex_timevarying_window.dynamic_gate_mppi_v3.geometry import Geometry


def diagnose(run):
    result=json.loads((run/'result.json').read_text())
    track,_=build_track()
    vary_track(track,result['phase'],result.get('layout',0.),result['motion_rate'],
               result['translation_scale'],result['rotation_scale'])
    truth=ULog(str(run/'flight.ulg'))
    positions=truth.get_dataset('vehicle_local_position_groundtruth').data
    attitudes=truth.get_dataset('vehicle_attitude_groundtruth').data
    q=continuous_quaternions(np.column_stack([attitudes[k] for k in ('q[1]','q[2]','q[3]','q[0]')]))
    time=np.arange(0.,result['last_time_s'],.01);absolute=time+30.
    rotation=Rotation.from_quat(np.column_stack([np.interp(absolute,attitudes['timestamp']*1e-6,q[:,i]) for i in range(4)]))
    p=np.column_stack([np.interp(absolute,positions['timestamp']*1e-6,positions[k]) for k in ('x','y','z')])
    p+=rotation.apply(np.tile([0.,0.,-.24],(len(time),1)))
    p=p[:,[1,0,2]]*[1.,1.,-1.]
    geometry=Geometry(track);centres,rotations=geometry.poses(time)
    previous=track.start;signs=[]
    for index in track.order:
        c,r,*_=track.windows[index].state_at(0.)
        signs.append(-np.sign(r[:,2]@(previous-c)));previous=c
    output=[]
    for crossing in result.get('center_crossings',result['crossings']):
        gate=crossing['gate'];index=track.order[gate];at=crossing['time']
        mask=(time>=at)&(time<=at+2.)
        local=np.einsum('tji,tj->ti',rotations[mask,index],p[mask]-centres[mask,index])
        forward=signs[gate]*local[:,2]
        slab=geometry.config.radius+geometry.config.error_margin+geometry.config.frame_thickness
        exits=np.flatnonzero(forward>slab)
        output.append(dict(gate=gate+1,center_event_s=at,
                           sampled_envelope_exit_s=float(time[mask][exits[0]]) if len(exits) else None,
                           max_forward_plane_distance_2s_m=float(forward.max()),
                           interval_end_s=float(time[mask][-1])))
    return dict(run=str(run.resolve()),gates=output)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('runs',type=Path,nargs='+');parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=dict(definition=__doc__,runs=[diagnose(run) for run in args.runs])
    with args.output.open('x') as stream:json.dump(result,stream,indent=2)
    print(json.dumps(result,indent=2))
