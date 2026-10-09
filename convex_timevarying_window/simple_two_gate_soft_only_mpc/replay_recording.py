"""Visual-only Gazebo playback of recorded ULog truth, not a new flight."""
import argparse
import json
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET
import numpy as np
from pyulog import ULog
from scipy.spatial.transform import Rotation, Slerp
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.audit import truth


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--recording', type=Path, required=True)
    parser.add_argument('--outdir', type=Path, required=True)
    parser.add_argument('--paused', action='store_true', help='Wait for pose verification before playback')
    args = parser.parse_args()
    source = args.recording.resolve(); out = args.outdir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    result = json.loads((source/'result.json').read_text())
    duration = result['flight_time_s']
    ts, positions = truth(source, duration)
    attitude = ULog(str(source/'flight.ulg')).get_dataset('vehicle_attitude_groundtruth').data
    at = attitude['timestamp']*1e-6-30.
    quats = np.column_stack([attitude[k] for k in ('q[1]', 'q[2]', 'q[3]', 'q[0]')])
    rot = Slerp(at, Rotation.from_quat(quats))(ts).as_matrix()
    ned_to_enu = np.array([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]])
    rot = Rotation.from_matrix(ned_to_enu@rot@np.diag([1.,-1.,-1.]))
    q = rot.as_quat()[:, [3,0,1,2]]
    velocity = np.gradient(positions, ts, axis=0)
    rates = (rot[:-1].inv()*rot[1:]).as_rotvec()/np.diff(ts)[:,None]
    rates = np.vstack([rates,rates[-1]])
    np.savetxt(out/'recording.csv', np.c_[ts,positions,velocity,q,rates], delimiter=',',
               header='time,x,y,z,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz', comments='')
    tree = ET.parse(source/'world.sdf'); world = tree.getroot().find('world')
    for filename, name in [('gz-sim-physics-system','Physics'),
                           ('gz-sim-user-commands-system','UserCommands'),
                           ('gz-sim-scene-broadcaster-system','SceneBroadcaster'),
                           ('gz-sim-contact-system','Contact')]:
        ET.SubElement(world,'plugin',filename=filename,name='gz::sim::systems::'+name)
    plugin = ET.SubElement(world,'plugin',filename='libRecordedPoseReplay.so',
                           name='RecordedPoseReplay')
    for key,val in [('target_model','x500_0'),('motion_start_time','30'),
                    ('reference_environment_variable','REPLAY_REFERENCE'),('release_udp_port','18689')]:
        ET.SubElement(plugin,key).text=val
    include=ET.SubElement(world,'include')
    ET.SubElement(include,'uri').text='model://x500'
    ET.SubElement(include,'name').text='x500_0'
    ET.SubElement(include,'static').text='true'
    ET.SubElement(include,'pose').text='-16 4 3.2 0 0 0'
    tree.write(out/'world.sdf',encoding='unicode')
    (out/'manifest.json').write_text(json.dumps(dict(source=str(source),duration=duration,
        mode='recorded truth STATIC visual model; no PX4, dynamics or collision validation',
        samples=len(ts),sample_step=.004,gate_motion_start=30),indent=2))
    import os
    display=os.environ['DISPLAY']
    subprocess.run(['xhost','+si:localuser:root'],check=True,capture_output=True)
    container='convex_seven_togt_experiment'
    subprocess.run(['docker','rm','-f',container],check=True,capture_output=True)
    assets=Path('convex_dynamic_seven_window_gazebo').resolve()
    playback_plugin=Path(__file__).parent.resolve()/'replay_plugin'
    command=['docker','run','-d','--name',container,'--network','host',
        '-e','GZ_PARTITION=convex_seven_togt_experiment_partition',
        '-e','GZ_SIM_SYSTEM_PLUGIN_PATH=/playback_plugin/build:/workspace/build',
        '-e','GZ_SIM_RESOURCE_PATH=/workspace:/opt/px4-gazebo/share/gz/models',
        '-e','REPLAY_REFERENCE=/recording/recording.csv','-e',f'DISPLAY={display}',
        '-e','QT_X11_NO_MITSHM=1','-v','/tmp/.X11-unix:/tmp/.X11-unix:rw',
        '-v',f'{assets}:/workspace:ro','-v',f'{out}:/recording:ro',
        '-v',f'{playback_plugin}:/playback_plugin:ro',
        '--entrypoint','bash','px4io/px4-sitl-gazebo:latest','-lc',
        'gz sim -s -v 2 /recording/world.sdf >/tmp/gazebo.log 2>&1 & '
        'exec gz sim -g -v 3 >/tmp/gazebo_gui.log 2>&1']
    subprocess.run(command,check=True)
    # Start paused so camera setup finishes before playback begins.
    def service(name,kind,request):
        return subprocess.run(['docker','exec',container,'gz','service','-s',name,
            '--reqtype',kind,'--reptype','gz.msgs.Boolean','--timeout','2000','--req',request],
            capture_output=True,text=True)
    for _ in range(30):
        answer=service('/gui/follow','gz.msgs.StringMsg','data: "x500_0"')
        if 'data: true' in answer.stdout: break
        time.sleep(1)
    else: raise RuntimeError('GUI follow service unavailable; playback remains paused')
    service('/gui/follow/offset','gz.msgs.Vector3d','x: -8 y: -8 z: 5')
    if args.paused:
        print('PLAYBACK_PAUSED_FOR_POSE_VERIFICATION',flush=True)
        return
    answer=service('/world/convex_seven_dynamic_px4_togt/control',
                   'gz.msgs.WorldControl','pause: false')
    if 'data: true' not in answer.stdout: raise RuntimeError('Could not start playback')
    print('PLAYBACK_STARTED: 30 s setup, then recorded flight of',duration,'s',flush=True)


if __name__=='__main__': main()
