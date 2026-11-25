"""Apply the same held-out course changes to the planner and Gazebo."""
from dataclasses import replace
from pathlib import Path
import os
import subprocess
import time
import xml.etree.ElementTree as ET
import numpy as np


def vary_track(track, phase=0., layout=0., motion_rate=1., translation_scale=1., rotation_scale=1.):
    if not np.isfinite(motion_rate) or motion_rate <= 0:
        raise ValueError('motion_rate must be finite and positive')
    for scale in (translation_scale,rotation_scale):
        if not np.isfinite(scale) or scale<0:raise ValueError('amplitude scales must be finite and nonnegative')
    for i,w in enumerate(track.windows):
        w.motion=replace(w.motion,phase=w.motion.phase+phase,
                         translation_amplitude=w.motion.translation_amplitude*translation_scale,
                         rotation_amplitude=w.motion.rotation_amplitude*rotation_scale,
                         translation_period=w.motion.translation_period/motion_rate,
                         rotation_period=w.motion.rotation_period/motion_rate)
        w.center0=w.center0+layout*np.array([np.sin(1.7*i),np.cos(2.1*i),.2*np.sin(i)])
    return track


def write_variant_world(track, assets, outdir):
    """A result-local world overlay, leaving all frozen assets untouched."""
    source=assets/'convex_seven_dynamic_px4_togt.sdf'
    tree=ET.parse(source)
    models=[m for m in tree.findall('.//world/model') if m.get('name','').startswith('gate_')]
    if len(models)!=len(track.windows): raise ValueError('world and track gate counts differ')
    for model,w in zip(models,track.windows):
        plugin=model.find("plugin[@name='convex_dynamic_course::PeriodicGateMotion']")
        plugin.find('phase').text=str(w.motion.phase)
        plugin.find('translation_period').text=str(w.motion.translation_period)
        plugin.find('rotation_period').text=str(w.motion.rotation_period)
        plugin.find('translation_amplitude').text=' '.join(map(str,w.motion.translation_amplitude))
        plugin.find('rotation_amplitude').text=' '.join(map(str,w.motion.rotation_amplitude))
        plugin.find('center0').text=' '.join(map(str,w.center0))
        c,R,*_=w.state_at(0.)
        angles=w.angles0_rpy+w.motion.rotation(0.)[0]
        model.find('pose').text=' '.join(map(str,np.r_[c,angles]))
    target=(outdir/'world.sdf').resolve()
    tree.write(target,encoding='unicode')
    return target


def launch_variant(track, assets, outdir):
    target=write_variant_world(track,assets,outdir)
    container='convex_seven_togt_experiment'
    gui=os.environ.get('GZ_SHOW_GUI')=='1'
    if gui:
        display=os.environ.get('DISPLAY')
        if not display or not Path(f"/tmp/.X11-unix/X{display.rsplit(':',1)[-1].split('.')[0]}").is_socket():
            raise RuntimeError('Gazebo GUI requested but DISPLAY/X11 socket is unavailable')
        subprocess.run(['xhost','+si:localuser:root'],check=True,stdout=subprocess.DEVNULL)
    # This is the same dedicated experiment target as the existing launcher.
    subprocess.run(['docker','rm','-f',container],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    time.sleep(2.)
    command=['docker','run','-d','--name',container,'--network','host',
             '--env','GZ_PARTITION=convex_seven_togt_experiment_partition',
             '--env','GZ_SIM_SYSTEM_PLUGIN_PATH=/workspace/build',
             '--env','PX4_GZ_STANDALONE=1','--env','PX4_GZ_WORLD=convex_seven_dynamic_px4_togt',
             '--env','PX4_GZ_MODEL_POSE=-16,4,-6,0,0,0',
             '--volume',f'{assets.resolve()}:/workspace:ro',
             '--volume',f'{target}:/workspace/convex_seven_dynamic_px4_togt.sdf:ro']
    if gui:
        command+=['--env',f'DISPLAY={display}','--env','QT_X11_NO_MITSHM=1',
                  '--volume','/tmp/.X11-unix:/tmp/.X11-unix:rw']
        if Path('/dev/dri').exists():command+=['--device','/dev/dri']
    shell='gz sim -s -r -v 2 /workspace/convex_seven_dynamic_px4_togt.sdf >/tmp/gazebo.log 2>&1 & '
    if gui:shell+='gz sim -g -v 3 >/tmp/gazebo_gui.log 2>&1 & '
    shell+='exec /opt/px4-gazebo/bin/px4-gazebo -d .'
    command+=['--workdir','/opt/px4-gazebo','--entrypoint','bash','px4io/px4-sitl-gazebo:latest','-lc',shell]
    subprocess.run(command,check=True,stdout=subprocess.DEVNULL)
    for _ in range(40):
        status=subprocess.run(['docker','exec',container,'/opt/px4-gazebo/bin/px4-commander','status'],capture_output=True)
        if status.returncode==0: time.sleep(2); return
        time.sleep(.5)
    raise RuntimeError('PX4 startup failed')
