"""Independent physics-resolution reruns; never replace the frozen matrix."""
import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

def main():
    p=argparse.ArgumentParser();p.add_argument('--step',type=float,required=True);p.add_argument('--frequency',type=float,required=True);p.add_argument('--amplitude',type=float,required=True);p.add_argument('--method',choices=['ours','center'],required=True);p.add_argument('--outdir',type=Path,required=True);a=p.parse_args()
    if a.step not in (.0005,.00025):raise ValueError('Only frozen independent finer steps')
    from convex_timevarying_window.comparisons.single_gate_swing import scenario,flight
    old=scenario.write_world;scenario.PHYSICS_STEP=a.step
    def write(track,assets,out):
        path=old(track,assets,out);tree=ET.parse(path);tree.find('world/physics/max_step_size').text=str(a.step)
        for sensor in tree.findall("world/model/link/sensor[@type='contact']"):sensor.find('update_rate').text=str(round(1/a.step))
        tree.write(path,encoding='unicode');return path
    scenario.write_world=write
    sys.argv=['resolution_flight','--frequency',str(a.frequency),'--amplitude',str(a.amplitude),'--method',a.method,'--outdir',str(a.outdir)]
    flight.main()

if __name__=='__main__':main()
