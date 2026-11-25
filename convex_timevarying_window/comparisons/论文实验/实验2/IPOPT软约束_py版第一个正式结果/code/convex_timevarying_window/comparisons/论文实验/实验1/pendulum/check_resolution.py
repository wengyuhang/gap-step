"""Independent finer physics check for the first actual frame-contact case."""
import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);a=p.parse_args()
    from . import scenario,flight
    original=scenario.write_world;scenario.PHYSICS_STEP=.0005
    def write(track,assets,out):
        path=original(track,assets,out);tree=ET.parse(path)
        for sensor in tree.findall("world/model/link/sensor[@type='contact']"):sensor.find('update_rate').text='2000'
        tree.write(path,encoding='unicode');return path
    scenario.write_world=write
    sys.argv=['pendulum_resolution','--angle','60','--method','high_mpc_author','--outdir',str(a.outdir)]
    flight.main()

if __name__=='__main__':main()
