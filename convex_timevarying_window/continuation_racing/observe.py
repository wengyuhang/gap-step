"""Read-only Gazebo odometry subscription to keep lazy truth publication live.

No sensor values, controller inputs, world state, or flight parameters are set.
Only follows the dedicated experiment container while the given parent lives.
"""
import argparse
import os
import subprocess
import time


def main():
    p=argparse.ArgumentParser();p.add_argument('--until-pid',type=int,required=True);a=p.parse_args()
    identity=None;reader=None
    try:
        while True:
            try:os.kill(a.until_pid,0)
            except ProcessLookupError:break
            status=subprocess.run(['docker','inspect','--format','{{.Id}} {{.State.Running}}','convex_seven_togt_experiment'],capture_output=True,text=True)
            value=status.stdout.strip()
            if status.returncode==0 and value.endswith(' true') and value!=identity:
                if reader is not None:reader.terminate()
                reader=subprocess.Popen(['docker','exec','-e','GZ_PARTITION=convex_seven_togt_experiment_partition',
                    'convex_seven_togt_experiment','gz','topic','-e','-t','/model/x500_0/odometry_with_covariance'],
                    stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
                identity=value;print('READ_ONLY_ODOMETRY_SUBSCRIBER',value,flush=True)
            time.sleep(.5)
    finally:
        if reader is not None:reader.terminate()


if __name__=='__main__':main()
