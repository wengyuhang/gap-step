"""Archived-head probe with the hard-actuator planner; never starts Gazebo."""
from . import probe
from .hard_planner import Config,Planner

if __name__=='__main__':
    probe.Config=Config;probe.Planner=Planner
    probe.main()
