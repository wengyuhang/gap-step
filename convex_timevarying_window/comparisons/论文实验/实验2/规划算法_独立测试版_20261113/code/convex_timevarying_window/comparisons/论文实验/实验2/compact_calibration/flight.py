"""Frozen compact reference, identical MPC/PX4/x500 and physical audit."""
from ..static_calibration import flight as shared
from . import scenario

if __name__=='__main__':
    shared.scenario=scenario
    shared.main()
