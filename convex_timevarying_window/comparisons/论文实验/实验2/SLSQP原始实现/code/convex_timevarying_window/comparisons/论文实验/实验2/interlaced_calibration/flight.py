"""Same frozen reference/MPC/PX4 and raw audits, actual interlaced scene."""
from ..static_calibration import flight as shared
from . import scenario

if __name__=='__main__':
    shared.scenario=scenario
    shared.main()
