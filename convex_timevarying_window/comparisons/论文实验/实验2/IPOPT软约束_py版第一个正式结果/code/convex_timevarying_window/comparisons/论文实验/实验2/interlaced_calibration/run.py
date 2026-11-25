"""Use exactly the same frozen six-flight coordinator for corrected scenes."""
from ..static_calibration import run as shared
from . import scenario

if __name__=='__main__':
    shared.MODULE='convex_timevarying_window.comparisons.论文实验.实验2.interlaced_calibration'
    shared.manifest=scenario.manifest
    shared.main()
