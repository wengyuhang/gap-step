"""Same six-flight protocol with the compact scene injected explicitly."""
from ..static_calibration import run as shared
from . import scenario

if __name__=='__main__':
    shared.MODULE='convex_timevarying_window.comparisons.论文实验.实验2.compact_calibration'
    shared.manifest=scenario.manifest
    shared.main()
